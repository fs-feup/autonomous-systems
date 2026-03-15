#include "io/input/keyboard.hpp"

KeyboardInputAdapter::KeyboardInputAdapter()
    : loop_period_ms_(20),
      acceleration_step_(0.05),
      acceleration_decay_step_(0.01),
      steering_step_radians_(M_PI / 64.0),
      max_steering_radians_(M_PI / 8.0),
      help_row_(0),
      status_row_(4),
      running_(false),
      curses_initialized_(false),
      tty_file_(nullptr),
      screen_(nullptr) {}

KeyboardInputAdapter::~KeyboardInputAdapter() { stop(); }

void KeyboardInputAdapter::run() {
  if (running_) {
    return;
  }

  running_ = true;

  tty_file_ = std::fopen("/dev/tty", "r+");
  if (tty_file_ != nullptr) {
    SCREEN* screen = newterm(nullptr, tty_file_, tty_file_);
    if (screen != nullptr) {
      set_term(screen);
      screen_ = static_cast<void*>(screen);
    }
  }

  WINDOW* window = screen_ != nullptr ? stdscr : initscr();
  if (window == nullptr) {
    std::cerr << "Failed to initialize keyboard input." << std::endl;
    if (tty_file_ != nullptr) {
      std::fclose(tty_file_);
      tty_file_ = nullptr;
    }
    running_ = false;
    return;
  }
  curses_initialized_ = true;

  cbreak();
  noecho();
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  curs_set(0);

  mvprintw(help_row_, 0, "w/s: increase/decrease acceleration");
  mvprintw(help_row_ + 1, 0, "a/d: increase/decrease steering angle");
  mvprintw(help_row_ + 2, 0, "Ctrl+C: quit");
  refresh();

  input_loop();
  restore_terminal();
}

void KeyboardInputAdapter::stop() { running_ = false; }

InvictaSimInput KeyboardInputAdapter::get_current_input() const {
  std::lock_guard<std::mutex> lock(input_mutex_);
  return current_input_;
}

void KeyboardInputAdapter::input_loop() {
  double acceleration = 0.0;
  double steering = 0.0;

  while (running_) {
    int key = getch();

    if (key == 'w' || key == 'W') {
      acceleration = std::min(acceleration + acceleration_step_, 1.0);
    } else if (key == 's' || key == 'S') {
      acceleration = std::max(acceleration - acceleration_step_, -1.0);
    } else if (key == ERR) {
      if (acceleration > 0.0) {
        acceleration = std::max(0.0, acceleration - acceleration_decay_step_);
      } else if (acceleration < 0.0) {
        acceleration = std::min(0.0, acceleration + acceleration_decay_step_);
      }
    }

    if (key == 'a' || key == 'A') {
      steering = std::min(steering + steering_step_radians_, max_steering_radians_);
    } else if (key == 'd' || key == 'D') {
      steering = std::max(steering - steering_step_radians_, -max_steering_radians_);
    }

    InvictaSimInput input;
    input.throttle.rear_left = acceleration;
    input.throttle.rear_right = acceleration;
    input.steering = steering;

    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      current_input_ = input;
    }

    mvprintw(status_row_, 0, "Accel: % .2f | Steer: % .2f      ", acceleration, steering);
    refresh();

    std::this_thread::sleep_for(std::chrono::milliseconds(loop_period_ms_));
  }
}

void KeyboardInputAdapter::restore_terminal() {
  if (curses_initialized_) {
    endwin();
    if (screen_ != nullptr) {
      delscreen(static_cast<SCREEN*>(screen_));
      screen_ = nullptr;
    }
    if (tty_file_ != nullptr) {
      std::fclose(tty_file_);
      tty_file_ = nullptr;
    }
    curses_initialized_ = false;
  }
}
