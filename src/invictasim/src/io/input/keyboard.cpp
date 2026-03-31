#include "io/input/keyboard.hpp"

KeyboardInputAdapter::KeyboardInputAdapter(const std::shared_ptr<InvictaSim>& simulator)
    : InvictaSimInputAdapter(simulator),
      loop_period_ms_(20),
      acceleration_step_(0.015),        // Slower linear build-up
      acceleration_decay_step_(0.005),  // Natural engine braking
      steering_step_radians_(0.10),     // Fixed increment per keypress
      max_steering_radians_(0.39),      // Absolute physical limit (~28.6 degrees)
      help_row_(0),
      status_row_(4),
      running_(false),
      curses_initialized_(false),
      tty_file_(nullptr),
      screen_(nullptr) {}

KeyboardInputAdapter::~KeyboardInputAdapter() { stop(); }

void KeyboardInputAdapter::run() {
  if (running_) return;

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

  mvprintw(help_row_, 0, "CONTROLS:");
  mvprintw(help_row_ + 1, 0, "  w / s : Accelerate / Brake");
  mvprintw(help_row_ + 2, 0, "  a / d : Step Steering +/- 0.10");
  mvprintw(help_row_ + 3, 0, "  Space : Center Wheels | q : Quit");
  refresh();

  input_loop();
  restore_terminal();
}

void KeyboardInputAdapter::stop() { running_ = false; }

void KeyboardInputAdapter::input_loop() {
  double acceleration = 0.0;
  double steering = 0.0;

  auto last_accel_time = std::chrono::steady_clock::now();
  const auto hold_timeout = std::chrono::milliseconds(250);
  int active_accel_key = 0;

  while (running_) {
    int key = getch();
    auto now = std::chrono::steady_clock::now();

    if (key != ERR) {
      int lower_key = std::tolower(key);

      if (lower_key == 'q') {
        running_ = false;
      }
      // --- Acceleration Input ---
      else if (lower_key == 'w' || lower_key == 's') {
        active_accel_key = lower_key;
        last_accel_time = now;
      }
      // --- Steering Input with Hard Clamping ---
      else if (lower_key == 'a') {
        // Increment and then clamp to the maximum limit
        steering += steering_step_radians_;
        if (steering > max_steering_radians_) {
          steering = max_steering_radians_;
        }
      } else if (lower_key == 'd') {
        // Decrement and then clamp to the negative maximum limit
        steering -= steering_step_radians_;
        if (steering < -max_steering_radians_) {
          steering = -max_steering_radians_;
        }
      }
      // --- Quick Reset ---
      else if (lower_key == ' ') {
        steering = 0.0;
      }
    }

    // --- Linear Acceleration Logic (Smooth) ---
    bool accel_is_held = (now - last_accel_time) < hold_timeout;

    if (accel_is_held && active_accel_key == 'w') {
      acceleration = std::min(acceleration + acceleration_step_, 1.0);
    } else if (accel_is_held && active_accel_key == 's') {
      acceleration = std::max(acceleration - acceleration_step_, -1.0);
    } else {
      // Gentle decay back to standstill
      if (acceleration > 0.0) {
        acceleration = std::max(0.0, acceleration - acceleration_decay_step_);
      } else if (acceleration < 0.0) {
        acceleration = std::min(0.0, acceleration + acceleration_decay_step_);
      }
    }

    // --- Apply Output to Simulator ---
    common_lib::structures::Wheels throttle{acceleration, acceleration, acceleration, acceleration};
    simulator_->set_input(throttle, steering);

    // --- UI Update ---
    mvprintw(status_row_ + 1, 0, "------------------------------------------");
    mvprintw(status_row_ + 2, 0, "  CURRENT STATE:                          ");
    mvprintw(status_row_ + 3, 0, "  Accel: % .3f | Steer: % .2f rad       ", acceleration,
             steering);

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