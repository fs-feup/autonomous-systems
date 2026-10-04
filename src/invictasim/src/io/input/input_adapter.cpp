#include "io/input/input_adapter.hpp"

InvictaSimInputAdapter::InvictaSimInputAdapter(const std::shared_ptr<InvictaSim>& simulator)
    : simulator_(simulator) {
  terminal_running_ = true;
  terminal_thread_ = std::thread(&InvictaSimInputAdapter::terminal_loop, this);
}

InvictaSimInputAdapter::~InvictaSimInputAdapter() {
  terminal_running_ = false;
  if (terminal_thread_.joinable()) {
    terminal_thread_.join();
  }
}

void InvictaSimInputAdapter::terminal_loop() {
  int tty_fd = open("/dev/tty", O_RDONLY | O_NONBLOCK);
  if (tty_fd < 0) {
    tty_fd = STDIN_FILENO;
  }

  // Under `ros2 launch` the output is a pipe, not a terminal. Reading the current attributes then
  // fails, and writing back an uninitialised termios would leave the user's shell without echo.
  struct termios oldt {};
  if (!isatty(tty_fd) || tcgetattr(tty_fd, &oldt) != 0) {
    std::cerr << "[InvictaSim] No terminal available; hotkeys are disabled." << std::endl;
    if (tty_fd != STDIN_FILENO) {
      close(tty_fd);
    }
    return;
  }

  struct termios newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  if (tcsetattr(tty_fd, TCSANOW, &newt) != 0) {
    std::cerr << "[InvictaSim] Could not configure the terminal; hotkeys are disabled."
              << std::endl;
    if (tty_fd != STDIN_FILENO) {
      close(tty_fd);
    }
    return;
  }

  int oldf = fcntl(tty_fd, F_GETFL, 0);
  fcntl(tty_fd, F_SETFL, oldf | O_NONBLOCK);

  std::cout << "[InvictaSim] Terminal listener started." << std::endl;
  std::cout << "[InvictaSim] Hotkeys: [SPACE] Go Signal, [+] Speed Up, [-] Slow Down, [e] Toggle "
               "EBS, [r] Reset"
            << std::endl;

  while (terminal_running_) {
    char ch;
    int n = read(tty_fd, &ch, 1);
    if (n > 0) {
      if (ch == ' ') {
        simulator_->activate_go_signal();
        std::cout << "[InvictaSim] Activated Go Signal." << std::endl;
      } else if (ch == '+') {
        simulator_->add_sim_speed(0.25);
        std::cout << "[InvictaSim] Increased simulation speed to " << simulator_->get_sim_speed()
                  << "x" << std::endl;
      } else if (ch == '-') {
        simulator_->add_sim_speed(-0.25);
        std::cout << "[InvictaSim] Decreased simulation speed to " << simulator_->get_sim_speed()
                  << "x" << std::endl;
      } else if (ch == 'e') {
        simulator_->toggle_ebs();
        std::cout << "[InvictaSim] Toggled EBS. Current state: "
                  << (simulator_->is_ebs_active() ? "ACTIVATED" : "DEACTIVATED") << std::endl;
      } else if (ch == 'r') {
        simulator_->reset_sim();
        std::cout << "[InvictaSim] Reset simulation." << std::endl;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  tcsetattr(tty_fd, TCSANOW, &oldt);
  fcntl(tty_fd, F_SETFL, oldf);
  if (tty_fd != STDIN_FILENO) {
    close(tty_fd);
  }
}
