#include "io/input/keyboard.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <thread>

KeyboardInputAdapter::KeyboardInputAdapter(const std::shared_ptr<InvictaSim>& simulator)
    : InvictaSimInputAdapter(simulator),
      running_(false),
      window_(nullptr),
      renderer_(nullptr),
      loop_period_ms_(static_cast<int>(1000.0 / 60.0)),
      throttle_step_(0.08),
      steering_step_(0.04),
      max_throttle_(1.0),
      max_steering_radians_(0.39),
      window_width_(500),
      window_height_(210),
      bar_x_(24),
      bar_width_(window_width_ - 2 * bar_x_),
      throttle_bar_y_(42),
      steering_bar_y_(108),
      bar_height_(30),
      label_scale_(2),
      label_top_offset_(18) {}

KeyboardInputAdapter::~KeyboardInputAdapter() { stop(); }

void KeyboardInputAdapter::run() {
  if (running_) {
    return;
  }

  running_ = true;

  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS) != 0) {
    std::cerr << "Failed to initialize SDL keyboard input: " << SDL_GetError() << std::endl;
    running_ = false;
    return;
  }

  window_ =
      SDL_CreateWindow("Invictasim Keyboard Controls", SDL_WINDOWPOS_CENTERED,
                       SDL_WINDOWPOS_CENTERED, window_width_, window_height_, SDL_WINDOW_SHOWN);
  if (window_ == nullptr) {
    std::cerr << "Failed to create keyboard window: " << SDL_GetError() << std::endl;
    SDL_Quit();
    running_ = false;
    return;
  }

  renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
  if (renderer_ == nullptr) {
    std::cerr << "Failed to create renderer: " << SDL_GetError() << std::endl;
    SDL_DestroyWindow(window_);
    window_ = nullptr;
    SDL_Quit();
    running_ = false;
    return;
  }

  std::cout << "Keyboard controls active. Use W/S and A/D in the SDL window." << std::endl;

  input_loop();
  shutdown_sdl();
}

void KeyboardInputAdapter::stop() { running_ = false; }

void KeyboardInputAdapter::shutdown_sdl() {
  if (renderer_ != nullptr) {
    SDL_DestroyRenderer(renderer_);
    renderer_ = nullptr;
  }
  if (window_ != nullptr) {
    SDL_DestroyWindow(window_);
    window_ = nullptr;
  }
  SDL_Quit();
}

double KeyboardInputAdapter::approach(double current, double target, double max_delta) {
  if (current < target) {
    return std::min(current + max_delta, target);
  }
  if (current > target) {
    return std::max(current - max_delta, target);
  }
  return current;
}

void KeyboardInputAdapter::input_loop() {
  double throttle = 0.0;
  double steering = 0.0;

  while (running_) {
    SDL_Event event;
    while (SDL_PollEvent(&event) != 0) {
      if (event.type == SDL_QUIT) {
        running_ = false;
      } else if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_q) {
        running_ = false;
      }
    }

    SDL_PumpEvents();
    const Uint8* keys = SDL_GetKeyboardState(nullptr);

    const bool accelerate = keys[SDL_SCANCODE_W];
    const bool brake = keys[SDL_SCANCODE_S];
    const bool steer_left = keys[SDL_SCANCODE_A];
    const bool steer_right = keys[SDL_SCANCODE_D];

    double target_throttle = 0.0;
    if (accelerate && !brake) {
      target_throttle = max_throttle_;
    } else if (brake && !accelerate) {
      target_throttle = -max_throttle_;
    }

    double target_steering = 0.0;
    if (steer_left && !steer_right) {
      target_steering = max_steering_radians_;
    } else if (steer_right && !steer_left) {
      target_steering = -max_steering_radians_;
    }

    throttle = approach(throttle, target_throttle, throttle_step_);
    steering = approach(steering, target_steering, steering_step_);

    common_lib::structures::Wheels throttle_command{throttle, throttle, throttle, throttle};
    simulator_->set_input(throttle_command, steering);

    if (renderer_ != nullptr) {
      render_bars(throttle, steering);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(loop_period_ms_));
  }
}

void KeyboardInputAdapter::render_bars(double throttle, double steering) {
  if (renderer_ == nullptr) {
    return;
  }

  SDL_SetRenderDrawColor(renderer_, 30, 32, 36, 255);
  SDL_RenderClear(renderer_);

  const int center_x = bar_x_ + (bar_width_ / 2);
  const double throttle_norm =
      max_throttle_ > 0.0 ? std::clamp(throttle / max_throttle_, -1.0, 1.0) : 0.0;
  const double steering_norm =
      max_steering_radians_ > 0.0 ? std::clamp(steering / max_steering_radians_, -1.0, 1.0) : 0.0;

  SDL_Rect throttle_track{bar_x_, throttle_bar_y_, bar_width_, bar_height_};
  SDL_Rect steering_track{bar_x_, steering_bar_y_, bar_width_, bar_height_};
  SDL_SetRenderDrawColor(renderer_, 55, 58, 63, 255);
  SDL_RenderFillRect(renderer_, &throttle_track);
  SDL_RenderFillRect(renderer_, &steering_track);

  SDL_SetRenderDrawColor(renderer_, 175, 175, 175, 255);
  SDL_RenderDrawRect(renderer_, &throttle_track);
  SDL_RenderDrawRect(renderer_, &steering_track);
  SDL_RenderDrawLine(renderer_, center_x, throttle_bar_y_, center_x, throttle_bar_y_ + bar_height_);
  SDL_RenderDrawLine(renderer_, center_x, steering_bar_y_, center_x, steering_bar_y_ + bar_height_);

  SDL_SetRenderDrawColor(renderer_, 225, 225, 225, 255);
  draw_label(bar_x_, throttle_bar_y_ - label_top_offset_, "THROTTLE", label_scale_);
  draw_label(bar_x_, steering_bar_y_ - label_top_offset_, "STEERING", label_scale_);

  const char* focus_note = "FOCUS TO CONTROL";
  const int focus_scale = 1;
  const int focus_text_width = static_cast<int>((std::strlen(focus_note) * 6 - 1) * focus_scale);
  const int focus_x = (window_width_ - focus_text_width) / 2;
  draw_label(focus_x, window_height_ - 14, focus_note, focus_scale);

  const int throttle_delta =
      static_cast<int>(std::lround(throttle_norm * static_cast<double>(bar_width_ / 2)));
  if (throttle_delta != 0) {
    SDL_Rect throttle_fill{std::min(center_x, center_x + throttle_delta), throttle_bar_y_,
                           std::abs(throttle_delta), bar_height_};
    if (throttle_delta > 0) {
      SDL_SetRenderDrawColor(renderer_, 50, 185, 110, 255);
    } else {
      SDL_SetRenderDrawColor(renderer_, 220, 85, 85, 255);
    }
    SDL_RenderFillRect(renderer_, &throttle_fill);
  }

  const int steering_delta =
      static_cast<int>(std::lround(-steering_norm * static_cast<double>(bar_width_ / 2)));
  if (steering_delta != 0) {
    SDL_Rect steering_fill{std::min(center_x, center_x + steering_delta), steering_bar_y_,
                           std::abs(steering_delta), bar_height_};
    SDL_SetRenderDrawColor(renderer_, 70, 145, 235, 255);
    SDL_RenderFillRect(renderer_, &steering_fill);
  }

  SDL_RenderPresent(renderer_);
}

void KeyboardInputAdapter::draw_label(int x, int y, const char* text, int scale) {
  if (renderer_ == nullptr || text == nullptr || scale <= 0) {
    return;
  }

  int cursor_x = x;
  for (const char* p = text; *p != '\0'; ++p) {
    draw_glyph(cursor_x, y, *p, scale);
    cursor_x += 6 * scale;
  }
}

void KeyboardInputAdapter::draw_glyph(int x, int y, char c, int scale) {
  using Glyph = std::array<std::uint8_t, 7>;
  static const Glyph kSpace = {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000};
  static const Glyph kC = {0b01110, 0b10001, 0b10000, 0b10000, 0b10000, 0b10001, 0b01110};
  static const Glyph kE = {0b11111, 0b10000, 0b11110, 0b10000, 0b10000, 0b10000, 0b11111};
  static const Glyph kF = {0b11111, 0b10000, 0b11110, 0b10000, 0b10000, 0b10000, 0b10000};
  static const Glyph kG = {0b01110, 0b10001, 0b10000, 0b10111, 0b10001, 0b10001, 0b01110};
  static const Glyph kH = {0b10001, 0b10001, 0b11111, 0b10001, 0b10001, 0b10001, 0b10001};
  static const Glyph kI = {0b11111, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b11111};
  static const Glyph kL = {0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b11111};
  static const Glyph kN = {0b10001, 0b11001, 0b10101, 0b10011, 0b10001, 0b10001, 0b10001};
  static const Glyph kO = {0b01110, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01110};
  static const Glyph kR = {0b11110, 0b10001, 0b10001, 0b11110, 0b10100, 0b10010, 0b10001};
  static const Glyph kS = {0b01110, 0b10001, 0b10000, 0b01110, 0b00001, 0b10001, 0b01110};
  static const Glyph kT = {0b11111, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100};
  static const Glyph kU = {0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01110};

  const Glyph* glyph = &kSpace;
  switch (c) {
    case 'C':
      glyph = &kC;
      break;
    case 'E':
      glyph = &kE;
      break;
    case 'F':
      glyph = &kF;
      break;
    case 'G':
      glyph = &kG;
      break;
    case 'H':
      glyph = &kH;
      break;
    case 'I':
      glyph = &kI;
      break;
    case 'L':
      glyph = &kL;
      break;
    case 'N':
      glyph = &kN;
      break;
    case 'O':
      glyph = &kO;
      break;
    case 'R':
      glyph = &kR;
      break;
    case 'S':
      glyph = &kS;
      break;
    case 'T':
      glyph = &kT;
      break;
    case 'U':
      glyph = &kU;
      break;
    default:
      break;
  }

  for (int row = 0; row < 7; ++row) {
    for (int col = 0; col < 5; ++col) {
      if (((*glyph)[row] & (1 << (4 - col))) == 0) {
        continue;
      }
      SDL_Rect pixel{x + col * scale, y + row * scale, scale, scale};
      SDL_RenderFillRect(renderer_, &pixel);
    }
  }
}