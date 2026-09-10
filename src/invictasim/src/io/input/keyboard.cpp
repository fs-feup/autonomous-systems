#include "io/input/keyboard.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "common_lib/config_load/config_load.hpp"
#include "config/yaml_helpers.hpp"

KeyboardInputAdapter::KeyboardInputAdapter(const std::shared_ptr<InvictaSim>& simulator,
                                           const std::string& config_file)
    : InvictaSimInputAdapter(simulator),
      running_(false),
      window_(nullptr),
      renderer_(nullptr),
      label_font_(nullptr),
      note_font_(nullptr),
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
      label_font_size_(18),
      note_font_size_(12),
      label_top_offset_(18) {
  load_parameters(config_file);
}

void KeyboardInputAdapter::load_parameters(const std::string& config_file) {
  using invictasim::config::load_optional;

  const std::string full_path =
      common_lib::config_load::get_config_yaml_path("invictasim", "invictasim/input", config_file);
  const YAML::Node config = YAML::LoadFile(full_path);

  const YAML::Node controls = config["controls"];
  double loop_frequency = 1000.0 / static_cast<double>(loop_period_ms_);
  load_optional(controls, "loop_frequency", loop_frequency);
  loop_period_ms_ = static_cast<int>(1000.0 / std::max(loop_frequency, 1.0));

  load_optional(controls, "max_throttle", max_throttle_);
  load_optional(controls, "max_steering_radians", max_steering_radians_);
  load_optional(controls, "throttle_step", throttle_step_);
  load_optional(controls, "steering_step", steering_step_);

  const YAML::Node window = config["window"];
  load_optional(window, "width", window_width_);
  load_optional(window, "height", window_height_);
  load_optional(window, "bar_x", bar_x_);
  load_optional(window, "throttle_bar_y", throttle_bar_y_);
  load_optional(window, "steering_bar_y", steering_bar_y_);
  load_optional(window, "bar_height", bar_height_);
  load_optional(window, "label_font_size", label_font_size_);
  load_optional(window, "note_font_size", note_font_size_);
  load_optional(window, "label_top_offset", label_top_offset_);

  // Derived from the window geometry, so it has to be recomputed after loading.
  bar_width_ = window_width_ - 2 * bar_x_;
}

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

  if (TTF_Init() != 0) {
    std::cerr << "SDL_ttf initialization failed. Labels will be hidden: " << TTF_GetError()
              << std::endl;
  } else {
    initialize_fonts();
  }

  std::cout << "Keyboard controls active. Use W/S and A/D in the SDL window." << std::endl;

  input_loop();
  shutdown_sdl();
}

void KeyboardInputAdapter::stop() { running_ = false; }

void KeyboardInputAdapter::shutdown_sdl() {
  close_fonts();

  if (renderer_ != nullptr) {
    SDL_DestroyRenderer(renderer_);
    renderer_ = nullptr;
  }
  if (window_ != nullptr) {
    SDL_DestroyWindow(window_);
    window_ = nullptr;
  }

  if (TTF_WasInit() != 0) {
    TTF_Quit();
  }

  SDL_Quit();
}

bool KeyboardInputAdapter::initialize_fonts() {
  if (TTF_WasInit() == 0) {
    return false;
  }

  const std::vector<std::string> font_candidates = {
      "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
      "/usr/share/fonts/truetype/freefont/FreeSans.ttf",
      "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf"};

  for (const std::string& path : font_candidates) {
    if (label_font_ == nullptr) {
      label_font_ = TTF_OpenFont(path.c_str(), label_font_size_);
    }
    if (note_font_ == nullptr) {
      note_font_ = TTF_OpenFont(path.c_str(), note_font_size_);
    }
    if (label_font_ != nullptr && note_font_ != nullptr) {
      return true;
    }
    if (label_font_ != nullptr) {
      TTF_CloseFont(label_font_);
      label_font_ = nullptr;
    }
    if (note_font_ != nullptr) {
      TTF_CloseFont(note_font_);
      note_font_ = nullptr;
    }
  }

  std::cerr << "Could not load a font from known system paths. Labels will be hidden." << std::endl;
  return false;
}

void KeyboardInputAdapter::close_fonts() {
  if (label_font_ != nullptr) {
    TTF_CloseFont(label_font_);
    label_font_ = nullptr;
  }
  if (note_font_ != nullptr) {
    TTF_CloseFont(note_font_);
    note_font_ = nullptr;
  }
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

  const SDL_Color label_color{225, 225, 225, 255};
  draw_label(bar_x_, throttle_bar_y_ - label_top_offset_, "THROTTLE", label_font_, label_color);
  draw_label(bar_x_, steering_bar_y_ - label_top_offset_, "STEERING", label_font_, label_color);

  const char* focus_note = "FOCUS TO CONTROL";
  int focus_text_width = 0;
  if (note_font_ != nullptr) {
    TTF_SizeUTF8(note_font_, focus_note, &focus_text_width, nullptr);
  }
  const int focus_x = (window_width_ - focus_text_width) / 2;
  draw_label(focus_x, window_height_ - 14, focus_note, note_font_, label_color);

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

void KeyboardInputAdapter::draw_label(int x, int y, const char* text, TTF_Font* font,
                                      const SDL_Color& color) {
  if (renderer_ == nullptr || text == nullptr || font == nullptr) {
    return;
  }

  SDL_Surface* text_surface = TTF_RenderUTF8_Blended(font, text, color);
  if (text_surface == nullptr) {
    return;
  }

  SDL_Texture* text_texture = SDL_CreateTextureFromSurface(renderer_, text_surface);
  if (text_texture == nullptr) {
    SDL_FreeSurface(text_surface);
    return;
  }

  SDL_Rect destination{x, y, text_surface->w, text_surface->h};
  SDL_RenderCopy(renderer_, text_texture, nullptr, &destination);

  SDL_DestroyTexture(text_texture);
  SDL_FreeSurface(text_surface);
}