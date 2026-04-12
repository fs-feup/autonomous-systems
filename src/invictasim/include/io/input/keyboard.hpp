#pragma once

#include <SDL.h>
#include <SDL2/SDL_ttf.h>

#include <atomic>
#include <memory>

#include "io/input/input_adapter.hpp"

/**
 * @brief SDL-backed keyboard simulator input adapter.
 *
 * Runs in its own thread and reads simultaneous key states from an SDL window instead of a
 * terminal.
 */
class KeyboardInputAdapter : public InvictaSimInputAdapter {
public:
  /**
   * @brief Construct a new KeyboardInputAdapter.
   * @param simulator Simulator instance.
   */
  explicit KeyboardInputAdapter(const std::shared_ptr<InvictaSim>& simulator);

  /**
   * @brief Destroy the KeyboardInputAdapter.
   */
  ~KeyboardInputAdapter() override;

  /**
   * @brief Start reading keyboard input.
   */
  void run() override;

  /**
   * @brief Stop reading keyboard input.
   */
  void stop() override;

private:
  /**
   * @brief Main SDL polling loop.
   */
  void input_loop();

  /**
   * @brief Close SDL resources before exit.
   */
  void shutdown_sdl();
  bool initialize_fonts();
  void close_fonts();

  void render_bars(double throttle, double steering);
  void draw_label(int x, int y, const char* text, TTF_Font* font, const SDL_Color& color);

  static double approach(double current, double target, double max_delta);

  std::atomic<bool> running_;
  SDL_Window* window_;
  SDL_Renderer* renderer_;
  TTF_Font* label_font_;
  TTF_Font* note_font_;

  const int loop_period_ms_;
  const double throttle_step_;
  const double steering_step_;
  const double max_throttle_;
  const double max_steering_radians_;

  const int window_width_;
  const int window_height_;
  const int bar_x_;
  const int bar_width_;
  const int throttle_bar_y_;
  const int steering_bar_y_;
  const int bar_height_;
  const int label_font_size_;
  const int note_font_size_;
  const int label_top_offset_;
};
