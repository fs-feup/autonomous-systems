#pragma once

/**
 * @brief Abstract base class that defines the interface for single variable filters
 * Derived classes must implement the filter and reset methods.
 *
 */
class Filter {
public:
  /**
   * @brief Virtual destructor for safe polymorphic use
   */
  virtual ~Filter() = default;

  /**
   * @brief Apply the filter to a new input value
   * @param input New input value
   * @return Filtered value
   */
  virtual double filter(double input) = 0;

  /**
   * @brief Reset the filter to a specific value
   * @param value Value to reset the filter to (default is 0.0)
   */
  virtual void reset(double value = 0.0) = 0;
};