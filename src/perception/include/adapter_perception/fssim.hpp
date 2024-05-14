#ifndef SRC_PERCEPTION_INCLUDE_ADAPTER_FSSIM_HPP_
#define SRC_PERCEPTION_INCLUDE_ADAPTER_FSSIM_HPP_

#include "adapter_perception/adapter.hpp"

class Perception;

/**
 * @brief Adapter class for interfacing with the FS Sim (Formula Student Simulator (AMZ)).
 * 
 * This class extends the Adapter class and provides functionality specific to
 * communication with the FS Sim.
 */
class FSSimAdapter : public Adapter {
public:
  /**
   * @brief Constructor for FSSimAdapter class.
   * 
   * @param perception A pointer to the Perception instance.
   */
  explicit FSSimAdapter(Perception* perception);

  /**
   * @brief Initializes the FS Sim Adapter.
   * 
   * Overrides the virtual method in the base class.
   */
  void init() override;

  /**
   * @brief Finalizes the FS Sim Adapter.
   * 
   * Overrides the virtual method in the base class.
   */
  void finish() override;
};

#endif  // SRC_PERCEPTION_INCLUDE_ADAPTER_FSSIM_HPP_
