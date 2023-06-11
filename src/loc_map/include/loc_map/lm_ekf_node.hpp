#ifndef SRC_LOC_MAP_INCLUDE_LOC_MAP_LM_EKF_NODE_HPP_
#define SRC_LOC_MAP_INCLUDE_LOC_MAP_LM_EKF_NODE_HPP_


#include "rclcpp/rclcpp.hpp"
#include "kalman_filter/ekf.hpp"

/**
 * @brief Class for the ros node responsible for executing the EKF
 * 
 */
class EKFNode : public rclcpp::Node {
    rclcpp::TimerBase::SharedPtr _timer; /**< timer */
    ExtendedKalmanFilter* _ekf; /**< SLAM EKF object */

    /**
     * @brief executes the prediction, validation and discovery steps of the EKF
     * 
     */
    void _timer_callback(); 
  public:

    /**
     * @brief Construct a new EKFNode object
     * 
     * @param ekf SLAM EKF object
     */
    EKFNode(ExtendedKalmanFilter* ekf);

    /**
     * @brief starts the timer
     * 
     */
    void start();
};



#endif // SRC_LOC_MAP_INCLUDE_LOC_MAP_LM_EKF_NODE_HPP_