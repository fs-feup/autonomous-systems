#ifndef SRC_LOC_MAP_INCLUDE_UTILS_FORMULAS_HPP_
#define SRC_LOC_MAP_INCLUDE_UTILS_FORMULAS_HPP_


/**
 * @brief Function to do sin in degrees
 * 
 * @param angle in degrees
 * @return double 
 */
double sin_in_degrees(double angle);

/**
 * @brief Function to do cos in degrees
 * 
 * @param angle in degrees
 * @return double 
 */
double cos_in_degrees(double angle);

/**
 * @brief Function to keep angle between 0 and 360 degrees
 * 
 * @param angle 
 * @return double 
 */
double normalize_angle(double angle);


#endif  // SRC_LOC_MAP_INCLUDE_UTILS_FORMULAS_HPP_