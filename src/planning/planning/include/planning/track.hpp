#ifndef SRC_PLANNING_PLANNING_INCLUDE_PLANNING_TRACK_HPP_
#define SRC_PLANNING_PLANNING_INCLUDE_PLANNING_TRACK_HPP_

#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "../utils/cone.hpp"

/**
 * Track class. Contains the track information data and the cones position
 */

class Track {
  bool completed;
  std::vector<Cone*> leftCones;  /**<left side cone list*/
  std::vector<Cone*> rightCones; /**<right side cone list*/

  int rightCount = 0;
  int leftCount = 0;

 public:
  Track();
  ~Track();

  /**
   * Access right cone in the list in a specified index
   */
  Cone* getRightConeAt(int index);
  /**
   * Access left cone in the list in a specified index
   */
  Cone* getLeftConeAt(int index);

  /**
   * Get right cones number
   */
  int getRightConesSize();
  /**
   * Get left cones number
   */
  int getLeftConesSize();

  /**
   * Read file to populate track
   */
  void fillTrack(const std::string& path);

  /**
   * Add cone to list
   */
  void addCone(float xValue, float yValue, const std::string& color);
  // adds the position of two cones to the track list

  // TODO(andre): CHANGE TO RECEIVE ID WITH PAIR SIDE DIFF INSTEAD OF COLOR

  void setCone(Cone* cone);

  Cone* findCone(int id);

  Cone* findCone(float x, float y);

  void reset();
};

#endif  // SRC_PLANNING_PLANNING_INCLUDE_PLANNING_TRACK_HPP_
