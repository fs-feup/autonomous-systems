#ifndef SRC_PLANNING_PLANNING_INCLUDE_PLANNING_TRACK_HPP_
#define SRC_PLANNING_PLANNING_INCLUDE_PLANNING_TRACK_HPP_

#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "./cone.hpp"

// TODO DESTRUCTOR

using namespace std;
/**
 * Track class. Contains the track information data and the cones position
 */

class Track {
  bool completed;
  vector<Cone*> leftCones;  /**<left side cone list*/
  vector<Cone*> rightCones; /**<right side cone list*/

 public:
  Track();
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
  void fillTrack(const string& path);

  /**
   * Add cone to list
   */
  void addCone(float xValue, float yValue, const string& color);  // adds the position of two cones to the track list

  // TODO CHANGE TO RECEIVE ID WITH PAIR SIDE DIFF INSTEAD OF COLOR

  void setCone(Cone* cone);
};

#endif  // SRC_PLANNING_PLANNING_INCLUDE_PLANNING_TRACK_HPP_
