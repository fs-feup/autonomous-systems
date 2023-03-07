#ifndef SRC_PLANNING_PLANNING_INCLUDE_PLANNING_TRACK_HPP_
#define SRC_PLANNING_PLANNING_INCLUDE_PLANNING_TRACK_HPP_

#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "position.hpp"

using namespace std;
/**
 * Track class. Contains the track information data and the cones position
 */

class Track {
  bool completed;
  vector<Position*> leftCones;  /**<left side cone list*/
  vector<Position*> rightCones; /**<right side cone list*/

 public:
  Track();

  /**
   * Access right cone in the list in a specified index
   */
  Position* getRightConeAt(int index);
  /**
   * Access left cone in the list in a specified index
   */
  Position* getLeftConeAt(int index);

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
  void addConePair(
      Position* cone,
      const string& color);  // adds the position of two cones to the track list

  void receiveConeData();  // TODO(andre): ros subscriber input (Position left,
                           // Position right, color);

  void sendTrack();  // TODO(andre): send track data to path planner
};

#endif // SRC_PLANNING_PLANNING_INCLUDE_PLANNING_TRACK_HPP_
