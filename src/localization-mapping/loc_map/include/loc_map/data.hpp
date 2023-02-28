#include <ctime>
#include <map>

struct Cone {
  std::string color;
};

struct Position {
  int x;
  int y;
} vehicle_position;

struct {
  std::map<Position, Cone> map;
} track_map;