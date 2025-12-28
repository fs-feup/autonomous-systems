#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace test_utils {

inline sensor_msgs::msg::PointCloud2::SharedPtr make_lidar_pointcloud2(
    const std::vector<std::array<float, 5>>& pts) {
  auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  msg->header.frame_id = "test";
  msg->height = 1;
  msg->width = pts.size();
  msg->is_dense = true;
  msg->is_bigendian = false;
  msg->point_step = 26;
  msg->row_step = msg->point_step * msg->width;
  msg->fields.resize(6);
  msg->fields[0].name = "x";
  msg->fields[0].offset = 0;
  msg->fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg->fields[0].count = 1;
  msg->fields[1].name = "y";
  msg->fields[1].offset = 4;
  msg->fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg->fields[1].count = 1;
  msg->fields[2].name = "z";
  msg->fields[2].offset = 8;
  msg->fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg->fields[2].count = 1;
  msg->fields[3].name = "intensity";
  msg->fields[3].offset = 12;
  msg->fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg->fields[3].count = 1;
  msg->fields[4].name = "ring";
  msg->fields[4].offset = 16;
  msg->fields[4].datatype = sensor_msgs::msg::PointField::UINT16;
  msg->fields[4].count = 1;
  msg->fields[5].name = "timestamp";
  msg->fields[5].offset = 18;
  msg->fields[5].datatype = sensor_msgs::msg::PointField::FLOAT64;
  msg->fields[5].count = 1;
  msg->data.resize(msg->row_step);
  for (size_t i = 0; i < pts.size(); ++i) {
    memcpy(&msg->data[i * 26 + 0], &pts[i][0], 4);
    memcpy(&msg->data[i * 26 + 4], &pts[i][1], 4);
    memcpy(&msg->data[i * 26 + 8], &pts[i][2], 4);
    float intensity = 0.0f;
    memcpy(&msg->data[i * 26 + 12], &intensity, 4);
    uint16_t ring = static_cast<uint16_t>(pts[i][4]);
    memcpy(&msg->data[i * 26 + 16], &ring, 2);
    uint64_t ts = 0;
    memcpy(&msg->data[i * 26 + 18], &ts, 8);
  }
  return msg;
}

}  // namespace test_utils
