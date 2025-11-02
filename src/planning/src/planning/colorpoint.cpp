#include "planning/midpoint_generator.hpp"

// // ===================== Cone coloring ===================== 

// void ColorCone::color_first_cones(std::shared_ptr<Midpoint> first_point){
//   std::shared_ptr<Cone> cone1 = first_point->cone1;
//   std::shared_ptr<Cone> cone2 = first_point->cone2;

//   double dx = std::cos(initial_pose_.orientation);
//   double dy = std::sin(initial_pose_.orientation);

//   // Vectors from the inital position to each cone
//   double vx1 = cone1->position.x - initial_pose_.position.x;
//   double vy1 = cone1->position.y - initial_pose_.position.y;
//   double vx2 = cone2->position.x - initial_pose_.position.x;
//   double vy2 = cone2->position.y - initial_pose_.position.y;

//   double cross_product1 = dx * vy1 - dy * vx1;
//   double cross_product2 = dx * vy2 - dy * vx2;

//   if(cross_product1 > cross_product2){
//     cone1->color = Color::BLUE;
//     cone2->color = Color::YELLOW;
//   }else{
//     cone1->color = Color::YELLOW;
//     cone2->color = Color::BLUE;
//   }
//   current_cones_.push_back(cone1);
//   current_cones_.push_back(cone2);
// }

// bool PathCalculation::color_cone(std::shared_ptr<Midpoint> point){
//   if(point->cone1->color == Color::BLUE){
//     point->cone2->color = Color::YELLOW;
//     current_cones_.push_back(point->cone2);
//     return true;
//   }
//   if(point->cone1->color == Color::YELLOW){
//     point->cone2->color = Color::BLUE;
//     current_cones_.push_back(point->cone2);
//     return true;
//   }
//   if(point->cone2->color == Color::YELLOW){
//     point->cone1->color = Color::BLUE;
//     current_cones_.push_back(point->cone1);
//     return true;
//   }
//   if(point->cone2->color == Color::YELLOW){
//     point->cone1->color = Color::BLUE;
//     current_cones_.push_back(point->cone1);
//     return true;
//   }

//   RCLCPP_ERROR(rclcpp::get_logger("planning"), "In this pathpoint there was no cone was colored!");
//   return false;
// }