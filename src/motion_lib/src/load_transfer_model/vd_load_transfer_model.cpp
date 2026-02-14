#include "motion_lib/load_transfer_model/vd_load_transfer_model.hpp"

Eigen::Vector4d VDLoadTransferModel::compute_loads(
    const Eigen::VectorXd& dynamic_state) const {
        Eigen::Vector4d loads;
        float mass_distribution = car_parameters_->cg_2_rear_axis / car_parameters_->wheelbase; 
        float downforce = dynamic_state[2];
        float front_static_load = (car_parameters_->total_mass * physical_constants_->gravity) * mass_distribution ;
        float rear_static_load = (car_parameters_->total_mass * physical_constants_->gravity) * (1 - mass_distribution);
        float aero_load_front = abs(downforce) * car_parameters_->aero_parameters->aero_balance_front;
        float aero_load_rear = abs(downforce) * (1 - car_parameters_->aero_parameters->aero_balance_front);
        float longitudinal_transfer = calculate_longitudinal_transfer(dynamic_state[0]);
        float lateral_transfer_front = calculate_front_lateral_transfer(mass_distribution, dynamic_state[1]);
        float lateral_transfer_rear = calculate_rear_lateral_transfer(mass_distribution, dynamic_state[1]);
        /*
        Important notes: This calculation assumes that static load is distributed evenly between left and right tires same for aero.
        Front static load returns the value of both tires
        Aero load front returns the value of both tires
        Longitudinal transfer returns the total value of load transfer that is then distributed between front and rear
        Lateral transfer returns the total value of load transfer that is then distributed between each axis
        */
        loads(0) = (front_static_load/2) + (aero_load_front/2) - (longitudinal_transfer/2) - lateral_transfer_front;  // Front Left
        loads(1) = (front_static_load/2) + (aero_load_front/2) - (longitudinal_transfer/2) + lateral_transfer_front;  // Front Right
        loads(2) = (rear_static_load/2) + (aero_load_rear/2) + (longitudinal_transfer/2) - lateral_transfer_rear;  // Rear Left
        loads(3) = (rear_static_load/2) + (aero_load_rear/2) + (longitudinal_transfer/2) + lateral_transfer_rear;  // Rear Right
        return loads;
    }

    // Assuming mass distribution is a value between 0 and 1 representing the percentage of total mass on the front axle and front stiffness distribution is a value between 0 and 1.
float VDLoadTransferModel::calculate_front_lateral_transfer(float massDistribution , float lateral_acceleration) const {
    float unsprung_load_transfer = ((car_parameters_->unsprung_mass * massDistribution) * lateral_acceleration * car_parameters_->unsprung_cg_z) / car_parameters_->track_width;
    float geometric_load_transfer = ((car_parameters_->sprung_mass * massDistribution) * lateral_acceleration * car_parameters_->load_transfer_parameters->front_roll_center_z) / car_parameters_->track_width;
    float elastic_load_transfer = (car_parameters_->sprung_mass * lateral_acceleration *  car_parameters_->load_transfer_parameters->roll_axis_z * car_parameters_->load_transfer_parameters->front_stiffness_distribution) / car_parameters_->track_width;
    return unsprung_load_transfer + geometric_load_transfer + elastic_load_transfer;
}

float VDLoadTransferModel::calculate_rear_lateral_transfer(float massDistribution , float lateral_acceleration) const {
    float unsprung_load_transfer = ((car_parameters_->unsprung_mass * (1 - massDistribution)) * lateral_acceleration * car_parameters_->unsprung_cg_z) / car_parameters_->track_width;
    float geometric_load_transfer = ((car_parameters_->sprung_mass * (1 - massDistribution)) * lateral_acceleration * car_parameters_->load_transfer_parameters->rear_roll_center_z) / car_parameters_->track_width;
    float elastic_load_transfer = (car_parameters_->sprung_mass * lateral_acceleration *  car_parameters_->load_transfer_parameters->roll_axis_z * (1-car_parameters_->load_transfer_parameters->front_stiffness_distribution)) / car_parameters_->track_width;
    return unsprung_load_transfer + geometric_load_transfer + elastic_load_transfer;
}


float VDLoadTransferModel::calculate_longitudinal_transfer(float longitudinal_acceleration) const{
    float unsprung_load_transfer = ((car_parameters_->unsprung_mass) * longitudinal_acceleration * car_parameters_->unsprung_cg_z ) / car_parameters_->wheelbase;
    float elastic_load_transfer = ((car_parameters_->sprung_mass) * longitudinal_acceleration * (car_parameters_->sprung_cg_z - car_parameters_->load_transfer_parameters->pitch_center_z)) / car_parameters_->wheelbase;
    float geometric_load_transfer = (car_parameters_->sprung_mass * longitudinal_acceleration * car_parameters_->load_transfer_parameters->pitch_center_z ) / car_parameters_->wheelbase;
    return unsprung_load_transfer + geometric_load_transfer + elastic_load_transfer;
}