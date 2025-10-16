#include "unreal_client.hpp"
#include "common_lib/config_load/config_load.hpp"


int main(int argc, char** argv) {
    Config main_config("main_config.yaml");
    Config unreal_config("unreal_config.yaml");

    if (main_config.use_unreal) {
        // Initialize Unreal client
        UnrealClient client(unreal_config.ip, unreal_config.port);
        if (!client.connect()) {
            std::cerr << "Failed to connect to Unreal" << std::endl;
            return 1;
        }

        // Run simulation with Unreal connection
        run_sim_with_unreal(client , unreal_config);
    } else {
        // Run simulation standalone
        //run_sim_standalone();
    }

    return 0;
}


void run_sim_with_unreal(UnrealClient& client, const Config& unreal_config) {
        PhysicsState state(unreal_config.num_lidar_points, unreal_config.camera_buffer_size);
        state.x = 0.0f;
        state.y = 0.0f;
        state.z = 0.0f;
        state.qx = 0.0f;
        state.qy = 0.0f;
        state.qz = 0.0f;
        state.qw = 1.0f;
        state.vx = 100.0f; // 1 m/s
        state.vy = 0.0f;
        state.vz = 0.0f;
        state.roll_rate = 0.0f;
        state.pitch_rate = 0.0f;
        state.yaw_rate = 0.0f;


        bool running = true;
    // Simulation loop
    while (running) {
        // PhysicsState state = get_physics_state();
        // Currently we mock just a car moving forward in x direction
        state.x += 10.0f; // move forward 10 cm per iteration
        client.sendState(state);

        if (state.x > 1000.0f) { // stop after 10 meters
            running = false;
        }
        // Other simulation tasks...
    }
}