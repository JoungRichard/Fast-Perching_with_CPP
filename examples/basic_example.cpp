#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>

#include "traj_opt.h"

// Helper function to compare trajectories
bool compareTrajectories(const Trajectory& traj1, const Trajectory& traj2, double tolerance = 1e-3) {
    if (traj1.getPieceNum() != traj2.getPieceNum()) {
        std::cout << "Different number of trajectory pieces: " << traj1.getPieceNum() << " vs " << traj2.getPieceNum() << std::endl;
        return false;
    }

    bool trajectories_match = true;
    double max_position_diff = 0.0;
    double max_velocity_diff = 0.0;

    // Compare trajectories at multiple sample points
    for (int i = 0; i < traj1.getPieceNum(); ++i) {
        double piece_duration = traj1[i].getDuration();
        int num_samples = 10;

        for (int j = 0; j <= num_samples; ++j) {
            double t = j * piece_duration / num_samples;

            Eigen::Vector3d pos1 = traj1[i].getPos(t);
            Eigen::Vector3d vel1 = traj1[i].getVel(t);
            Eigen::Vector3d pos2 = traj2[i].getPos(t);
            Eigen::Vector3d vel2 = traj2[i].getVel(t);

            double pos_diff = (pos1 - pos2).norm();
            double vel_diff = (vel1 - vel2).norm();

            max_position_diff = std::max(max_position_diff, pos_diff);
            max_velocity_diff = std::max(max_velocity_diff, vel_diff);

            if (pos_diff > tolerance || vel_diff > tolerance) {
                trajectories_match = false;
            }
        }
    }

    std::cout << "Max position difference: " << std::fixed << std::setprecision(6) << max_position_diff << std::endl;
    std::cout << "Max velocity difference: " << std::fixed << std::setprecision(6) << max_velocity_diff << std::endl;

    return trajectories_match;
}

// Helper function to print trajectory statistics
void printTrajectoryStats(const std::string& name, const Trajectory& traj) {
    if (traj.getPieceNum() == 0) {
        std::cout << name << ": Empty trajectory" << std::endl;
        return;
    }

    std::cout << name << " statistics:" << std::endl;
    std::cout << "  Pieces: " << traj.getPieceNum() << std::endl;
    std::cout << "  Total duration: " << std::fixed << std::setprecision(3) << traj.getTotalDuration() << "s" << std::endl;

    // Sample trajectory at start, middle, and end
    Eigen::Vector3d start_pos = traj.getPos(0.0);
    Eigen::Vector3d end_pos = traj.getPos(traj.getTotalDuration());
    Eigen::Vector3d start_vel = traj.getVel(0.0);
    Eigen::Vector3d end_vel = traj.getVel(traj.getTotalDuration());

    std::cout << "  Start position: [" << std::fixed << std::setprecision(3)
              << start_pos.x() << ", " << start_pos.y() << ", " << start_pos.z() << "]" << std::endl;
    std::cout << "  End position: [" << std::fixed << std::setprecision(3)
              << end_pos.x() << ", " << end_pos.y() << ", " << end_pos.z() << "]" << std::endl;
    std::cout << "  Start velocity: [" << std::fixed << std::setprecision(3)
              << start_vel.x() << ", " << start_vel.y() << ", " << start_vel.z() << "]" << std::endl;
    std::cout << "  End velocity: [" << std::fixed << std::setprecision(3)
              << end_vel.x() << ", " << end_vel.y() << ", " << end_vel.z() << "]" << std::endl;
}

int main() {
    std::cout << "=================================================================\n";
    std::cout << "        Trajectory Optimization Test\n";
    std::cout << "=================================================================\n";

    // Common problem setup
    Eigen::MatrixXd initial_state(3, 4);
    initial_state.setZero();
    initial_state.col(0) = Eigen::Vector3d(0.0, 0.0, 3.0);   // Initial position
    initial_state.col(1) = Eigen::Vector3d(2.0, 0.0, -0.5);  // Initial velocity
    initial_state.col(2) = Eigen::Vector3d(0.0, 0.0, 0.0);   // Initial acceleration
    initial_state.col(3) = Eigen::Vector3d(0.0, 0.0, 0.0);   // Initial jerk

    Eigen::Vector3d target_position(8.0, 2.0, 1.0);
    Eigen::Vector3d target_velocity(1.5, 0.5, 0.0);
    Eigen::Quaterniond landing_quaternion(0.9659, 0.0, 0.2588, 0.0);  // Identity quaternion

    int num_pieces = 6;

    std::cout << "\nProblem setup:\n";
    std::cout << "  Initial position: [" << initial_state.col(0).transpose() << "]\n";
    std::cout << "  Initial velocity: [" << initial_state.col(1).transpose() << "]\n";
    std::cout << "  Target position:  [" << target_position.transpose() << "]\n";
    std::cout << "  Target velocity:  [" << target_velocity.transpose() << "]\n";
    std::cout << "  Number of pieces: " << num_pieces << "\n";

    // Test result storage
    Trajectory traj_opt_result;
    bool traj_opt_success = false;
    double traj_opt_time = 0.0;

    // Test: Original TrajOpt class
    {
        std::cout << "\n=== Testing Original TrajOpt Class ===\n";
        traj_opt::TrajOpt optimizer;

        // Configure parameters
        optimizer.setDynamicLimits(10.0, 10.0, 20.0, 2.0, 3.0, 2.0);
        optimizer.setRobotParameters(1.0, 0.3, 0.1, 0.5);
        optimizer.setOptimizationWeights(1.0, -1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0);
        optimizer.setIntegrationSteps(20);
        optimizer.setDebugMode(false);

        std::cout << "Configuration complete. Starting optimization...\n";

        auto start_time = std::chrono::high_resolution_clock::now();

        traj_opt_success = optimizer.generate_traj(
            initial_state,
            target_position,
            target_velocity,
            landing_quaternion,
            num_pieces,
            traj_opt_result);

        auto end_time = std::chrono::high_resolution_clock::now();
        traj_opt_time = std::chrono::duration<double, std::milli>(end_time - start_time).count();

        if (traj_opt_success) {
            std::cout << "✓ TrajOpt optimization successful!\n";
            std::cout << "  Optimization time: " << std::fixed << std::setprecision(2) << traj_opt_time << " ms\n";
            printTrajectoryStats("TrajOpt", traj_opt_result);
        } else {
            std::cout << "✗ TrajOpt optimization failed!\n";
        }
    }

    // Results and feasibility check
    if (traj_opt_success) {
        std::cout << "\n=== Results ===\n";
        std::cout << "TrajOpt optimization completed successfully.\n";

        std::cout << "\nRunning feasibility check...\n";
        traj_opt::TrajOpt feasibility_checker;
        feasibility_checker.setDynamicLimits(10.0, 10.0, 20.0, 2.0, 3.0, 2.0);
        bool is_feasible = feasibility_checker.feasibleCheck(traj_opt_result);
        std::cout << "  Result feasible: " << (is_feasible ? "✓ Yes" : "✗ No") << std::endl;

        // Save trajectory for visualization
        std::cout << "\nSaving trajectory for visualization...\n";

// Create assets directory if it doesn't exist
#ifdef _WIN32
        system("if not exist \"..\\assets\" mkdir \"..\\assets\"");
#else
        system("mkdir -p ../assets");
#endif

        // Save trajectory to CSV file
        std::ofstream csv_file("../assets/original_trajectory.csv");
        csv_file << "time,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,acc_x,acc_y,acc_z\n";

        double dt = 0.01;  // 10ms sampling interval
        double total_duration = traj_opt_result.getTotalDuration();

        for (double t = 0.0; t <= total_duration; t += dt) {
            Eigen::Vector3d pos = traj_opt_result.getPos(t);
            Eigen::Vector3d vel = traj_opt_result.getVel(t);
            Eigen::Vector3d acc = traj_opt_result.getAcc(t);

            csv_file << std::fixed << std::setprecision(6)
                     << t << ","
                     << pos.x() << "," << pos.y() << "," << pos.z() << ","
                     << vel.x() << "," << vel.y() << "," << vel.z() << ","
                     << acc.x() << "," << acc.y() << "," << acc.z() << "\n";
        }

        csv_file.close();
        std::cout << "✓ Trajectory saved to ../assets/original_trajectory.csv\n";
        std::cout << "  Total data points: " << (int)(total_duration / dt) + 1 << "\n";
        std::cout << "  Sampling interval: " << dt << "s\n";
    } else {
        std::cout << "\n=== Results ===\n";
        std::cout << "TrajOpt optimization failed. Check problem setup and dependencies.\n";
    }

    return 0;
}