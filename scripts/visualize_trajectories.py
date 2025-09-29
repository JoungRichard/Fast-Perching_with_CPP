#!/usr/bin/env python3
"""
Trajectory Visualization Script for Original TrajOpt
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def load_trajectory_data(filename):
    try:
        data = np.loadtxt(filename, delimiter=',', skiprows=1)
        traj_data = {
            'time': data[:, 0], 'pos_x': data[:, 1], 'pos_y': data[:, 2], 'pos_z': data[:, 3],
            'vel_x': data[:, 4], 'vel_y': data[:, 5], 'vel_z': data[:, 6],
            'acc_x': data[:, 7], 'acc_y': data[:, 8], 'acc_z': data[:, 9]
        }
        print(f"Loaded: {len(data)} points, duration: {traj_data['time'][-1]:.2f}s")
        return traj_data
    except Exception as e:
        print(f"Error: {e}")
        return None

def create_visualization(traj_data):
    fig = plt.figure(figsize=(16, 10))
    
    # 3D plot
    ax1 = fig.add_subplot(2, 2, 1, projection='3d')
    ax1.plot(traj_data['pos_x'], traj_data['pos_y'], traj_data['pos_z'], 'b-', linewidth=2)
    ax1.scatter(traj_data['pos_x'][0], traj_data['pos_y'][0], traj_data['pos_z'][0], c='g', s=100, marker='o')
    ax1.scatter(traj_data['pos_x'][-1], traj_data['pos_y'][-1], traj_data['pos_z'][-1], c='r', s=100, marker='s')
    ax1.set_title('3D Trajectory')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    
    # Position vs time
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.plot(traj_data['time'], traj_data['pos_x'], 'b-', label='X')
    ax2.plot(traj_data['time'], traj_data['pos_y'], 'r--', label='Y') 
    ax2.plot(traj_data['time'], traj_data['pos_z'], 'g:', label='Z')
    ax2.set_title('Position vs Time')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Position (m)')
    ax2.legend()
    ax2.grid(True)
    
    # Velocity magnitude
    ax3 = fig.add_subplot(2, 2, 3)
    vel_mag = np.sqrt(traj_data['vel_x']**2 + traj_data['vel_y']**2 + traj_data['vel_z']**2)
    ax3.plot(traj_data['time'], vel_mag, 'b-', linewidth=2)
    ax3.axhline(y=10.0, color='r', linestyle='--', alpha=0.7, label='Limit')
    ax3.set_title('Velocity Magnitude')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Velocity (m/s)')
    ax3.legend()
    ax3.grid(True)
    
    # Acceleration magnitude
    ax4 = fig.add_subplot(2, 2, 4)
    acc_mag = np.sqrt(traj_data['acc_x']**2 + traj_data['acc_y']**2 + traj_data['acc_z']**2)
    ax4.plot(traj_data['time'], acc_mag, 'b-', linewidth=2)
    ax4.axhline(y=10.0, color='r', linestyle='--', alpha=0.7, label='Limit')
    ax4.set_title('Acceleration Magnitude')
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Acceleration (m/s)')
    ax4.legend()
    ax4.grid(True)
    
    plt.tight_layout()
    return fig

def main():
    print(" TrajOpt Visualization Tool")
    traj_data = load_trajectory_data('../assets/original_trajectory.csv')
    if traj_data:
        fig = create_visualization(traj_data)
        fig.savefig('../assets/trajectory_visualization.png', dpi=300, bbox_inches='tight')
        print("Visualization saved!")
        plt.show()

if __name__ == "__main__":
    main()
