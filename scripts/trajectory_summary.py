#!/usr/bin/env python3
"""
Trajectory Analysis Script for Original TrajOpt

This script loads and analyzes trajectory data from CSV files generated 
by the original TrajOpt implementation.
"""

import numpy as np
import os

def analyze_trajectory():
    """Load trajectory and generate analysis summary."""
    
    print("🚁 Original TrajOpt Trajectory Analysis")
    print("=" * 50)
    
    try:
        # Load original trajectory
        data = np.loadtxt('../assets/original_trajectory.csv', delimiter=',', skiprows=1)
        traj = {
            'time': data[:, 0],
            'pos': data[:, 1:4],
            'vel': data[:, 4:7], 
            'acc': data[:, 7:10]
        }
        
        print(f"✅ Successfully loaded trajectory with {len(traj['time'])} points")
        
    except Exception as e:
        print(f"❌ Error loading trajectory: {e}")
        return

    # Basic trajectory info
    duration = traj['time'][-1]
    points = len(traj['time'])
    
    print(f"\n📊 BASIC TRAJECTORY INFO:")
    print(f"{'Metric':<25} {'Value':<15}")
    print("-" * 40)
    print(f"{'Duration (s)':<25} {duration:<15.3f}")
    print(f"{'Data Points':<25} {points:<15d}")
    
    # Final position
    final_pos = traj['pos'][-1]
    print(f"\n🎯 FINAL POSITION:")
    print(f"  Position: [{final_pos[0]:.3f}, {final_pos[1]:.3f}, {final_pos[2]:.3f}]")
    
    # Dynamic characteristics
    vel_mag = np.linalg.norm(traj['vel'], axis=1)
    acc_mag = np.linalg.norm(traj['acc'], axis=1)
    vel_max = vel_mag.max()
    acc_max = acc_mag.max()
    
    print(f"\n🚀 DYNAMIC CHARACTERISTICS:")
    print(f"  Max Velocity:     {vel_max:.3f}m/s")
    print(f"  Max Acceleration: {acc_max:.3f}m/s²")
    
    # Check constraint violations (assuming limits from C++ code)
    vel_limit, acc_limit = 10.0, 10.0
    
    print(f"\n⚖️  CONSTRAINT COMPLIANCE:")
    print(f"  Velocity Limit ({vel_limit}m/s):     {'✅ OK' if vel_max <= vel_limit else '❌ VIOLATED'}")
    print(f"  Acceleration Limit ({acc_limit}m/s²): {'✅ OK' if acc_max <= acc_limit else '❌ VIOLATED'}")
    
    # Trajectory smoothness analysis
    print(f"\n📏 TRAJECTORY SMOOTHNESS ANALYSIS:")
    
    # Velocity changes (jerk approximation)
    dt = np.diff(traj['time'])
    jerk_approx = np.diff(traj['acc'], axis=0) / dt[:, np.newaxis]
    jerk_mag = np.linalg.norm(jerk_approx, axis=1)
    jerk_max = jerk_mag.max()
    jerk_mean = jerk_mag.mean()
    
    print(f"  Max Jerk (approx):  {jerk_max:.3f}m/s³")
    print(f"  Mean Jerk (approx): {jerk_mean:.3f}m/s³")
    
    # Position changes
    pos_changes = np.diff(traj['pos'], axis=0)
    distance_traveled = np.sum(np.linalg.norm(pos_changes, axis=1))
    straight_line_distance = np.linalg.norm(final_pos - traj['pos'][0])
    
    print(f"  Total Distance:     {distance_traveled:.3f}m")
    print(f"  Straight Distance:  {straight_line_distance:.3f}m")
    print(f"  Path Efficiency:    {straight_line_distance/distance_traveled:.3f}")
    
    print(f"\n📋 SUMMARY:")
    
    feasible = vel_max <= vel_limit and acc_max <= acc_limit
    smooth = jerk_max < 50.0  # Reasonable jerk limit
    efficient = straight_line_distance/distance_traveled > 0.7
    
    if feasible and smooth and efficient:
        print(f"  🎉 EXCELLENT: Trajectory is feasible, smooth, and efficient.")
    elif feasible and smooth:
        print(f"  ✅ GOOD: Trajectory is feasible and smooth.")
    elif feasible:
        print(f"  ⚠️  ACCEPTABLE: Trajectory is feasible but may have smoothness issues.")
    else:
        print(f"  ❌ POOR: Trajectory violates dynamic constraints.")
        
    print(f"\nTrajectory analysis complete. Check visualization for detailed plots.")

if __name__ == "__main__":
    analyze_trajectory()