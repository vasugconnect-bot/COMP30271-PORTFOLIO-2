#!/usr/bin/env python3
"""
Waypoint mission navigator using Nav2 Simple Commander API.
"""

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import time
import math


def create_pose(x, y, theta):
    """Create a PoseStamped message."""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = rclpy.time.Time().to_msg()

    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0

    # Convert yaw (theta) to quaternion
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = math.sin(theta / 2.0)
    pose.pose.orientation.w = math.cos(theta / 2.0)

    return pose


def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Wait for Nav2 lifecycle nodes to become active
    navigator.waitUntilNav2Active()

    # -------------------------------
    # Waypoints for maze environment
    # -------------------------------
    # These are spaced out, reachable, and force turns
    waypoints = [
        create_pose(-3.5, -3.0, 0.0),      # Waypoint 1: move forward from start
        create_pose(-1.0,  1.5, 1.57),     # Waypoint 2: left corridor turn
        create_pose( 1.8,  2.5, 3.14),     # Waypoint 3: far corner
        create_pose( 2.5, -1.5, -1.57),    # Waypoint 4: long diagonal return
        create_pose( 0.0,  0.0, 0.0),      # Waypoint 5: return near centre
    ]

    print(f"Starting waypoint mission with {len(waypoints)} waypoints")

    mission_start = time.time()

    for i, waypoint in enumerate(waypoints, start=1):
        print(f"\nNavigating to waypoint {i}/{len(waypoints)}")
        print(f"Target: x={waypoint.pose.position.x:.2f}, "
              f"y={waypoint.pose.position.y:.2f}")

        waypoint_start = time.time()
        navigator.goToPose(waypoint)

        # Monitor progress
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback and feedback.distance_remaining is not None:
                print(f"  Distance remaining: {feedback.distance_remaining:.2f} m")
            time.sleep(0.5)

        waypoint_time = time.time() - waypoint_start
        result = navigator.getResult()

        if result == TaskResult.SUCCEEDED:
            print(f"  ✓ Waypoint {i} reached in {waypoint_time:.1f} seconds")
        elif result == TaskResult.CANCELED:
            print("  ✗ Navigation canceled — aborting mission")
            break
        elif result == TaskResult.FAILED:
            print("  ✗ Navigation failed — aborting mission")
            break

        # Small pause between waypoints
        time.sleep(1.0)

    total_time = time.time() - mission_start
    print(f"\nMission completed in {total_time:.1f} seconds")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
