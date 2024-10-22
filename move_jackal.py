#!/usr/bin/env python

# Team 01
# Spanou Maria, AM: 5351
# Gkovaris Christos-Grigorios, AM: 5203

import ast                                       # Module for safely evaluating string expressions to Python objects
import rospy                                     # ROS Python client library
import argparse                                  # Module for parsing command-line arguments
from math import radians                         # Function to convert degrees to radians
import matplotlib.pyplot as plt                  # Library for plotting graphs
from geometry_msgs.msg import Twist              # Message type for velocity commands

# Store position and velocity data
time_data = []
linear_velocity_data = []
angular_velocity_data = []

def move_jackal(rate_hz, linear_velocity, angular_velocity_deg, duration):
    global time_data, linear_velocity_data, angular_velocity_data

    # Convert angular velocity from degrees to radians
    angular_velocity = [radians(angular_velocity_deg[0]), radians(angular_velocity_deg[1]), radians(angular_velocity_deg[2])]

    # Publisher to the /jackal_velocity_controller/cmd_vel topic to send velocity commands
    pub = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist, queue_size=10)
    
    # Initialize the ROS node with the name 'jackal_controller'
    rospy.init_node('jackal_controller', anonymous=True)
    
    # Wait for the publisher to be ready
    rospy.sleep(2)

    # Set the loop rate
    rate = rospy.Rate(rate_hz)
    
    # Create a Twist message instance to set the linear and angular velocities
    vel_msg = Twist()
    vel_msg.linear.x = linear_velocity[0]
    vel_msg.linear.y = linear_velocity[1]
    vel_msg.linear.z = linear_velocity[2]
    vel_msg.angular.x = angular_velocity[0]
    vel_msg.angular.y = angular_velocity[1]
    vel_msg.angular.z = angular_velocity[2]

    rospy.loginfo(f"Moving Jackal with linear velocity: {linear_velocity} and angular velocity: {angular_velocity} (radians) for duration: {duration} seconds")

    # Calculate the end time for the movement
    end_time = rospy.Time.now() + rospy.Duration(duration)
    
    # Loop until the current time reaches the end time
    while rospy.Time.now() < end_time:
        current_time = rospy.Time.now().to_sec()
        time_data.append(current_time)
        linear_velocity_data.append([vel_msg.linear.x, vel_msg.linear.y, vel_msg.linear.z])
        angular_velocity_data.append([vel_msg.angular.x, vel_msg.angular.y, vel_msg.angular.z])
        
        # Publish the velocity command
        pub.publish(vel_msg) 

        # Sleep to maintain the loop rate                                                                 
        rate.sleep()  

    # Stop the robot after the movement by setting velocities to zero
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    
    # Publish the stop command
    pub.publish(vel_msg)
    rospy.loginfo("Stopping Jackal")

def plot_data():
    global time_data, linear_velocity_data, angular_velocity_data
    
    # Transpose to get separate lists for x, y, z
    linear_velocity_data = list(zip(*linear_velocity_data))
    angular_velocity_data = list(zip(*angular_velocity_data))
    
    plt.figure(figsize=(12, 8))

    # Plot linear velocities
    plt.subplot(2, 1, 1)
    plt.plot(time_data, linear_velocity_data[0], label='Linear Velocity X')
    plt.plot(time_data, linear_velocity_data[1], label='Linear Velocity Y')
    plt.plot(time_data, linear_velocity_data[2], label='Linear Velocity Z')
    plt.xlabel('Time (s)')
    plt.ylabel('Linear Velocity (m/s)')
    plt.legend()
    plt.grid(True)
    plt.title('Linear Velocities Over Time')

    # Plot angular velocities
    plt.subplot(2, 1, 2)
    plt.plot(time_data, angular_velocity_data[0], label='Angular Velocity X')
    plt.plot(time_data, angular_velocity_data[1], label='Angular Velocity Y')
    plt.plot(time_data, angular_velocity_data[2], label='Angular Velocity Z')
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Velocity (rad/s)')
    plt.legend()
    plt.grid(True)
    plt.title('Angular Velocities Over Time')

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Move the Jackal robot with given velocities and duration.')
    parser.add_argument('rate_hz', type=int, help='Transmission frequency in Hz')
    parser.add_argument('linear_velocity', type=str, help='Linear velocity as a list [x,y,z]')
    parser.add_argument('angular_velocity_deg', type=str, help='Angular velocity in degrees as a list [x,y,z]')
    parser.add_argument('duration', type=int, help='Duration to run the robot in seconds')

    args = parser.parse_args()

    # Safely evaluate the string representations of the lists to actual lists
    linear_velocity = ast.literal_eval(args.linear_velocity)
    angular_velocity_deg = ast.literal_eval(args.angular_velocity_deg)

    try:
        move_jackal(args.rate_hz, linear_velocity, angular_velocity_deg, args.duration)
        # Call plot_data to display the plots after movement
        plot_data()  
    except rospy.ROSInterruptException:
        # Handle ROS interrupt exception
        pass