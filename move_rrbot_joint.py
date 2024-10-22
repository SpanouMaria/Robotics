#!/usr/bin/env python3

# Team 01
# Spanou Maria, AM: 5351
# Gkovaris Christos-Grigorios, AM: 5203

import sys                                       # For accessing command-line arguments and exiting the program
import rospy                                     # ROS Python client library
import numpy as np                               # For numerical operations, specifically to create arrays of joint angles
import matplotlib.pyplot as plt                  # For plotting the joint positions over time
from std_msgs.msg import Float64                 # For using Float64 ROS message type to publish joint positions

# Initialize lists to store time and joint positions
time_data = []
q1_data = []
q2_data = []

def main(joint_type, start_angle, end_angle):
    # Initialize ROS node
    rospy.init_node("robot_move2", anonymous=True)

    # Publishers for joint positions
    pub1 = rospy.Publisher("/rrbot/joint1_position_controller/command", Float64, queue_size=10)
    pub2 = rospy.Publisher("/rrbot/joint2_position_controller/command", Float64, queue_size=10)
    
    # 10 Hz rate for publishing
    rate = rospy.Rate(10)  

    # Start time of the movement
    start = rospy.get_time()  
    
    # Duration of the movement in seconds
    duration = 15.0

    # Number of steps for the rate of 10 Hz  
    steps = int(duration * 10)  

    # Create an array of joint angles to move from start_angle to end_angle
    if joint_type == "q1":
        joint1_angles = np.linspace(start_angle, end_angle, steps)
        # Keep joint2 static
        joint2_angles = [0.0] * steps  
    elif joint_type == "q2":
        joint2_angles = np.linspace(start_angle, end_angle, steps)
        # Keep joint1 static
        joint1_angles = [0.0] * steps  
    else:
        print("Invalid joint type. Please specify 'q1' or 'q2'.")
        # Exit if the joint type is invalid
        sys.exit(1)

    for i in range(steps):
        if rospy.is_shutdown():
            # Exit loop if ROS is shut down
            break  

        msg1 = Float64()
        msg2 = Float64()
        msg1.data = joint1_angles[i]
        msg2.data = joint2_angles[i]

        # Append data for plotting
        q1_data.append(msg1.data)
        q2_data.append(msg2.data)
        time_data.append(rospy.get_time() - start)

        # Publish messages to the topics
        pub1.publish(msg1)
        pub2.publish(msg2)

        # Sleep to maintain the loop rate
        rate.sleep()  

    # Plot the results
    plt.figure()
    plt.plot(time_data, q1_data, label='Joint 1 Position (rad)')
    plt.plot(time_data, q2_data, label='Joint 2 Position (rad)')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (rad)')
    plt.legend()
    plt.title('Joint Positions Over Time')
    plt.show()

if __name__ == '__main__':
    if len(sys.argv) != 4:
        print("Usage: robot_move2.py <joint_type> <start_angle_degrees> <end_angle_degrees>")
        # Exit if the number of arguments is incorrect
        sys.exit(1)  

    # Get the joint type and angles from command-line arguments
    joint_type = sys.argv[1]
    
    # Convert to radians
    start_angle = float(sys.argv[2]) * (3.141592653589793 / 180.0)  
    end_angle = float(sys.argv[3]) * (3.141592653589793 / 180.0) 
    
    try:
        # Run the main function
        main(joint_type, start_angle, end_angle)  
    except rospy.ROSInterruptException:
        # Handle ROS interrupt exception
        pass  