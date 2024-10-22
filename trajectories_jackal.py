#!/usr/bin/env python3

# Team 01
# Spanou Maria, AM: 5351
# Gkovaris Christos-Grigorios, AM: 5203

import rospy                                     # ROS Python client library
from std_srvs.srv import Empty                   # Service type for empty service calls
import matplotlib.pyplot as plt                  # Library for plotting graphs
from geometry_msgs.msg import Twist              # Message type for velocity commands

# Store position and velocity data
linear_velocities = []
linear_velocities2 = []
angular_velocities = []
time_data = []
position_x = []
position_y = []

angle_theta = []
angle_theta2 = []
combined_theta = []

# Initialize theta, x, y for storing position and angle trajectories
theta = 0.0
theta2 = 0.0
x = 0.0
y = 0.0

def reset_gazebo_simulation_time():
    # Wait for the Gazebo simulation reset service to become available
    rospy.wait_for_service('/gazebo/reset_simulation')
    try:
        # Call the reset simulation service
        reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        reset_simulation()
        rospy.loginfo("Gazebo simulation time reset.")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

def plot_data():
    plt.figure(figsize=(15, 12))

    # Plot linear velocities
    plt.subplot(4, 1, 1)
    plt.plot(time_data, linear_velocities, label='Linear Velocity X')
    plt.plot(time_data, linear_velocities2, label='Linear Velocity Y')
    plt.xlabel('Time (s)')
    plt.ylabel('Linear Velocity (m/s)')
    plt.legend()
    plt.grid(True)
    plt.title('Linear Velocities Over Time')

    # Plot angular velocities
    plt.subplot(4, 1, 2)
    plt.plot(time_data, angular_velocities, label='Angular Velocity Z', color='g')
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Velocity (rad/s)')
    plt.legend()
    plt.grid(True)
    plt.title('Angular Velocity Over Time')

    # Plot positions
    plt.subplot(4, 1, 3)
    plt.plot(time_data, position_x, label='Position X')
    plt.plot(time_data, position_y, label='Position Y')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.legend()
    plt.grid(True)
    plt.title('Position Over Time')

    # Plot combined theta
    plt.subplot(4, 1, 4)
    plt.plot(time_data, combined_theta, label='Combined Theta (Theta + Theta2)', color='purple')
    plt.xlabel('Time (s)')
    plt.ylabel('Combined Angle (rad)')
    plt.legend()
    plt.grid(True)
    plt.title('Combined Theta (Theta + Theta2) Over Time')

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node("jackal_trajectories")

    # Create a publisher for the /cmd_vel topic
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    # Reset the simulation time
    reset_gazebo_simulation_time()

    # Set the rate to 10 Hz
    rate = rospy.Rate(10)        

    # Get the start time           
    start_time = rospy.get_time()           

    # Initialize the last_time variable
    last_time = start_time                  

    while not rospy.is_shutdown():
        msg = Twist()
    
        # Trajectory 1
        current_time = rospy.get_time()          # Get the current time
        t = current_time - start_time            # Calculate the elapsed time
        elapsed_time = current_time - last_time  # Calculate the elapsed time since the last iteration
        last_time = current_time                 # Update last_time to the current time

        print(f"Current time: {current_time:.3f}, Start time: {start_time:.3f}, Elapsed time: {t:.3f}")
        
        # Define the angular velocity for different time intervals
        if t >= 0 and t <= 0.12:
            theta += 4.16 * t * elapsed_time
            msg.angular.z = 4.16 * t
            
        elif t > 0.12 and t <= 1.08:
            theta += 0.49 * elapsed_time
            msg.angular.z = 0.49
            
        elif t > 1.08 and t <= 1.2:
            theta += (-4.16 * t + 4.992) * elapsed_time
            msg.angular.z = -4.16 * t + 4.992
        
        # Define the linear velocity for different time intervals
        if t > 1.2 and t <= 4.4:
            x += 0.05 * (t - 1.2) * elapsed_time
            y += 0.03 * (t - 1.2) * elapsed_time
            msg.linear.x = 0.05 * (t - 1.2)
            msg.linear.y = 0.03 * (t - 1.2)
            
        elif t > 4.4 and t <= 33.2:
            x += 0.16 * elapsed_time
            y += 0.096 * elapsed_time
            msg.linear.x = 0.16
            msg.linear.y = 0.096
            
        elif t > 33.2 and t <= 36.4:
            x += (-0.05 * (t - 4.4) + 1.6) * elapsed_time
            y += (-0.03 * (t - 4.4) + 0.96) * elapsed_time
            msg.linear.x = -0.05 * (t - 4.4) + 1.6
            msg.linear.y = -0.03 * (t - 4.4) + 0.96
        
        # Define the second angular velocity for different time intervals
        if t > 36.4 and t <= 36.602:
            theta2 += 2.58 * (t - 36.4) * elapsed_time
            msg.angular.z = 2.58 * (t - 36.4)
            
        elif t > 36.602 and t <= 38.42:
            theta2 += 0.52 * elapsed_time
            msg.angular.z = 0.52
            
        elif t > 38.42 and t <= 38.622:
            theta2 += (-2.58 * (t - 36.602) + 5.2) * elapsed_time
            msg.angular.z = -2.58 * (t - 36.602) + 5.2
            
        elif t > 38.622:
            break

        # Store velocities, position, and angle data
        angular_velocities.append(msg.angular.z)
        linear_velocities.append(msg.linear.x)
        linear_velocities2.append(msg.linear.y)
        angle_theta.append(theta)
        angle_theta2.append(theta2)
        combined_theta.append(theta + theta2)
        position_y.append(y)
        position_x.append(x)
        time_data.append(t)
        
        # Publish the velocity command
        pub.publish(msg)  

        # Sleep to maintain the loop rate
        rate.sleep()  

    # Ensure the robot stops at the end of the trajectory
    msg.linear.x = 0
    msg.linear.y = 0
    msg.linear.z = 0
    msg.angular.x = 0
    msg.angular.y = 0
    msg.angular.z = 0

    # Publish the stop command
    pub.publish(msg)  

    # Store the final stop state
    time_data.append(time_data[-1] + 1.0 / rate.sleep_dur.to_sec())
    linear_velocities.append(0)
    linear_velocities2.append(0)
    angular_velocities.append(0)
    position_x.append(position_x[-1])
    position_y.append(position_y[-1])
    angle_theta.append(angle_theta[-1])
    angle_theta2.append(angle_theta2[-1])
    combined_theta.append(angle_theta[-1] + angle_theta2[-1])

    # Plot data after the loop ends
    plot_data()