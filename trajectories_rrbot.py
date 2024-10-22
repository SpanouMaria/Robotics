#!/usr/bin/env python3

# Team 01
# Spanou Maria, AM: 5351
# Gkovaris Christos-Grigorios, AM: 5203

import rospy                                     # ROS Python client library for interacting with ROS
from std_srvs.srv import Empty                   # Service type for empty service calls
import matplotlib.pyplot as plt                  # Library for plotting graphs
from std_msgs.msg import Float64                 # Standard ROS message type for publishing float64 data

# Lists to store time, joint positions, and velocities
time_data = []
joint1_positions = []
joint2_positions = []
joint1_velocities = []
joint2_velocities = []

# Function to reset the simulation time in Gazebo
def reset_gazebo_simulation_time():
    # Wait for the reset simulation service to become available
    rospy.wait_for_service('/gazebo/reset_simulation')  
    try:
        # Create a service proxy for the reset simulation service
        reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)  
        # Call the service to reset the simulation time
        reset_simulation()  
        # Log information about the successful reset
        rospy.loginfo("Gazebo simulation time reset.")  
    except rospy.ServiceException as e:
        # Log an error message if the service call fails
        rospy.logerr("Service call failed: %s" % e)  

# Function to calculate velocities from positions and time data
def calculate_velocities(positions, time_data):
    velocities = []
    # Iterate through the position data starting from the second element
    for i in range(1, len(positions)):  
        # Calculate the velocity as the difference in position divided by the difference in time
        velocity = (positions[i] - positions[i-1]) / (time_data[i] - time_data[i-1])  
        # Append the calculated velocity to the velocities list
        velocities.append(velocity)  
    return velocities

# Function to plot the joint positions and velocities over time
def plot_data():
    global joint1_velocities, joint2_velocities

    # Calculate velocities for joint 1 and 2
    joint1_velocities = calculate_velocities(joint1_positions, time_data)  
    joint2_velocities = calculate_velocities(joint2_positions, time_data)  

    # Create a new figure for plotting with specified size
    plt.figure(figsize=(12, 8))  

    # Create the first subplot for joint positions
    plt.subplot(2, 1, 1)  
    plt.plot(time_data, joint1_positions, label='Joint 1 Position')  
    plt.plot(time_data, joint2_positions, label='Joint 2 Position') 
    plt.xlabel('Time (s)')  
    plt.ylabel('Joint Position (rad)') 
    plt.legend() 
    plt.grid(True)  
    plt.title('Joint Positions Over Time')  

    # Create the second subplot for joint velocities
    plt.subplot(2, 1, 2)  
    plt.plot(time_data[1:], joint1_velocities, label='Joint 1 Velocity') 
    plt.plot(time_data[1:], joint2_velocities, label='Joint 2 Velocity')  
    plt.xlabel('Time (s)')  
    plt.ylabel('Joint Velocity (rad/s)')
    plt.legend()  
    plt.grid(True)  
    plt.title('Joint Velocities Over Time')  

    # Adjust subplots to fit into the figure area
    plt.tight_layout()  
    
    # Display the plot
    plt.show()  

# Main function to control the robot joint positions
def main():
    global time_data, joint1_positions, joint2_positions

    # Initialize a new ROS node named "robot_trajectory2"
    rospy.init_node("robot_trajectory2", anonymous=True)  

    # Reset the Gazebo simulation time
    reset_gazebo_simulation_time()  

    # Create publishers for joint positions
    # Publisher for joint 1 positions
    pub1 = rospy.Publisher("/rrbot/joint1_position_controller/command", Float64, queue_size=10)  
    
    # Publisher for joint 2 positions
    pub2 = rospy.Publisher("/rrbot/joint2_position_controller/command", Float64, queue_size=10)  
    
    # Set the publishing rate to 10 Hz
    rate = rospy.Rate(10)  

    # Duration of the movement in seconds
    tf = 11.0  

    # Get the start time
    start = rospy.Time.now()  

    # Loop until ROS is shutdown
    while not rospy.is_shutdown():  
        # Calculate the elapsed time
        t = (rospy.Time.now() - start).to_sec()  

        # Exit the loop if the elapsed time exceeds the duration
        if t > tf:                               
            break

        # Define the desired trajectories for the joints
        msg1 = Float64()
        msg2 = Float64()

        # Calculate the position for joint 1 and 2
        msg1.data = 0.03 * (t ** 2) - 0.002 * (t ** 3)  
        msg2.data = -0.02 * (t ** 2) + 0.001 * (t ** 3)

        
        pub1.publish(msg1)                       # Publish the position for joint 1  
        pub2.publish(msg2)                       # Publish the position for joint 2

        # Store data
        time_data.append(t)                      # Append the current time to the time data list
        joint1_positions.append(msg1.data)       # Append the position of joint 1 to the joint 1 positions list
        joint2_positions.append(msg2.data)       # Append the position of joint 2 to the joint 2 positions list

        # Sleep to maintain the loop rate
        rate.sleep()  

    # Plot the data after the loop ends
    plot_data()  

if __name__ == '__main__':
    try:
        # Run the main function
        main()  
    except rospy.ROSInterruptException:
        # Handle ROS interrupt exception
        pass  