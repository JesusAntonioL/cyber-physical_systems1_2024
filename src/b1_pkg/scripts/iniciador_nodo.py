#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty
from std_msgs.msg import Float32

# Global variable to store the Gz value
gz_value = None

# Callback function for the Gz topic subscriber
def gz_callback(data):
    global gz_value
    gz_value = data.data
    rospy.loginfo(f"Received Gz value: {gz_value}")

# Function to call the service that starts publishing from the Arduino
def start_publishing_client():
    rospy.wait_for_service('start_publishing')
    
    try:
        # Create a service proxy to call the 'start_publishing' service
        start_publishing = rospy.ServiceProxy('start_publishing', Empty)
        
        # Call the service
        resp = start_publishing()
        rospy.loginfo("Service call successful, Arduino should start publishing now.")
        
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node('start_publishing_client', anonymous=True)
    
    # Call the service to start publishing
    start_publishing_client()

    # Subscribe to the 'Gz' topic
    rospy.Subscriber("Gz", Float32, gz_callback)

    # Keep the node running and listening to the topic
    rospy.loginfo("Subscribed to the Gz topic. Waiting for data...")
    rospy.spin()  # This keeps the node alive and processing callbacks
