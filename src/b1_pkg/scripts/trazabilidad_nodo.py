#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
import pyrebase
import time

# Configuración de Firebase
config = {
    "apiKey": "AIzaSyAddBEou-qdzKj7fX2M1s5dgW6eg5In53Y",
    "authDomain": "a01742247.firebaseapp.com",
    "databaseURL": "https://a01742247-default-rtdb.firebaseio.com",
    "projectId": "a01742247",
    "storageBucket": "a01742247.appspot.com",
    "messagingSenderId": "280761398173",
    "appId": "1:280761398173:web:0d751e6c4eed6397ee7574"
}

firebase = pyrebase.initialize_app(config)
db = firebase.database()

place_b1_status = False

# Function to check the "OMRON Out of Reach" and "State" status
def check_omrom_state():
    # Query for pieces where "OMRON Out of Reach" is "Out of Reach"
    pieces = db.child("Acuity Process").order_by_child("OMRON Out of Reach").equal_to("Out of Reach").get()

    for pieza in pieces.each():
        # Check if the piece also has the "State" as "Waiting at Rework Table"
        if pieza.val().get("State") == "Waiting at Rework Table":
            return True  # Condition met, return True
    
    return False  # No piece met both conditions


def omron_status_callback(msg):
    global place_b1_status
    # Update the global variable with the message received from the topic
    place_b1_status = msg.data
    rospy.loginfo(f"Received /place_b1: {place_b1_status}")

def start_trazability():
    global place_b1_status
    # Initialize the ROS node
    rospy.init_node('trazability', anonymous=True)
    bool_publisher = rospy.Publisher('/omron_status', Bool, queue_size=10)

    rospy.Subscriber("/place_b1", Bool, omron_status_callback)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        omron_state_detected = check_omrom_state()

        # Publish the boolean value to /omron_status
        bool_publisher.publish(Bool(omron_state_detected))

        if place_b1_status:
            piezas = db.child("Acuity Process").order_by_child("Piece").equal_to("Blue").get()  # Example: "Blue" as piece color
            for pieza in piezas.each():
                estado = "Waiting at Rework Station"
                update_data = {
                    "State": estado
                }
                db.child("Acuity Process").child(pieza.key()).update(update_data)
                rospy.loginfo(f"State updated for piece {pieza.key()} to '{estado}'")
        rate.sleep()

if __name__ == '__main__':
    try:
        start_trazability()
    except rospy.ROSInterruptException:
        pass
