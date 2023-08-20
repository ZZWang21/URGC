#!/usr/bin/env python

import rospy
import csv
from geometry_msgs.msg import WrenchStamped
from datetime import datetime  # Import datetime module

# Callback function for the /wrench/wrench/force/z topic
def wrench_callback(msg):
    global wrench_data
    z_force = msg.wrench.force.z
    wrench_data.append([rospy.Time.now(), z_force])
    print("Received wrench data:", z_force)

# Callback function for the /target_wrench/wrench/force/z topic
def target_wrench_callback(msg):
    global target_wrench_data
    z_force = msg.wrench.force.z
    target_wrench_data.append([rospy.Time.now(), z_force])

def main():
    global wrench_data, target_wrench_data

    rospy.init_node('ros_topics_to_csv')

    # Create subscribers for the topics
    rospy.Subscriber('/wrench', WrenchStamped, wrench_callback) #/wrench/force/z
    rospy.Subscriber('/target_wrench', WrenchStamped, target_wrench_callback) #/wrench/force/z

    # Initialize empty lists to store data
    wrench_data = []
    target_wrench_data = []

    # Wait for incoming messages and process callbacks
    rospy.spin()

    # Write collected data to CSV file
    csv_file = '/home/walter/Desktop/output202308160616.csv'  # Replace with the desired output file path
    with open(csv_file, 'w', newline='') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow(['Timestamp', 'Wrench Z', 'Target Wrench Z'])

        for data in wrench_data:
            timestamp, wrench_z = data
            target_data = next((d[1] for d in target_wrench_data if d[0] == timestamp), None)
            if target_data is not None: 
                csvwriter.writerow([timestamp, wrench_z, target_data])

if __name__ == '__main__':
    main()
