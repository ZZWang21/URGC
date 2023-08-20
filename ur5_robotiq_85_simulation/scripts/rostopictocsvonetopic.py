#!/usr/bin/env python

import rospy
import csv
from geometry_msgs.msg import WrenchStamped

# Callback function for the /wrench/wrench/force/z topic
def wrench_callback(msg):
    global wrench_data
    z_force = msg.wrench.force.y
    print(z_force)
    wrench_data.append([rospy.Time.now(), z_force])
    print(rospy.Time.now())
    print("Received wrench data:", z_force)


def main():
    global wrench_data, target_wrench_data

    rospy.init_node('ros_topics_to_csv')

    # Create subscribers for the topics
    rospy.Subscriber('/wrench', WrenchStamped, wrench_callback) #/wrench/force/y
    
    # Initialize empty lists to store data
    wrench_data = []
    
    # Wait for incoming messages and process callbacks
    rospy.spin()

    # Write collected data to CSV file
    csv_file = '/home/walter/Desktop/outpuwrench.csv'  # Replace with the desired output file path
    with open(csv_file, 'w', newline='') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow(['Timestamp', 'Wrench Z'])

        for data in wrench_data:
            timestamp, wrench_z = data
            #target_data = next((d[1] for d in target_wrench_data if d[0] == timestamp), None)
            if data is not None:
                csvwriter.writerow([timestamp, wrench_z])

if __name__ == '__main__':
    main()
