#!/usr/bin/env python

import rosbag
import csv

bag = rosbag.Bag('/home/walter/Desktop/filtered_data.bag')
csv_file = '/home/walter/Desktop/output.csv'

with open(csv_file, 'w', newline='') as csvfile:
    csvwriter = csv.writer(csvfile)
    csvwriter.writerow(['Timestamp', 'Topic', 'Linear Force X', 'Linear Force Y', 'Linear Force Z', 'Angular Torque X', 'Angular Torque Y', 'Angular Torque Z'])

    for topic, msg, t in bag.read_messages():
        if topic == '/your/wrench_stamped/topic':  # Replace with the actual topic name
            timestamp = t.to_sec()
            linear_force = msg.wrench.force
            angular_torque = msg.wrench.torque
            data_row = [timestamp, topic, linear_force.x, linear_force.y, linear_force.z, angular_torque.x, angular_torque.y, angular_torque.z]
            csvwriter.writerow(data_row)

bag.close()