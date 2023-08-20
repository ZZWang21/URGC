#!/usr/bin/env python

import rosbag
import pandas as pd
from Crypto.Cipher import AES

input_bag = '~/Desktop/filtered_data.bag'  # Replace with the actual bag file name
output_csv = '~/Desktop/combined_data.csv'  # Replace with the desired CSV file name
topics = ['/wrench', '/target_wrench']  # Replace with the actual topic names

bag = rosbag.Bag(input_bag)
data = {topic: [] for topic in topics}

for topic, msg, t in bag.read_messages(topics=topics):
    data[topic].append((t, msg))

bag.close()

dfs = []
for topic, msgs in data.items():
    df = pd.DataFrame(msgs, columns=['time', 'message'])
    dfs.append(df)

combined_df = pd.concat(dfs, axis=1)
combined_df.to_csv(output_csv, index=False)