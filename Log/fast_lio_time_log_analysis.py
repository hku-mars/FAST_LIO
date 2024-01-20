import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV data
csv_file_path = 'fast_lio_time_log.csv'
fast_lio_data = pd.read_csv(csv_file_path, header=0)

# csv file format:
# time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time
fast_lio_data['timestamp'] = fast_lio_data.iloc[:, 0] - fast_lio_data.iloc[:, 0].min()
fast_lio_data['total_time'] = fast_lio_data.iloc[:, 1] * 1e3
fast_lio_data['incremental_time'] = fast_lio_data.iloc[:, 3] * 1e3
fast_lio_data['search_time'] = fast_lio_data.iloc[:, 4] * 1e3
fast_lio_data['delete_time'] = fast_lio_data.iloc[:, 6] * 1e3
fast_lio_data['fov_check_time'] = fast_lio_data.iloc[:, 4] * 1e3
fast_lio_data['add_point_size'] = fast_lio_data.iloc[:, 9]
fast_lio_data['preprocess_time'] = fast_lio_data.iloc[:, 10] * 1e3
fast_lio_data['tree_size'] = fast_lio_data.iloc[:, 8]
fast_lio_data['delete_size'] = fast_lio_data.iloc[:, 5]
fast_lio_data['scan_point_size'] = fast_lio_data.iloc[:, 2]
fast_lio_data['tree_size'] = fast_lio_data.iloc[:, 8]


# Define colors for the plots
colors = {
    "red": [0.6350, 0.0780, 0.1840],
    "blue": [0, 0.4470, 0.7410],
    "orange": [0.8500, 0.3250, 0.0980],
    "green": [0.4660, 0.6740, 0.1880],
    "lightblue": [0.3010, 0.7450, 0.9330],
    "purple": [0.4940, 0.1840, 0.5560],
    "yellow": [0.9290, 0.6940, 0.1250],
    "black": [0, 0, 0],
    "gray": [0.5, 0.5, 0.5],
    "lightgray": [0.8, 0.8, 0.8]
}

# Create multiple subplots
fig, axs = plt.subplots(6, 2, figsize=(15, 20))

axs[0, 0].plot(fast_lio_data['timestamp'], fast_lio_data['total_time'], color=colors['blue'])
axs[0, 0].set_title('Total Time')
axs[0, 0].set_xlabel('Time (s)')
axs[0, 0].set_ylabel('Total Time (ms)')

axs[0, 1].plot(fast_lio_data['timestamp'], fast_lio_data['incremental_time'], color=colors['green'])
axs[0, 1].set_title('Incremental Time')
axs[0, 1].set_xlabel('Time (s)')
axs[0, 1].set_ylabel('Incremental Time (ms)')

axs[1, 0].plot(fast_lio_data['timestamp'], fast_lio_data['search_time'], color=colors['red'])
axs[1, 0].set_title('Search Time')
axs[1, 0].set_xlabel('Time (s)')
axs[1, 0].set_ylabel('Search Time (ms)')

axs[1, 1].plot(fast_lio_data['timestamp'], fast_lio_data['delete_time'], color=colors['orange'])
axs[1, 1].set_title('Delete Time')
axs[1, 1].set_xlabel('Time (s)')
axs[1, 1].set_ylabel('Delete Time (ms)')

axs[2, 0].plot(fast_lio_data['timestamp'], fast_lio_data['fov_check_time'], color=colors['purple'])
axs[2, 0].set_title('FOV Check Time')
axs[2, 0].set_xlabel('Time (s)')
axs[2, 0].set_ylabel('FOV Check Time (ms)')

axs[2, 1].plot(fast_lio_data['timestamp'], fast_lio_data['add_point_size'], color=colors['yellow'])
axs[2, 1].set_title('Add Point Size')
axs[2, 1].set_xlabel('Time (s)')
axs[2, 1].set_ylabel('Add Point Size')

axs[3, 0].plot(fast_lio_data['timestamp'], fast_lio_data['preprocess_time'], color=colors['lightblue'])
axs[3, 0].set_title('Preprocess Time')
axs[3, 0].set_xlabel('Time (s)')
axs[3, 0].set_ylabel('Preprocess Time (ms)')

axs[3, 1].plot(fast_lio_data['timestamp'], fast_lio_data['tree_size'], color=colors['black'])
axs[3, 1].set_title('Tree Size')
axs[3, 1].set_xlabel('Time (s)')
axs[3, 1].set_ylabel('Tree Size')

axs[4, 0].plot(fast_lio_data['timestamp'], fast_lio_data['delete_size'], color=colors['gray'])
axs[4, 0].set_title('Delete Size')
axs[4, 0].set_xlabel('Time (s)')
axs[4, 0].set_ylabel('Delete Size')

axs[4, 1].plot(fast_lio_data['timestamp'], fast_lio_data['scan_point_size'], color=colors['lightgray'])
axs[4, 1].set_title('Scan Point Size')
axs[4, 1].set_xlabel('Time (s)')
axs[4, 1].set_ylabel('Scan Point Size')

axs[5, 0].plot(fast_lio_data['timestamp'], fast_lio_data['delete_size'], color=colors['gray'])
axs[5, 0].set_title('Delete Size')
axs[5, 0].set_xlabel('Time (s)')
axs[5, 0].set_ylabel('Delete Size')

axs[5, 1].plot(fast_lio_data['timestamp'], fast_lio_data['scan_point_size'], color=colors['lightgray'])
axs[5, 1].set_title('Scan Point Size')
axs[5, 1].set_xlabel('Time (s)')
axs[5, 1].set_ylabel('Scan Point Size')

plt.tight_layout()
plt.show()
