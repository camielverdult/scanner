import pandas as pd
import matplotlib.pyplot as plt

def get_moving_avg_data(data: pd.Series, alpha=0.03):
    return data.ewm(alpha=alpha).mean()

# Load the CSV file into a DataFrame
data = pd.read_csv("ble_hid_server_rssi.csv")

# Each measurement is approx. 250ms apart
# Calculate the time in seconds by multiplying the index by 0.25 and add as a new column
data['Time (s)'] = data.index * 0.25

# Plot the data
plt.figure(figsize=(10, 6))

# Plot raw RSSI data as points and filtered data as a line
plt.plot(data['Time (s)'], data['Raw_RSSI'], label="Raw RSSI", color="orange", linestyle="None", marker="o", markersize=2.5)
# plt.plot(data['Time (s)'], get_moving_avg_data(data['Raw_RSSI'], 0.2), label="Filtered RSSI (a = 0.2)", color="green", linestyle="--", linewidth=1.5)

initial_filtered = get_moving_avg_data(data['Raw_RSSI'], 0.3)
plt.plot(data['Time (s)'], initial_filtered, label="Filtered RSSI (a = 0.3)", color="blue", linestyle="--", linewidth=1.5)

filtered_second = get_moving_avg_data(initial_filtered, 0.3)
filtered_third = get_moving_avg_data(filtered_second, 0.3)
filtered_fourth = get_moving_avg_data(filtered_third, 0.3)
filtered_fifth = get_moving_avg_data(filtered_fourth, 0.3)

plt.plot(data['Time (s)'], filtered_fifth, label="Filtered RSSI (a = 0.3)^5", color="red", linestyle="--", linewidth=1.5)

# Add labels and title
plt.xlabel("Time (s)")
plt.ylabel("RSSI (dBm)")
plt.title("RSSI Measurements Over Time")
plt.legend(loc="upper right")

# Show the plot until -80 dBm
plt.ylim(-80, -20)
plt.xlim(0, len(data) * 0.25)

# Display grid for easier reading
plt.grid(True, linestyle="--", alpha=0.6)

# Add a grey area to indicate the minimum acceptable RSSI level
plt.axhspan(-75, -100, color="grey", alpha=0.3)

# Save the plot as an image file
plt.savefig(data.columns[0] + "_plot.png", dpi=300)

# Show the plot
plt.show()
