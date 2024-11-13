import pandas as pd
import matplotlib.pyplot as plt

def get_moving_avg_data(data: pd.Series, alpha=0.03):
    return data.ewm(alpha=alpha).mean()

file_name = "ble_server_rssi.csv"

# Load the CSV file into a DataFrame
data = pd.read_csv(file_name)

# Each measurement is approx. 200ms apart
time_interval = 0.2

# Calculate the time in seconds by multiplying the index by 0.25 and add as a new column
data['Time (s)'] = data.index * time_interval

# Plot the data
plt.figure(figsize=(10, 6))

# Plot raw RSSI data as points and filtered data as a line
plt.plot(data['Time (s)'], data['Raw_RSSI'], label="Raw RSSI", color="orange", linestyle="None", marker="o", markersize=2.5)

initial_filtered = get_moving_avg_data(data['Raw_RSSI'], 0.3)
plt.plot(data['Time (s)'], initial_filtered, label="Filtered RSSI (a = 0.3)", color="blue", linestyle="--", linewidth=1.5)

# filtered_second = get_moving_avg_data(initial_filtered, 0.3)
# filtered_third = get_moving_avg_data(filtered_second, 0.3)
# filtered_fourth = get_moving_avg_data(filtered_third, 0.3)
# filtered_fifth = get_moving_avg_data(filtered_fourth, 0.3)

# plt.plot(data['Time (s)'], filtered_fifth, label="Filtered RSSI (a = 0.3)^5", color="red", linestyle="--", linewidth=1.5)

# Plot 1.5IQR lines
q1 = data['Raw_RSSI'].quantile(0.25)
q3 = data['Raw_RSSI'].quantile(0.75)
iqr = q3 - q1

plt.axhline(q1 - 1.5 * iqr, color="red", linestyle="--", label="1.5IQR range")
plt.axhline(q3 + 1.5 * iqr, color="red", linestyle="--")

# Calculate mean and standard deviation
mean = data['Raw_RSSI'].mean()
std = data['Raw_RSSI'].std()
print(f"Q1: {q1}, Q3: {q3}, IQR: {iqr}, Mean: {mean}, Std: {std}")

# Add labels and title
plt.xlabel("Time (s)")
plt.ylabel("RSSI (dBm)")
plt.title("RSSI Measurements over Time")
plt.legend(loc="upper right")

# Show the plot until -80 dBm
max_rssi = data['Raw_RSSI'].max()
plt.ylim(-80, max_rssi + 10)
plt.xlim(0, len(data) * time_interval)

# Display grid for easier reading
plt.grid(True, linestyle="--", alpha=0.6)

# Add a grey area to indicate the minimum acceptable RSSI level
plt.axhspan(-75, -100, color="grey", alpha=0.3)

# Add text for gray area
plt.text(1, -79, "Minimum acceptable RSSI", fontsize=10, color="black")

# Save the plot as an image file
plt.savefig(f"{file_name[:-4]}_plot.png", dpi=300)

# Show the plot
plt.show()
