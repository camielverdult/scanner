import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file into a DataFrame
data = pd.read_csv("ble_hid_server_rssi.csv")

# Plot both Filtered_RSSI and Raw values
plt.figure(figsize=(10, 6))
plt.plot(data['Filtered_RSSI'], label="Filtered RSSI", color="blue", linewidth=1.5)
plt.plot(data['Raw_RSSI'], label="Raw RSSI", color="orange", linestyle="--", linewidth=1)

# Add labels and title
plt.xlabel("Measurement Index")
plt.ylabel("RSSI (dBm)")
plt.title("RSSI Measurements Over Time")
plt.legend(loc="upper right")

# Display grid for easier reading
plt.grid(True, linestyle="--", alpha=0.6)

# Save the plot as an image file
plt.savefig(data.columns[0] + "_plot.png", dpi=300)

# Show the plot
plt.show()
