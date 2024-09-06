import serial
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
import math

num_scans = 12
displacement = 100
scan_number = 0
num_measurements = 32
combined_data = []

for k in range(num_scans): 

    # Initialize the serial port
    serial_port = serial.Serial(port='COM4', baudrate=115200, timeout=10)
    print("Opening: " + serial_port.name)

    data = []

    # Reset the buffers of the UART port to delete the remaining data in the buffers
    serial_port.reset_output_buffer()
    serial_port.reset_input_buffer()

    # Wait for the user's signal to start the program
    input("Press enter to start")

    # Send the character 's' to MCU via UART. This will signal MCU to start the transmission
    serial_port.write(b's')  # Assuming 's' is encoded as bytes

    for i in range(num_measurements):
        x = serial_port.readline()
        data_str = x.decode().strip()  # Remove leading/trailing whitespace and newline characters
        print(x.decode())
        if data_str.isdigit():  # Check if the string contains only digits
            data.append(int(data_str))
        
    # Close the port
    print("Closing: " + serial_port.name)
    serial_port.close()

    # Debugging information
    print("Number of data points:", len(data))

    # Write data to file
    for i in range(len(data)):  # Use the length of data to iterate
        x = data[i] * math.sin(math.radians(11.25*i))
        y = data[i] * math.cos(math.radians(11.25*i))
        z = k * displacement
        combined_data.append((x, y, z))
        print(x, y, z)
        
with open('scanned_data.xyz', 'w') as f:
    for x, y, z in combined_data:
        f.write(f"{x:f} {y:f} {z:f}\n")

# Read point cloud from file and visualize
pcd = o3d.io.read_point_cloud("scanned_data.xyz", format="xyz")
points = np.asarray(pcd.points)

o3d.visualization.draw_geometries([pcd])
points = np.asarray(combined_data)

lines_within_scan = []
for scan in range(num_scans):
    for i in range(num_measurements - 1):
        index = scan * num_measurements + i
        lines_within_scan.append([index, index + 1])

lines_between_scans = []
for scan in range(num_scans - 1):
    for i in range(num_measurements):
        current_index = scan * num_measurements + i
        next_index = (scan + 1) * num_measurements + i
        lines_between_scans.append([current_index, next_index])

# connect the last point of one scan to the first point of the next scan
all_lines = lines_within_scan + lines_between_scans

# create a lineset object to represent the lines
line_set = o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector(points),
    lines=o3d.utility.Vector2iVector(lines_within_scan + lines_between_scans)
)

# Visualize the point cloud with the lines
o3d.visualization.draw_geometries([pcd, line_set])
lines = [[i, i + 1] for i in range(len(points - 1))]
lines.append([len(points) - 1, 0])
