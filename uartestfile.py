import serial
from time import sleep

# Set up the serial connection
arduino = serial.Serial('COM10', 115200, timeout=1)
sleep(2)  # Wait for connection to establish

for i in range(30):
    # Define the three values
    angle_pitch = 1*i+90    # Between -30 and 90
    angle_yaw = 19+90      # Between -90 and 90
    info_tracking = 1   # 0 or 1

    # Combine the values into a string with commas
    data_string = f"{angle_pitch},{angle_yaw},{info_tracking}"
    # Send the data
    arduino.write(data_string.encode())
    sleep(0.3)

# Close the connection
arduino.close()
print(f"Sent: {data_string}")