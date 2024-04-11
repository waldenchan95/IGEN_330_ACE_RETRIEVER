import serial
import time

# Define the serial port (adjust this based on your system)
serial_port = 'COM5'  # Change this to match your Arduino's serial port
baud_rate = 115200

# # Initialize the serial connection
ser = serial.Serial(serial_port, baud_rate, timeout=None)
time.sleep(1)  # Wait for the Arduino to initialize

# Define the array to send to Arduino
data_array = [100, 120, 200, 235, 59, 80, 103, 167, 197, 202, 250, 100, 120, 170, 180, 180]
node_8 = [100, 200]
#data_array = [18, 12, 20, 25, 59, 8, 3, 17, 19, 20, 25, 10, 12, 17, 18, 18]
data_bytes = bytes(data_array)
i = 0
while i < 5: #try writing it 5 times
    ser.write(data_bytes)
    acknowledged = False
    timeout = time.time() + 5  # Wait for 5 seconds
    while time.time() < timeout:
        if ser.in_waiting > 0:
            response = ser.readline().strip().decode('utf-8')
            if response == 'ACK':
                acknowledged = True
                break
    i=i+1

    # Send the bytes
    # ser.write(data_bytes)
    # for value in data_array:
    #     for i in value:
    #         ser.write(i.to_bytes(1, byteorder='little'))
            # line = ser.readline().decode().strip()
            # if line.isdigit():
            #     print(int(line))


# Send the array size as the first byte
# ser.write(len(data_array).to_bytes(1, byteorder='little'))
# time.sleep(0.1)  # Wait for transmission

# # Send each element of the array to Arduino
# for value in data_array:
#     ser.write(value.to_bytes(1, byteorder='little'))
#     time.sleep(0.1)  # Wait for transmission

# Close the serial connection
