from time import sleep
from pySerialTransfer import pySerialTransfer as txfer
import serial
import time

# class struct(object):
#     x = 5.5
#     y = 4.5

# try:
#     sendStruct = struct
#     link = txfer.SerialTransfer('COM6')
    
#     link.open()
#     sleep(5)

#     while True:
#         sendSize = 0
        
#         sendSize = link.tx_obj(sendStruct.x, start_pos=sendSize)
#         sendSize = link.tx_obj(sendStruct.y, start_pos=sendSize)
        
#         link.send(sendSize)

#         if link.available():
#             recSize = 0
#             testStruct = struct
#             testStruct.x = link.rx_obj(obj_type='f', start_pos=recSize)
#             recSize += txfer.STRUCT_FORMAT_LENGTHS['f']
            
#             testStruct.y = link.rx_obj(obj_type='f', start_pos=recSize)
#             recSize += txfer.STRUCT_FORMAT_LENGTHS['f']
            
#             print('{} : {}'.format(testStruct.x, testStruct.y))
            
#         elif link.status <= 0:
#             if link.status == txfer.CRC_ERROR:
#                 print('ERROR: CRC_ERROR')
#             elif link.status == txfer.PAYLOAD_ERROR:
#                 print('ERROR: PAYLOAD_ERROR')
#             elif link.status == txfer.STOP_BYTE_ERROR:
#                 print('ERROR: STOP_BYTE_ERROR')
#             else:
#                 print('ERROR: {}'.format(link.status))
# except KeyboardInterrupt:
#     link.close()

def send_data_to_arduino(data, port):
    """
    Assuming the arrays coming from YOLO in the form [[1, 3, 'person'], [3, 4, 'sports ball'], [5, 6, 'sports ball']], create class instance per len()
    """
    class struct(object):
        x = 0
        y = 0
    coords = struct
    link = txfer.SerialTransfer(port)
    link.open()
    sleep(5)
    try: 
        while True:
            list_coords = [coords for i in range(len(data))]
            sendSize = 0
            for coords in list_coords:
                i = 0
                coords.x = data[i][0]
                coords.y = data[i][1]
                i += 1
                sendSize = link.tx_obj(coords.x, start_pos=sendSize)
                sendSize = link.tx_obj(coords.y, start_pos=sendSize)
                link.send(sendSize)

            if link.available():
                recSize = 0
                testStruct = struct
                testStruct.x = link.rx_obj(obj_type='f', start_pos=recSize)
                recSize += txfer.STRUCT_FORMAT_LENGTHS['f']
                
                testStruct.y = link.rx_obj(obj_type='f', start_pos=recSize)
                recSize += txfer.STRUCT_FORMAT_LENGTHS['f']
                
                print('{} : {}'.format(testStruct.x, testStruct.y))
                
            elif link.status <= 0:
                if link.status == txfer.CRC_ERROR:
                    print('ERROR: CRC_ERROR')
                elif link.status == txfer.PAYLOAD_ERROR:
                    print('ERROR: PAYLOAD_ERROR')
                elif link.status == txfer.STOP_BYTE_ERROR:
                    print('ERROR: STOP_BYTE_ERROR')
                else:
                    print('ERROR: {}'.format(link.status))
            link.close()
    except KeyboardInterrupt:
        link.close()

def write_array_to_arduino(array, port, baud_rate):
    serial_port = port  # Change this to match your Arduino's serial port
    baud_rate = baud_rate
    # # Initialize the serial connection
    ser = serial.Serial(serial_port, baud_rate)
    time.sleep(1)  # Wait for the Arduino to initialize
    data_bytes = bytes(array)
    ser.write(data_bytes)
    acknowledged = False
    timeout = time.time() + 5  # Wait for 5 seconds
    while time.time() < timeout:
        if ser.in_waiting > 0:
            response = ser.readline().strip().decode('utf-8')
            if response == 'ACK':
                acknowledged = True
                break
    return acknowledged

sendData = write_array_to_arduino([80,80], 'COM6', 115200)
print(sendData)
# send_data_to_arduino([[1, 3, 'person'], [3, 4, 'sports ball'], [5, 6, 'sports ball']], port = 'COM6')