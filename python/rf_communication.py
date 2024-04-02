# import serial
# import time

# #we need a function to pass motor controls to the arduino where the scale is scaled to the distance of the pen
# # 640:480 is a 4/3 ratio
# # example output: go left

# def write_data_to_arduino(data, port, baudrate=115200, timeout=.1 ):
#     arduino = serial.Serial(port = port, baudrate=115200, timeout=.1)
#     time.sleep(0.05)
#     arduino.write(data)
#     return data

# data = [[100,500], [350, 500]]
# data1 = 5
# COM = 'COM5'
# write_data_to_arduino(data1, port = COM, baudrate=9600)

import time
import serial
from pySerialTransfer import pySerialTransfer as txfer

# Define the serial port (adjust this based on your system)
port = 'COM6'  # Change this to match your Arduino's serial port

# def write_data_to_ard(data):
#     ard = serial.Serial(port=port, baudrate=9600, timeout=0.1)
#     time.sleep(2)
#     ard.write(data.encode())
#     msg_return = ard.readline()
#     return msg_return


try:
    link = txfer.SerialTransfer(port)
    link.open()
    time.sleep(2)
    while True:
        send_size = 0
        list = [1, 3]
        list_size = link.tx_obj(list)
        send_size += list_size
        link.send(send_size)

        while not link.available():
                if link.status < 0:
                    if link.status == txfer.CRC_ERROR:
                        print('ERROR: CRC_ERROR')
                    elif link.status == txfer.PAYLOAD_ERROR:
                        print('ERROR: PAYLOAD_ERROR')
                    elif link.status == txfer.STOP_BYTE_ERROR:
                        print('ERROR: STOP_BYTE_ERROR')
                    else:
                        print('ERROR: {}'.format(link.status))
        rec_list  = link.rx_obj(obj_type=type(list),
                                     obj_byte_size=list_size,
                                     list_format='i')
        print('SENT: {}'.format(list))
        print('RCVD: {}'.format(rec_list))
except KeyboardInterrupt:
        try:
            link.close()
        except:
            pass
except:
    import traceback
    traceback.print_exc()
    
    try:
        link.close()
    except:
        pass