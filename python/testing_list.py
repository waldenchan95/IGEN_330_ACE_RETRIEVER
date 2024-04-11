from time import sleep
from pySerialTransfer import pySerialTransfer as txfer

# class struct(object):
#     node1X = 10
#     node1Y = 10

#     node2X = 20
#     node2Y = 20 

#     node3X = 30
#     node3Y = 30

#     node4X = 40
#     node4Y = 40

#     node5X = 50
#     node5Y = 50

#     node6X = 60
#     node6Y = 60

#     node7X = 70
#     node7Y = 70

#     node8X = 80
#     node8Y = 80

class testing(object):
    node1X = 6
    node1Y = 10
    node2X = 8
    node2Y = 20 

# coords = struct


try:
    link = txfer.SerialTransfer('COM5')
    # letsTry = testing
    link.open()
    sleep(5)
    while True:
        sendSize = 0
        list = [10, 30, 50, 60, 40, 50, 60, 70, 10, 30, 50, 60, 40, 50, 60, 70]
        sendSize = link.tx_obj(list)
        link.send(sendSize)
        sleep(50)
        
        # if link.available():
        #     recSize = 0
        #     rec_list = link.rx_obj(obj_type=type(list), obj_byte_size=sendSize, list_format='i')
        #     print('{}'.format(rec_list))
            
        # elif link.status <= 0:
        #     if link.status == txfer.CRC_ERROR:
        #         print('ERROR: CRC_ERROR')
        #     elif link.status == txfer.PAYLOAD_ERROR:
        #         print('ERROR: PAYLOAD_ERROR')
        #     elif link.status == txfer.STOP_BYTE_ERROR:
        #         print('ERROR: STOP_BYTE_ERROR')
        #     else:
        #         print('ERROR: {}'.format(link.status))

except KeyboardInterrupt:
    link.close()
# coords = this

# list = [10, 30, 50, 60, 40, 50, 60, 70, 10, 30, 50, 60, 40, 50, 60, 70]
# for i in list:
#     coords.