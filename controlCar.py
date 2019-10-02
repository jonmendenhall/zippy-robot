import time
import serial
import glob
import struct


class CarController:
    vel_x, vel_y, vel_r = 0, 0, 0

    def __init__(self, port=None):
        self.tty = serial.Serial()
        self.tty.port = port
        self.tty.baudrate = 115200
    
    
    # write the binary packet to the serial port to be sent to the car
    def write(self):
        # motor mixing for mecanum wheels
        motor1 = -self.vel_y - self.vel_x + self.vel_r
        motor2 = -self.vel_y + self.vel_x - self.vel_r
        motor3 = -self.vel_y - self.vel_x - self.vel_r
        motor4 = -self.vel_y + self.vel_x + self.vel_r

        # limit the speeds of each motor to +/- 100%
        motor1 = int(round(min(max(motor1, -1), 1) * 720 * 20))
        motor2 = int(round(min(max(motor2, -1), 1) * 720 * 20))
        motor3 = int(round(min(max(motor3, -1), 1) * 720 * 20))
        motor4 = int(round(min(max(motor4, -1), 1) * 720 * 20))
        
        # create the binary packet with motor speeds
        packet = struct.pack('<hhhhB', 
            motor1,
            motor2,
            motor3,
            motor4,
            0
        )

        # write the packet to the serial port with a header and footer for error checking
        packet = bytes([0xff, 0x57]) + packet + bytes([0x99])
        self.tty.write(packet)


    # open the serial port for the transmitter
    def start(self):
        self.tty.open()


    # close the serial port for the transmitter
    def stop(self):
        self.tty.close()




# from math import *
# from boost_control import ControllerManager



# def main():

#     axes = [0, 0, -1, 0, 0, 0]
#     switches = [0, 0, 0, 0]
#     buttons = [0, 0]
#     allButtons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

#     remote = ControllerManager()
#     if not remote.init():
#         print('Remote not connected!')
#         return

#     car = CarController(port=glob.glob('/dev/tty.usbmodem*')[0])
#     car.start()
#     time.sleep(1)

#     # for i in range(4):
#     #   carControl.setLedOn(i, True)

#     try:
#         while True:
#             event, source, value = remote.getEvent()
#             if event == 1:
#                 axes[source] = (value / 27090) if (value > 0) else (value / 27349)
#             elif event == 2:
#                 allButtons[source] = value
#                 switches[0] = 1.0 - allButtons[0]
#                 switches[1] = 0.5 + (allButtons[1] - allButtons[2]) * 0.5
#                 switches[2] = 0.5 + (allButtons[3] - allButtons[4]) * 0.5
#                 switches[3] = 0.0 + allButtons[5]
#                 buttons[0] = 0.0 + allButtons[6]
#                 buttons[1] = 0.0 + allButtons[7]

#             car.vel_r = axes[3]
#             car.vel_x = -axes[0]
#             car.vel_y = -axes[1]
        
#             # for i in range(4):
#             #     car.leds[i] = switches[i] != 0
                        
#             car.write()
#             time.sleep(1/60)
#             # for i in range(4):
#             #   carControl.setLedBlink(i, True)
#             #   carControl.write()
#             #   time.sleep(1.5)
#             #   carControl.setLedBlink(i, False)
#     except KeyboardInterrupt:
#         pass

#     # for i in range(4):
#     #   carControl.setLedOn(i, False)
#     #   carControl.setLedBlink(i, False)

#     car.stop()


# if __name__ == '__main__':
#     main()

# # if manager.init():
# #   
# #   try:
# #       while True:
# #           event, source, value = manager.getEvent()
# #           if event == 1:
# #               axes[source] = (value / 27090) if (value > 0) else (value / 27349)
# #           elif event == 2:
# #               allButtons[source] = value
# #               switches[0] = 1.0 - allButtons[0]
# #               switches[1] = 0.5 + (allButtons[1] - allButtons[2]) * 0.5
# #               switches[2] = 0.5 + (allButtons[3] - allButtons[4]) * 0.5
# #               switches[3] = 0.0 + allButtons[5]
# #               buttons[0] = 0.0 + allButtons[6]
# #               buttons[1] = 0.0 + allButtons[7]

# #           # sendPacket(1410 + 500 * -axes[3], -axes[1] * 255, 0)      
# #           # time.sleep(1/60)  
# #           carControl.setSteering(axes[3])
# #           carControl.setSpeed(axes[1])
# #           carControl.write()
# #           time.sleep(1/60)
# #   except KeyboardInterrupt:
# #       pass





# # t0 = time.time()
# # try:
# #   while True:
# #       t = time.time() - t0
# #       
# # except KeyboardInterrupt:
# #   pass






