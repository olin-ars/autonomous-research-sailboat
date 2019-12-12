'''
For manually changing the PWM values being sent to the sail and rudder, in order
to calibrate the sail and rudder controller.

Once determined, sail and rudder min and max values should be updated in
`motor_control.ino`, in `sail_rudder_transmitter.py`, and here at least in
the comments. This way, this code can be used for either re-calibration or for
testing with manual rudder/sail positioning.

Author: Jane Sieving
'''

import serial
ser = serial.Serial('/dev/ttyUSB0')
print(ser.name)

''' Leaving these values pretty unrestricted until this is calibrated.
May want to set them to the calibrated values if this is being used for
testing other than calibration. '''
SAIL_MAX = 360  # 100
SAIL_MIN = -1   # 20
RUD_MAX = 360   # 120
RUD_MIN = -1    # 60

rudder = 180
sail = 180

while(True):
    val = input(">")[0].lower()
    if val == 'a':
        if rudder + 10 > RUD_MAX:
            print("Maximum rudder value reached")
        else:
            rudder += 10
            print("Sail: %i\t\tRudder: %i" % (sail, rudder))
    elif val == 'd':
        if rudder - 10 < RUD_MIN:
            print("Minimum rudder value reached")
        else:
            rudder -= 10
            print("Sail: %i\t\tRudder: %i" % (sail, rudder))
    elif val == 'w':
        if sail + 10 > SAIL_MAX:
            print("Maximum sail value reached")
        else:
            sail += 10
            print("Sail: %i\t\tRudder: %i" % (sail, rudder))
    elif val == 's':
        if sail - 10 < SAIL_MIN:
            print("Minimum sail value reached")
        else:
            sail -= 10
            print("Sail: %i\t\tRudder: %i" % (sail, rudder))
    else:
        print("Invalid command")

    ser.write(b'(%i,%i)' % (sail, rudder))

ser.close()
