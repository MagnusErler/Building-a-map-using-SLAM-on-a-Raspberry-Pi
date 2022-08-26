

#!/usr/bin/env python
'''
import struct

infile_path = "/dev/input/js0"
EVENT_SIZE = struct.calcsize("llHHI")
file = open(infile_path, "rb")
event = file.read(EVENT_SIZE)
while event:
    print(struct.unpack("llHHI", event))
    (tv_sec, tv_usec, type, code, value) = struct.unpack("llHHI", event)
    event = file.read(EVENT_SIZE)
'''

'''
from evdev import InputDevice
from select import select

gamepad = InputDevice('/dev/input/js0')

while True:
    r,w,x = select([gamepad], [], [])
    for event in gamepad.read():
        #print(event)

        if event.value != 0:
            if event.code == 304:
                print('A')
            elif event.code == 305:
                print('B')
            elif event.code == 307:
                print('X')
            elif event.code == 308:
                print('Y')

            elif event.code == 310:
                print('L1')
            elif event.code == 2:
                print('L2')
            elif event.code == 311:
                print('R1')
            elif event.code == 5:
                print('R2')

            elif event.code == 16 and event.value == -1:
                print('Arrow left')
            elif event.code == 16 and event.value == 1:
                print('Arrow right')
            elif event.code == 17 and event.value == -1:
                print('Arrow up')
            elif event.code == 17 and event.value == 1:
                print('Arrow down')

            elif event.code == 314:
                print('Select')
            elif event.code == 315:
                print('Start')
            elif event.code == 316:
                print('Mode')

            elif event.code == 0 and event.value > 1:
                print('L thumbstick right')
            elif event.code == 0 and event.value < 1:
                print('L thumbstick left')
            elif event.code == 1 and event.value < 1:
                print('L thumbstick up')
            elif event.code == 1 and event.value > 1:
                print('L thumbstick down')
            elif event.code == 317:
                print('L thumbstick press')

            elif event.code == 3 and event.value > 1:
                print('R thumbstick right')
            elif event.code == 3 and event.value < 1:
                print('R thumbstick left')
            elif event.code == 4 and event.value < 1:
                print('R thumbstick up')
            elif event.code == 4 and event.value > 1:
                print('R thumbstick down')
            elif event.code == 318:
                print('R thumbstick press')

'''



# Released by rdb under the Unlicense (unlicense.org)
# Based on information from:
# https://www.kernel.org/doc/Documentation/input/joystick-api.txt

import os, struct, array
from fcntl import ioctl

# Iterate over the joystick devices.
print('Available devices:')

for fn in os.listdir('/dev/input'):
    if fn.startswith('js'):
        print('  /dev/input/%s' % (fn))

# We'll store the states here.
axis_states = {}
button_states = {}

# These constants were borrowed from linux/input.h
axis_names = {
    0x00 : 'x',
    0x01 : 'y',
    0x02 : 'z',
    0x03 : 'rx',
    0x04 : 'ry',
    0x05 : 'rz',
    0x06 : 'throttle',
    0x07 : 'rudder',
    0x08 : 'wheel',
    0x09 : 'gas',
    0x0a : 'brake',
    0x10 : 'hat0x',
    0x11 : 'hat0y',
    0x12 : 'hat1x',
    0x13 : 'hat1y',
    0x14 : 'hat2x',
    0x15 : 'hat2y',
    0x16 : 'hat3x',
    0x17 : 'hat3y',
    0x18 : 'pressure',
    0x19 : 'distance',
    0x1a : 'tilt_x',
    0x1b : 'tilt_y',
    0x1c : 'tool_width',
    0x20 : 'volume',
    0x28 : 'misc',
}

button_names = {
    0x120 : 'trigger',
    0x121 : 'thumb',
    0x122 : 'thumb2',
    0x123 : 'top',
    0x124 : 'top2',
    0x125 : 'pinkie',
    0x126 : 'base',
    0x127 : 'base2',
    0x128 : 'base3',
    0x129 : 'base4',
    0x12a : 'base5',
    0x12b : 'base6',
    0x12f : 'dead',
    0x130 : 'a',
    0x131 : 'b',
    0x132 : 'c',
    0x133 : 'x',
    0x134 : 'y',
    0x135 : 'z',
    0x136 : 'tl',
    0x137 : 'tr',
    0x138 : 'tl2',
    0x139 : 'tr2',
    0x13a : 'select',
    0x13b : 'start',
    0x13c : 'mode',
    0x13d : 'thumbl',
    0x13e : 'thumbr',

    0x220 : 'dpad_up',
    0x221 : 'dpad_down',
    0x222 : 'dpad_left',
    0x223 : 'dpad_right',

    # XBox 360 controller uses these codes.
    0x2c0 : 'dpad_left',
    0x2c1 : 'dpad_right',
    0x2c2 : 'dpad_up',
    0x2c3 : 'dpad_down',
}

axis_map = []
button_map = []

# Open the joystick device.
fn = '/dev/input/js0'
print('Opening %s...' % fn)
jsdev = open(fn, 'rb')

# Get the device name.
#buf = bytearray(63)
buf = array.array('B', [0] * 64)
ioctl(jsdev, 0x80006a13 + (0x10000 * len(buf)), buf) # JSIOCGNAME(len)
js_name = buf.tobytes().rstrip(b'\x00').decode('utf-8')
print('Device name: %s' % js_name)

# Get number of axes and buttons.
buf = array.array('B', [0])
ioctl(jsdev, 0x80016a11, buf) # JSIOCGAXES
num_axes = buf[0]

buf = array.array('B', [0])
ioctl(jsdev, 0x80016a12, buf) # JSIOCGBUTTONS
num_buttons = buf[0]

# Get the axis map.
buf = array.array('B', [0] * 0x40)
ioctl(jsdev, 0x80406a32, buf) # JSIOCGAXMAP

for axis in buf[:num_axes]:
    axis_name = axis_names.get(axis, 'unknown(0x%02x)' % axis)
    axis_map.append(axis_name)
    axis_states[axis_name] = 0.0

# Get the button map.
buf = array.array('H', [0] * 200)
ioctl(jsdev, 0x80406a34, buf) # JSIOCGBTNMAP

for btn in buf[:num_buttons]:
    btn_name = button_names.get(btn, 'unknown(0x%03x)' % btn)
    button_map.append(btn_name)
    button_states[btn_name] = 0

print('%d axes found: %s' % (num_axes, ', '.join(axis_map)))
print('%d buttons found: %s' % (num_buttons, ', '.join(button_map)))

# Main event loop
while True:
    evbuf = jsdev.read(8)
    if evbuf:
        time, value, type, number = struct.unpack('IhBB', evbuf)

        if type & 0x80:
             print("(initial)", end="")

        if type & 0x01:
            button = button_map[number]
            if button:
                button_states[button] = value
                if value:
                    print("%s pressed" % (button))
                else:
                    print("%s released" % (button))

        if type & 0x02:
            axis = axis_map[number]
            if axis:
                fvalue = value / 32767.0
                axis_states[axis] = fvalue
                print("%s: %.3f" % (axis, fvalue))