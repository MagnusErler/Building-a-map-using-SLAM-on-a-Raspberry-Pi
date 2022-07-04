
from evdev import InputDevice
from select import select

gamepad = InputDevice('/dev/input/event15')

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

