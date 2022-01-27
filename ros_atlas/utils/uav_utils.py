from djitellopy import Tello
from curtsies import Input

def connect_uav():
    print("\n################################################################################")
    print("Connecting to Tello UAV...")
    try:
        uav = Tello()
        uav.connect()
        print("UAV connected successfully!")
        print(f"Current battery percentage: {uav.get_battery()}")
        return uav
    except Exception as e:
        print("Failed to connect to Tello UAV, please try to reconnect")
        raise e

def autoflight(uav):
    uav.takeoff()
    uav.rotate_counter_clockwise(180)
    uav.move_forward(50)
    uav.rotate_clockwise(90)
    uav.move_forward(40)
    uav.rotate_clockwise(90)
    uav.move_forward(40)
    uav.rotate_counter_clockwise(270)
    uav.move_forward(50)
    uav.land()

def manual_control(uav):
    print("---------------------------------------\n From control thread.....")
    print("Listen Key: \n \
          t: takeoff \n \
          l: land \n \
          w a s d: move forward/left/back/right \n \
          q e: rotate \n \
          Arrow Up/Down: move up/down")
    
    with Input(keynames='curses') as input_generator:
        for key in input_generator:
            try:
                if key == 'w':
                    uav.move_forward(30)
                elif key == 'a':
                    uav.move_left(30)
                elif key == 's':
                    uav.move_back(30)
                elif key == 'd':
                    uav.move_right(30)
                elif key == 'e':
                    uav.rotate_clockwise(30)
                elif key == 'q':
                    uav.rotate_counter_clockwise(30)
                elif key == 'KEY_UP':
                    uav.move_up(30)
                elif key == 'KEY_DOWN':
                    uav.move_down(30)
                elif key == 't':
                    uav.takeoff()
                elif key == 'l':
                    uav.land()
                else:
                    print("Unknown key: {}".format(key))
            except:
                print("something wrong; key: " + key)
    # land after a few second ?
    uav.land()
