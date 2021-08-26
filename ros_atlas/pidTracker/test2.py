import sys
sys.path.append("../../")
from ros_atlas.pidTracker.SingletonDrone import _Drone

if __name__ == "__main__":
    b = _Drone()
    print(b)
    print(id(b))
    print(id(b.uav))
    while True:
        pass