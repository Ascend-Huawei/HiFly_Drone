import sys
from typing import final

sys.path.append("../../")
from ros_atlas.utils.uav_utils import connect_uav

@final
class Singleton:
    _instance = None
    # _inited = False

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance # address of class instance

    # def __init__(self) -> None:
    #     print(type(self)._inited)
    #     if type(self)._inited == True:
    #         return
    #     # self.uav = connect_uav()
    #     self.uav = 19
    #     type(self)._inited = True
