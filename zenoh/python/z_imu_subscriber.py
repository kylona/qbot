from dataclasses import dataclass
from pycdr2 import IdlStruct
from time import sleep

from pycdr2.types import int8, int32, uint32, float64
import zenoh, time
from typing import Tuple

@dataclass
class Quaternion(IdlStruct):
    x : float64
    y : float64
    z : float64
    w : float64

@dataclass
class ImuVector3(IdlStruct):
    x : float64
    y : float64
    z : float64


@dataclass
class Time(IdlStruct):
    sec: int32
    nsec: int32

@dataclass
class Header(IdlStruct):
    stamp : Time
    frame_id : str

@dataclass
class Imu(IdlStruct):
    header: Header
    orientation: Quaternion
    oc0 : float64
    oc1 : float64
    oc2 : float64
    oc3 : float64
    oc4 : float64
    oc4 : float64
    oc5 : float64
    oc7 : float64
    oc8 : float64
    angular_velocity: ImuVector3
    ac0 : float64
    ac1 : float64
    ac2 : float64
    ac3 : float64
    ac4 : float64
    ac4 : float64
    ac5 : float64
    ac7 : float64
    ac8 : float64
    linear_acceleration: ImuVector3
    lc0 : float64
    lc1 : float64
    lc2 : float64
    lc3 : float64
    lc4 : float64
    lc4 : float64
    lc5 : float64
    lc7 : float64
    lc8 : float64

def listener(sample):
    # print(f"Received {sample.kind} ('{sample.key_expr}': '{sample.payload}')")
    z_bytes = bytes(sample.payload.to_bytes())
    # print(f"Received bytes\t{z_bytes}")
    imu = Imu.deserialize(z_bytes)
    #pose = StampedPose.deserialize(z_bytes)
    print(f"{imu.linear_acceleration}")

if __name__ == "__main__":
    with zenoh.open(zenoh.Config()) as session:
        sub = session.declare_subscriber('qbot/imu', listener)
        time.sleep(60)
