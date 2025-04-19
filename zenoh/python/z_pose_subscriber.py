from dataclasses import dataclass
from pycdr2 import IdlStruct
from time import sleep

from pycdr2.types import int8, int32, uint32, float64
import zenoh, time

@dataclass
class Quaternion(IdlStruct):
    x : float64
    y : float64
    z : float64
    w : float64

@dataclass
class Point(IdlStruct):
    x : float64
    y : float64
    z : float64

@dataclass
class Pose(IdlStruct):
    orientation: Quaternion
    position: Point

@dataclass
class Time(IdlStruct):
    sec: int32
    nsec: int32

@dataclass
class Header(IdlStruct):
    seq: int32
    stamp : Time
    frame_id : str

@dataclass
class StampedPose(IdlStruct):
    header : Header
    pose : Pose

@dataclass
class HelloMessage(IdlStruct):
    data : str

def listener(sample):
    print(f"Received {sample.kind} ('{sample.key_expr}': '{sample.payload}')")
    z_bytes = bytes(sample.payload.to_bytes())
    print(f"Received bytes\t{z_bytes}")
    pose = StampedPose.deserialize(z_bytes)
    #pose = StampedPose.deserialize(z_bytes)
    print(f"{pose=}")

if __name__ == "__main__":
    with zenoh.open(zenoh.Config()) as session:
        sub = session.declare_subscriber('imu/orientation', listener)
        time.sleep(60)
