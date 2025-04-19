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
    seq: int32
    stamp : Time
    frame_id : str

@dataclass
class Imu(IdlStruct):
    header: Header
    orientation: Quaternion
    angular_velocity: ImuVector3
    linear_acceleration: ImuVector3

@dataclass
class ImuPack(IdlStruct):
    size: uint32
    data: [Imu]

def listener(sample):
    print(f"Received {sample.kind} ('{sample.key_expr}': '{sample.payload}')")
    z_bytes = bytes(sample.payload.to_bytes())
    print(f"Received bytes\t{z_bytes}")
    pose = ImuPack.deserialize(z_bytes)
    #pose = StampedPose.deserialize(z_bytes)
    print(f"{pose=}")

if __name__ == "__main__":
    with zenoh.open(zenoh.Config()) as session:
        sub = session.declare_subscriber('qbot/imu_pack', listener)
        time.sleep(60)
