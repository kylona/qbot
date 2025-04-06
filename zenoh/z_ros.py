# Some required imports
import zenoh
from dataclasses import dataclass
from pycdr2 import IdlStruct
from time import sleep

from pycdr2.types import int8, int32, uint32, float64

# Declare the types of Twist message to be encoded and published via zenoh
@dataclass
class Vector3(IdlStruct):
   x: float64
   y: float64
   z: float64

@dataclass
class Twist(IdlStruct):
   linear: Vector3
   angular: Vector3

# Declare the types of Log message to be decoded and subscribed to via zenoh
@dataclass
class Time(IdlStruct):
   sec: int32
   nanosec: uint32

@dataclass
class Log(IdlStruct):
   stamp: Time
   level: int8
   name: str
   msg: str
   file: str
   function: str
   line: uint32

# Initiate the zenoh-net API
with zenoh.open(zenoh.Config()) as session:
    # Declare the callback and the subscriber for Log messages with key 'rt/rosout'
    def rosout_callback(sample):
        log = Log.deserialize(sample.payload)
        print('[{}.{}] [{}]: {}'.format(
            log.stamp.sec, log.stamp.nanosec, log.name, log.msg))

    def generic_callback(sample):
        print(f"{sample=}")

    sub = session.declare_subscriber('rt/rosout', rosout_callback)
    sub = session.declare_subscriber('rt/rosout', rosout_callback)

    # Publish a Twist message with key 'rt/turtle1/cmd_vel' to make the turtlesim to move forward
    t = Twist(linear=Vector3(x=2.0, y=0.0, z=0.0),
              angular=Vector3(x=0.0, y=0.0, z=0.0)).serialize()
    session.declare_publisher("rt/turtle/cmd_vel")


    # Make it move forward until it hits the wall!!
    while True:
        session.put('rt/turtle1/cmd_vel', t)
        sleep(1.0)
        print("SENT COMMANDS:", t)
