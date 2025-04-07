# Some required imports
import zenoh
from pycdr import cdr
from pycdr.types import int8, int32, uint32, float64

# Declare the types of Twist message to be encoded and published via zenoh
@cdr
class Vector3:
   x: float64
   y: float64
   z: float64

@cdr
class Twist:
   linear: Vector3
   angular: Vector3

# Declare the types of Log message to be decoded and subscribed to via zenoh
@cdr
class Time:
   sec: int32
   nanosec: uint32

@cdr
class Log:
   stamp: Time
   level: int8
   name: str
   msg: str
   file: str
   function: str
   line: uint32

# Initiate the zenoh-net API
session = zenoh.open()

# Declare the callback and the subscriber for Log messages with key 'rt/rosout'
def rosout_callback(sample):
    log = Log.deserialize(sample.payload)
    print('[{}.{}] [{}]: {}'.format(
        log.stamp.sec, log.stamp.nanosec, log.name, log.msg))

sub = session.declare_subscriber('rt/rosout', rosout_callback, reliability=zenoh.Reliability.RELIABLE())

# Publish a Twist message with key 'rt/turtle1/cmd_vel' to make the turtlesim to move forward
t = Twist(linear=Vector3(x=2.0, y=0.0, z=0.0),
          angular=Vector3(x=0.0, y=0.0, z=0.0)).serialize()
session.put('rt/turtle1/cmd_vel', t)

# Make it move forward until it hits the wall!!
session.put('rt/turtle1/cmd_vel', t)
session.put('rt/turtle1/cmd_vel', t)
