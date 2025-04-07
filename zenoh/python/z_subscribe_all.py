import zenoh, time
from dataclasses import dataclass
from pycdr2 import IdlStruct

@dataclass
class HelloWorldMessage(IdlStruct):
    data: str

def listener(sample):
    try:
        message = HelloWorldMessage.deserialize(sample.payload.to_bytes())
        print(f"Received {sample.kind} ('{sample.key_expr}': '{message.data}')")
    except Exception as e:
        print(f"Received {sample.kind} ('{sample.key_expr}': '{sample.payload}')")

if __name__ == "__main__":
    with zenoh.open(zenoh.Config()) as session:
        sub = session.declare_subscriber('**', listener)
        sub = session.declare_subscriber('**', listener)
        time.sleep(60)
