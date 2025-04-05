# Some required imports
import zenoh

# Initiate the zenoh-net API
config = zenoh.Config()
with zenoh.open(config) as session:
    # Declare the callback and the subscriber for Log messages with key 'rt/rosout'
    def rosout_callback(sample):
        print(f"PAYLOAD:{sample}")

    sub = session.declare_subscriber('rt/rosout', rosout_callback)

    session.put('rt/hello/message', "Hello World!")
