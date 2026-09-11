#!/usr/bin/env python3
import os
import sys
import time

os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

received = []

def cb(msg: Image):
    received.append((msg.width, msg.height, msg.pixel_format_type, len(msg.data)))
    print(f"-> GOT FRAME: {msg.width}x{msg.height}, format={msg.pixel_format_type}, bytes={len(msg.data)}", flush=True)

def main():
    node = Node()
    topic = sys.argv[1] if len(sys.argv) > 1 else "/camera"
    print(f"Subscribing to {topic}...")
    node.subscribe(Image, topic, cb)
    
    t0 = time.time()
    while time.time() - t0 < 10.0:
        time.sleep(0.5)
        if received:
            print(f"SUCCESS: Total frames received: {len(received)}")
            return 0
    print(f"TIMEOUT: No frames received on {topic}")
    return 1

if __name__ == "__main__":
    sys.exit(main())
