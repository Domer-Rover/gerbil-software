from basicmicro import Basicmicro
import time

PORT = "/dev/ttyTHS1"
BAUDRATE = 38400
ADDRESS = 0x80

print(f"Opening {PORT} at {BAUDRATE}")
try:
    with Basicmicro(PORT, BAUDRATE) as controller:
        print("Connected. Reading encoders...")
        
        # Try reading version
        version = controller.ReadVersion(ADDRESS)
        if version[0]:
            print(f"Firmware Version: {version[1]}")
        
        # Try combined read if capable
        if hasattr(controller, 'ReadEncM1M2'):
            print("Testing ReadEncM1M2 (Cmd 78)...")
            try:
                encs = controller.ReadEncM1M2(ADDRESS)
                if encs[0]:
                    print(f"EncM1M2: {encs[1]} {encs[2]}")
                else:
                    print("Failed ReadEncM1M2 (returned success=False)")
            except Exception as e:
                print(f"Failed calling ReadEncM1M2: {e}")
        else:
            print("Basicmicro lib has no ReadEncM1M2 method?")

except Exception as e:
    print(f"Exception: {e}")
