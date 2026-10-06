#!/usr/bin/env python3
"""Check RoboClaw PID configuration"""

from basicmicro import Basicmicro

PORT = "/dev/ttyTHS1"
BAUDRATE = 38400
ADDRESS = 0x80

with Basicmicro(PORT, BAUDRATE) as controller:
    print("Checking RoboClaw PID Configuration...")
    print(f"Address: 0x{ADDRESS:02X}\n")

    # Read M1 velocity PID
    m1_result = controller.ReadM1VelocityPID(ADDRESS)
    if m1_result[0]:
        p, i, d, qpps = m1_result[1:]
        print(f"M1 Velocity PID:")
        print(f"  P: {p}")
        print(f"  I: {i}")
        print(f"  D: {d}")
        print(f"  QPPS: {qpps}")
        if qpps == 0:
            print("  ⚠️  WARNING: QPPS is 0 - PID is NOT configured!")
    else:
        print("❌ Failed to read M1 PID")

    print()

    # Read M2 velocity PID
    m2_result = controller.ReadM2VelocityPID(ADDRESS)
    if m2_result[0]:
        p, i, d, qpps = m2_result[1:]
        print(f"M2 Velocity PID:")
        print(f"  P: {p}")
        print(f"  I: {i}")
        print(f"  D: {d}")
        print(f"  QPPS: {qpps}")
        if qpps == 0:
            print("  ⚠️  WARNING: QPPS is 0 - PID is NOT configured!")
    else:
        print("❌ Failed to read M2 PID")

    print("\n" + "="*50)
    if m1_result[0] and m2_result[0]:
        if m1_result[4] == 0 or m2_result[4] == 0:
            print("⚠️  PID NEEDS CONFIGURATION")
            print("Run 'python3 scripts/configure_pid.py' to set it up")
        else:
            print("✅ PID is configured and ready!")
    else:
        print("❌ Communication error - check connections")
