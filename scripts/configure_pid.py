#!/usr/bin/env python3
"""Configure RoboClaw velocity PID for speed control mode"""

from basicmicro import Basicmicro

PORT = "/dev/gerbil_roboclaw"
BAUDRATE = 38400
ADDRESS = 0x80

# ============================================================================
# PID PRESETS - Uncomment the one you want to try:
# ============================================================================
# VERY_GENTLE = {"P": 0.1, "I": 0.05, "D": 0.0}  # Very smooth, slow response
# GENTLE      = {"P": 0.2, "I": 0.1,  "D": 0.0}  # Smooth, good for testing
# MODERATE    = {"P": 0.5, "I": 0.2,  "D": 0.0}  # Balanced
# AGGRESSIVE  = {"P": 1.0, "I": 0.5,  "D": 0.0}  # Fast response (your current)
# VERY_AGGRESSIVE = {"P": 2.0, "I": 1.0, "D": 0.0}  # Very fast, may oscillate
# ============================================================================

# PID Configuration
# These are starting values - you may need to tune them for your specific motors
# P: Proportional gain - how aggressively it tries to reach target speed
# I: Integral gain - corrects steady-state errors over time
# D: Derivative gain - dampens oscillations (usually keep low or 0)
# QPPS: Quadrature Pulses Per Second - max encoder speed your motor can achieve

# goBILDA 5304-8002-0100 (99.5:1, 180 RPM @ 24 V, 2786.2 PPR at the output shaft):
#   180 RPM = 3 rev/s, so 3 x 2786.2 = ~8360 counts/sec at no load.
# Prefer the value Motion Studio reports at full duty over this estimate.

# GENTLE PID values for smooth, controlled operation
# Lower P = less aggressive response, smoother acceleration
# Lower I = gentler steady-state correction
P_VALUE = 0.2
I_VALUE = 0.1
D_VALUE = 0.0
QPPS_VALUE = 8400

print("RoboClaw Velocity PID Configuration")
print("="*50)
print(f"Address: 0x{ADDRESS:02X}")
print(f"P: {P_VALUE}")
print(f"I: {I_VALUE}")
print(f"D: {D_VALUE}")
print(f"QPPS: {QPPS_VALUE}")
print("="*50)

with Basicmicro(PORT, BAUDRATE) as controller:
    # Set M1 velocity PID
    print("\nSetting M1 velocity PID...")
    result = controller.SetM1VelocityPID(ADDRESS, P_VALUE, I_VALUE, D_VALUE, QPPS_VALUE)
    if result:
        print("✅ M1 PID configured successfully")
    else:
        print("❌ Failed to configure M1 PID")
        exit(1)

    # Set M2 velocity PID
    print("Setting M2 velocity PID...")
    result = controller.SetM2VelocityPID(ADDRESS, P_VALUE, I_VALUE, D_VALUE, QPPS_VALUE)
    if result:
        print("✅ M2 PID configured successfully")
    else:
        print("❌ Failed to configure M2 PID")
        exit(1)

    # Save to EEPROM so it persists across power cycles
    print("\nSaving configuration to EEPROM...")
    result = controller.WriteNVM(ADDRESS)
    if result:
        print("✅ Configuration saved to EEPROM")
    else:
        print("❌ Failed to save to EEPROM")
        exit(1)

    print("\n" + "="*50)
    print("✅ PID Configuration Complete!")
    print("="*50)
    print("\nYou can now test with ROS:")
    print("  ros2 launch gerbil_bringup gerbil.launch.xml use_mock_hardware:=false")
    print("\nTo verify, run:")
    print("  python3 scripts/check_pid.py")
