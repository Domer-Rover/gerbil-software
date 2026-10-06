from basicmicro import Basicmicro

PORT = "/dev/ttyTHS1"  # Update as needed for your system
BAUDRATE = 38400

# The context manager automatically closes the connection when done
with Basicmicro(PORT, BAUDRATE) as controller: 
    address = 0x80
    
    # Read battery voltages
    main_batt = controller.ReadMainBatteryVoltage(address)
    logic_batt = controller.ReadLogicBatteryVoltage(address)
    
    if main_batt[0] and logic_batt[0]:
        print(f"Main battery: {main_batt[1]/10.0}V")
        print(f"Logic battery: {logic_batt[1]/10.0}V")
    
    # Read temperatures
    temp = controller.ReadTemp(address)
    if temp[0]:
        print(f"Temperature: {temp[1]/10.0}°C")

    print("Roboclaw test complete.")