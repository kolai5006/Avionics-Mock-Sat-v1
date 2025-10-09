# Avionics-Mock-Sat-v1
Repository For CSULB's SharkSat first MockSat 

Disclaimer:
dont mind the name from ss_mock_v2 das what adam named it in cube ide

# todo list (in order of easiest to hardest)
- test sun sensor on mock - (have to double check if its connected to i2c mux or not)
- test temp sensor on mock  - adrian
- test bn0055 on mock (drivers already written)
- i2c mux TCA9548A driver [gotta double check da pinouts]
- W25Q128 [flash mem] [spi]
- MCP3564RT Drivers and testing [thermocouple ADC] [spi]
- GPS Drivers and testing [PA1616D] 
- PID Control Loop - im doing this one nicholai

# whats finished
- sunsensor driver [i2c]
- bno055 drivers [uart]
- motor encoder drivers 
- motor drivers

# Avionics Pinouts In IOC
pinouts based off altium schematic
- i2c1 - Temp Sensor - MCP9808
- i2c2 - IMU - BN055
- :)
