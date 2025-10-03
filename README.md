# Avionics-Mock-Sat-v1
First Repository For CSULB's SharkSat MockSat 

Disclaimer:
This is the first repository using github, through cubeide
ideally want to move a away from cube ide, and start using VS Code
to code and debug through there

This repository is practically temporary for now, until we find a permanent solution and learn how to setup.

also dont mind the name from ss_mock_v2 das what adam named it in cube ide

# todo list (in order of easiest to hardest)
- test temp sensor on board (have to double check if its connected to bus or not)
- test bn0055 (drivers already written)
- i2c mux TCA9548A driver [gotta double check da pinouts]
- W25Q128 [flash mem] 
- MCP3564RT Drivers and testing [thermocouple ADC] [spi]
- GPS Drivers and testing [PA1616D] 
- PID Control Loop - im doing this one nicholai

# whats finished
- sunsensor driver [i2c[
- bno055 drivers [uart]
- motor encoder drivers 
- motor drivers

# Avoinics Pinouts In IOC
pinouts based off altium schematic
- i2c1 - Temp Sensor - MCP9808
- i2c2 - IMU - BN055
- :)
