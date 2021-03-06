# ME495 Robot Design Studio - Skateboard Team

This is a directory of the code that is used to program the Tiva microcontroller for the Skateboard Team of Robot Design Studio 2020.

Since each member of the team has different Makefiles, different folders contain code that only works on a specific computer. This is why each folder name contains a name of a team member.

## Folders
* BNO_drivers - Bosch Sensortec BNO055 sensor driver library
* IMU_sparkfun - Streams data from SparkFun IMU (LSM9DS1)
* PID_controller_on_tiva -
* accelerometer - Accelerometer data retrieval
* encoder - Communication between encoder and Tiva
* hello_world - Multiple hello_worlds created by different team members
* hello_world_linux - hello_world for linux
* simulation - MATLAB code that runs a simulations of the skateboard robot
* tiva_roboteq - tiva-roboteq interface
* RDS_Skateboard_Matlab - MATLAB code for pumping and idle
* (insert name here) - MATLAB code for dropping in
* (insert name here) - MATLAB code for manual

All the MATLAB codes for tricks have options for the three tricks but will only run the specified one correctly (correct gains, will plot for the specified trick)
