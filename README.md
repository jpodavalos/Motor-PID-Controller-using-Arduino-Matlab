# Motor-PID-Controller-using-Arduino-Matlab
A simple motor PID setup using Arduino and Matlab

Hardware Requirements:
* Arduino Uno
* Motor Dual H-Bridge L298 Driver https://circuit.rocks/motor-dualh-bridge-l298-driver
* Metal DC Geared Motor w/ Encoder https://www.dfrobot.com/wiki/index.php/12V_DC_Motor_251rpm_w/Encoder_(SKU:_FIT0186)

Software Requirements:
* Matlab R2016a +
* Arduino IDE


How to use:
* Edit COMPORT of Matlab code (PIDController.m) and run the GUI

handles.s = serial('COM5');

* Input P, I, D, and target speed (in RPM) values

* Click Send 

> After clicking Send, the motor should start moving and System Response details shoudld start to populate and update.

> To test if PID works, try stopping the Motor, it should fight back. Response depends on P, I, and D values.

To understand the PID algorith included in Arduino code, I recommend watching this video: https://www.youtube.com/watch?v=sDd4VOpOnnA
