# Motor-PID-Controller-using-Arduino-Matlab
A simple motor PID setup using Arduino and Matlab

![alt text](https://github.com/jpodavalos/Motor-PID-Controller-using-Arduino-Matlab/raw/master/images/sample_gui1.png)

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

* Update P, I, D to get the best response.

NOTES:

* After clicking Send, the motor should start moving and System Response details should start to populate and update.

* To test if PID works, try stopping the Motor (be careful - use piece of cloth), it should fight back. Response depends on P, I, and D values.


To understand the PID algorith included in the Arduino code, I recommend watching this video: https://www.youtube.com/watch?v=sDd4VOpOnnA

