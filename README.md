# Wiper_Motor_Servo
converting a wiper motor into a strong servo motor

I had an old wiper motor made by Bosch. 

They are very powerful, they draw 2 to 3A, do not run very fast but have really a lot of torque, much more than what I would ever need to rotate a solar tracker (even a big size one !).

However these motors are DC ones without any solution to control their position. 

So I decided to add a cheap 12bits magnetic encoder AS5600, to add it to the motor shaft and to pilot all this with an ESP32.

See the project details on hackaday : https://hackaday.io/project/187675-converting-a-wiper-motor-into-a-strong-servo-motor

Version 1 of this firmware corespond to a "soft PID" tuning. https://github.com/f2knpw/Wiper_Motor_Servo/blob/master/ESP32_AS5600_DC_Motor_IBT-2.ino


Version 2 of this firmware gives a much stronger behavior and allows a very precise positionning : https://github.com/f2knpw/Wiper_Motor_Servo/blob/master/ESP32_AS5600_DC_Motor_IBT-2_strongPID.ino

