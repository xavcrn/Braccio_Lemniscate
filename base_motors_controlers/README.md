The Arduino MKR1000 control the motors of the base of the robot.
It read the command sent by remote_server across UART and execute it.
A commadn is composed of 4 bytes.
The first couple of bytes is for the left motor and the second couple is for the  right motor.
The first byte of each couple is for the direction (forward if 0, backward else) and the second is for the speed (simply writen as an analog output).