/*******************************************************************************
*                           Balancebot Template Code
*                           pgaskell@umich.edu
*       
*******************************************************************************/

bin/			             : Binaries folder
balanceebot/balanceebot.c/.h : Main setup and threads
test_motors/test_motors.c/.h : Program to test motor implementation
common/mb_controller.c/.h    : Contoller for manual and autonomous nav
common/mb_defs.h             : Define hardware config
common/mb_motors.c/.h        : Motor functions to be used by balancebot
common/mb_odometry.c/.h	     : Odometry functions
optitrack/		     : optitrack driver/server
xbee_serial		     : xbee serial optitrack client code



-----------------------motor params-------------------------------------------
Resistors:  (5.0 R, 5.7 L)

Diagonising Motors Operation Capability ...

Testing RIGHT FWD
[ Ticks : -5755,  Rot Speed (rad/s) : -36.9091]
Encoder Reads Right FWD

Testing LEFT FWD
[ Ticks : -5883,  Rot Speed (rad/s) : -37.7300]
Encoder Reads Left FWD

Testing LEFT BKWD
[ Ticks : 5431,  Rot Speed (rad/s) : 34.8312]
Encoder Reads Left BKWD

Testing RIGHT BKWD
[ Ticks : 5828,  Rot Speed (rad/s) : 37.3773]
Encoder Reads Right BKWD

Now Obtaining Motor Parametrs, make sure there is no load on their rotation ...
Obtaining RIGHT motor PARAMETERS :
Got time constant
[ No Load Speed (rad/s) : 40.3351, No Load Current (A) : 0.1169,  Stall Torque (N.m) : 2.4000 ]
[ Motor Constant K : 0.0139, Shaft Friction : 0.0008,  Shaft Inertia (Kg.m^2) : 5.9891e-05 ]

RIGHT Motor parameters are within the acceptable range

Obtaining LEFT Motor Parametrs :
Got time constant
[ No Load Speed (rad/s) : 40.6071, No Load Current (A) : 0.1002,  Stall Torque (N.m) : 2.1053 ]
[ Motor Constant K : 0.0138, Shaft Friction : 0.0007,  Shaft Inertia (Kg.m^2) : 4.3361e-05 ]

LEFT motor parameters are within the acceptable range

Motors passed all tests. You can proceed to controls. 


---------------------------------------------------
Inertia measurements
L=1752.6mm 
Jzz: d=125.08mm T = 2.555s 2.578s 2.585s
Jyy: d=123.11mm T=3.823s 3.825s 3.815s
Jxx: d=66.71mm T=5.955s 5.836s 5.889s
m0 = 1097.5g