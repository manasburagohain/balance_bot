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
Resistors:  (10.0 R, 7.0 L)
Testing RIGHT FWD
[ Ticks : -6425,  Rot Speed (rad/s) : -41.2061]
Encoder Reads Right FWD

Testing LEFT FWD
[ Ticks : -6454,  Rot Speed (rad/s) : -41.3921]
Encoder Reads Left FWD

Testing LEFT BKWD
[ Ticks : 5974,  Rot Speed (rad/s) : 38.3136]
Encoder Reads Left BKWD

Testing RIGHT BKWD
[ Ticks : 6578,  Rot Speed (rad/s) : 42.1873]
Encoder Reads Right BKWD

Now Obtaining Motor Parametrs, make sure there is no load on their rotation ...
Obtaining RIGHT motor PARAMETERS :
Got time constant
[ No Load Speed (rad/s) : 45.5672, No Load Current (A) : 0.1398,  Stall Torque (N.m) : 1.2000 ]
[ Motor Constant K : 0.0114, Shaft Friction : 0.0007,  Shaft Inertia (Kg.m^2) : 4.4449e-05 ]

Obtaining LEFT Motor Parametrs :
Got time constant
[ No Load Speed (rad/s) : 45.2849, No Load Current (A) : 0.1187,  Stall Torque (N.m) : 1.7143 ]
[ Motor Constant K : 0.0121, Shaft Friction : 0.0006,  Shaft Inertia (Kg.m^2) : 3.3589e-05 ]


---------------------- Inertia measurements -----------------------------
L=1.7526 m 
Jzz = 0.004024119355058131 d=0.12508m T = 2.555s 2.578s 2.585s 
Jyy = 0.00859941300185732 d=0.12311mm T=3.823s 3.825s 3.815s
Jxx = 0.006006644959194755 d=0.06671mm T=5.955s 5.836s 5.889s
m0 = 1.0975kg