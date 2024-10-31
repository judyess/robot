# robot
Unprofessional Details of this repo.

Important stuff I'm using:
6DOF (all revolute) robot arm w/ Arduino, PCA 9685, Adafruit PWM Servo Library

Important physical attributes of the stuff I'm using:
6 "high torque" servo motors. ~rated 6.8-8V, stall current 2.5-3A.
Motors are technically rated at 70-75Hz on their datasheets, but is set to 60Hz in code.
All motors connected to PCA 9685. Supplied with 7.5V. 

Useful things in the code right now:
    -PWM <-> Degree conversions (Main.ino)
    -servo motor speed controls (Main.ino)
    -checkPulse() function for manually testing PWM/Coords output at given angles. (Main.ino)
    -A bunch of scattered half-math functions. (function-tests.ino and prob everywhere else)



-function-tests.ino for testing output. No motor control functions.
-Main.ino has motor testing functions.
-sketch-X.ino was just a file to work out some code, pre-git. Useless.
-LinksModel.py computer model for testing, still in progress.