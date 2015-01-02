#2150DSkyrise
###2150DMain.c
    This is our team's main operational code. This allows our team to run the game's backend structure, with, or without the dedicated field control system. This truly is a great solution to controlling the robot's native OS. Secondly, our complete backend solution has been rewritten in C for optimal compatibility between code. We have also completely simplified the main file, allowing for some easy implementation of autonomous selection software.

##Header Files
----------------
###DriverControl.h
    This Stores all of the driver control code, as well as our PID library. Found in V10, V11, and V12 this code is amazingly writen, and can be used on any holonomic-type drivetrain. Next summer we will be releasing a software that allows for easy PID writing.

###auton.h

##PID Based Software
-------------------
###PID Autotuner
    This PID Autotuner relies on the ziegler-nichols method to tune the PID, while not variable like some other PID controllers, this allows us to take pre-determined values such as Ku and Tu, which we have found for you, and apply them to the robot's PID loop, making highly-accurate tunings, which then can be later edited.
    We follow the table:

    |                   Z-N Tuning Method               |  
    | Control Type 	| Kp     	| Ki       	| Kd     	|
    |--------------	|--------	|----------	|--------	|
    | P            	| 0.5Ku  	| -        	| -      	|
    | PI           	| 0.45Ku 	| 1.2Kp/Tu 	| -      	|
    | PD           	| 0.8Ku  	| -        	| KpTu/8 	|
    | PID           | 0.60Ku  	| 2Kp/Tu    | KpTu/8 	|
    | Passen Rule   | 0.7Ku  	| 0.4Kp/Tu  | 0.15KpTu 	|
    | Some Overshoot| 0.33Ku  	| 2Kp/Tu    | KpTu/3 	|
    | No Overshoot  | 0.2Ku  	| 2Kp/Tu    | KpTu/3 	|
##TODO
---------

The next things on our list to complete are:  
1. Add In Autonomii  
2. Finish Up PID Loop  
3. Revise LCD Menu  
  * Add Encoder Values / Lift Heights  
  * Rework the Battery Values for all 3 Batts  
  * Design "Fancier" UI (Not Much Can Be Done)  
  * Make Animations  
* Add Some Built-In Math Functions.
