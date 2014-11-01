#2150D_Skyrise
=============

##Main Class Files
-----------------
###2150CompControl.c
  
    Stores the basic competition outline that teams have to follow, not much interesting info here.
    We can call methods into this class to bring our competition controls into a centralized location.
  
##Header Files
----------------
###DriverControl.h
    This Stores all of the driver control code, as well as our PID library. 
    Found in V10, V11, and V12 this code is amazingly writen, and can be used on any 
    holonomic-type drivetrain.
    Next summer we will be releasing a software that allows for easy PID writing.

###LiftControl.h
    Our Lift Control Uses an array of sensors to get the most accurate readings for controling
    our lift. Again, fully packaged with our ground-breaking PID package. Soon, however we will 
    replace the PID package with a much higher-end PID autotuining software that Harrison has designed.

###IntakeControl.h

###menu.h

###auton.h

##PID Based Software
-------------------
###PID Autotuner
    This PID Autotuner relies on the ziegler-nichols method to tune the PID, while not variable like some other PID 
    controllers, this allows us to take pre-determined values such as Ku and Tu, which we have found for you, and apply
    them to the robot's PID loop, making highly-accurate tunings, which then can be later edited.
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
