// smart_PTC_monitor_template.beta.8.27.2012.c   Rev 0

// This program is an open source beta version of a competition template for Sack Attack.  It adds a task
// called smart_PTC_monitor which uses current and PTC temperature models to manage a current_limiter function
// which can keep hardware PTC fuses in the motors,cortex and power expander from ever tripping.

// Program Author: Chris Siegert aka Vamfun on the Vex forums.
// see my blog: vamfun.wordpress.com  for model details and vex forum threads on subject
// email: vamfun@yahoo.com
// Mentor for team 599 Robodox and team 1508 Lancer Bots

// Teams are authorized to freely use this program , however, it is requested that improvements or additions be
// shared with the Vex community via the vex forum.  Please acknowledge my work when appropriate.  Thanks

// Scope and limitations:
// Program assumes that servo currents are zero.  If servo position is known, then currents can be modeled.
// Testing has been limited to 393 motors and 3wire motors, however, the program handles  269 motors.
// Periodic updates to the models will be made and posted on the Vex forum as more testing is done.
// Program handles the Power Expander PTC when the ports are entered with provided #defines
// All other motor ports are automatically configured to calculate the correct currents for each PTC

// Basic Instructions:
// Write your program as you normally would.  When done, put your autonomous code in place of my_autonomous()
// and your user code in place of my_user().  Then do a find and replace to change all the "motor" functions to
// "motor_input" i.e.  motor[Left_motor] --> motor_input[Left_motor].

// Put all your encoder speed calculations into  void compute_speeds() function and assign speeds to M[port].speed variable.
// You can read these speeds from the global array whenever you need them.  This loop runs at about 100ms update rate.

// Use a #define or switch to create a
// PTC_monitor_engage boolean and set true to engage the monitor.

// Tell the program which ports are used by the Power Expander as shown in the example below:
// #define PE_CH1  UNUSED // if used put port number .... i.e. PE_CH1  port1  else put the #define UNUSED
// #define PE_CH2  Right_drive_motor  //use setup names
// #define PE_CH3  port3  //use port name
// #define PE_CH4  1  // use integer = to port number -1.  This is motor port2

// Initialize the PTC temperatures using these #defines:
// #define T_0_DEG_F  (72.) //Set robot ambient temperature here.  If you are in a hot gym...this might be 85 deg
// #define T_M  (100.) //100 deg C =Setpoint temperature for the PTC current monitor.

// Motor currents and PTC temperature data are held in a global state matrix M[] which is a structured array that contains
// the motor PTC/current data in the first 10 elements (index 0 to 9) and the non motor PTC's data in the next three elements.
// M[10] , M[11] and M[12] pertain to the cortex1_5,cortex6_10 and power expander PTC states respectively.

// Program uses two update rates: The current_limiter updates at PTC_TASK_DELAY_MS which is 15ms
// and the temperature calculations are set to run at about 6 times {PTC_TASK_DELAY_MS + subtasks delays) ~= 100ms.
//****************************************************************************************************************

#define UNUSED 99  //Just a high number above motor port 10.
#define PTC_TASK_DELAY_MS 10 // Update rate of the smart_PTC_monitor task.  Nominal 10ms
#define T_0_DEG_F  (72.) //Set robot ambient temperature here.  If you are in a hot gym...this might be 85 deg
#define T_M  (100.) //100 deg C =Setpoint temperature for the PTC current monitor
#define MOTOR_SLEW_RATE 20 //Maximum change in motor command per PTC monitor update.  Msec to max command = PTC_TASK_DELAT_MS*127/MOTOR_SLEW_RATE

// Tell the software which ports are used by the power expander
#define PE_CH1  UNUSED  // if used put port number .... i.e. PE_CH1  port1  else put the #define UNUSED
#define PE_CH2  UNUSED
#define PE_CH3  UNUSED
#define PE_CH4  UNUSED

#define R_SYS           0.0 // Series resistance of a motor model.  Typically just ignore because it is small w/r to motor resistance
#define PWM_FREQ        1150 //1150  Frequency of the motor controller PWM waveform in HZ
#define V_DIODE         0.75 //.75   Voltage of the catch diodes in the H_bridge controller

#define I_FREE_269      0.18 //.18  Amps
#define I_STALL_269     2.88 //2.6  Amps
#define RPM_FREE_269    110 //100   RPM
#define R_269           (7.2/I_STALL_269) //Ohms
#define L_269           0.000650 //.00065  Henrys
#define Ke_269          (7.2*(1-I_FREE_269/I_STALL_269)/RPM_FREE_269)  //volts/RPM

#define I_FREE_393      0.37 //Amps
#define I_STALL_393     4.8 //Amps
#define RPM_FREE_393    100 //RPM
#define R_393           (7.2/I_STALL_393)
#define L_393           0.000650 //CDS??
#define Ke_393          (7.2*(1-I_FREE_393/I_STALL_393)/RPM_FREE_393)

#define I_FREE_3WIRE      0.15 //CDS??
#define I_STALL_3WIRE     1.8  //CDS??
#define RPM_FREE_3WIRE    100
#define R_3WIRE           (7.2/I_STALL_3WIRE)
#define L_3WIRE           0.000650 //??CDS just a guess... assumed the same as 269 for now
#define Ke_3WIRE          (7.2*(1-I_FREE_3WIRE/I_STALL_3WIRE)/RPM_FREE_3WIRE)

//Safe currents are assumed to be the spec hold currents for the PTC's unless testing shows otherwise.
#define SAFE_CURRENT_269 .75 //.75 @7.2 volts
#define SAFE_CURRENT_393 .9 //.9
#define SAFE_CURRENT_3WIRE .75 //.75
#define SAFE_CURRENT_SERVO .75 //.75
#define SAFE_CURRENT_CORTEX 3.//3. Spec says 4 amps but Jpearman testing shows 3
#define SAFE_CURRENT_PE 3. //3  Spec says 4 amps but Jpearman testing shows 3
//PTC Constant calculation******************************************
//R_0  No power PTC resistance at 25C still air
//T_c Trip temperature degC, nom = 100C
//T_ref = 25.
//I_hold PTC hold current
//tau is computed by putting a I_trip = 5*I_hold or something close.  The time to trip T_trip is
// measured at this current and used to compute tau.
// tau = factor*T_trip*(I_trip/I_hold)^2   I_trip typically = 5*I_hold, factor = .5 to 1.
// K = I_hold*I_hold*R_0/(T_c -T_0); //PTC dissipation constant
// c_1 =(T_c - T_ref)/I_hold/I_hold ;   // c1 = R_0/K = (T_c - T_ref)/I_hold/I_hold
// c_2 = 1./(tau*1000.);
// T = T + c2*(I*I*c1 -T +T0)*dt_ms  //differential equation for T

// PTC HR16-400 used in cortex and power expander
// R_0_cortex = .018 ;
// T_c_cortex = 100;
// I_HOLD_cortex = 3.;
// t_trip = 1.7 @ I_trip = 5*I_hold
// TAU_cortex = .5*25*1.7= 21.25 sec;  factor = .5
// c_1_cortex  75/9 = 8.3333
// c_2_cortex  1/21250= 4.7e-5

// PTC HR30-090 used in 393
// R_0_393 = .14 ;
// T_c_393 = 100;
// I_hold_393 = .9;
// t_trip = 7.1 @ I_trip = 5*I_hold
// tau_393 = .5*25*7.1= 88.75;   factor = .5
// c_1_393  75/.81= 95.59
// c_2_393  1/88750= 1.1267e-5

// PTC HR16-075 used in 269
// R_0_269 = .11 ohm min   .2 ohm max ;
// T_c_269 = 100;
// I_hold_269 = .75;
// t_trip = 2 sec @ I_trip = 5*I_hold
// tau_269 = .5*(I_trip/I_hold)^2*2 sec= 12.5*2 = 25 sec;  factor = .5
// c_1_269  75/(.75^2)= 133.33
// c_2_269  1/25000= 4.e-5



// PTC MINISMDC-075F used in three wire
// R_0_3wire = .11 min; max = .45
// T_c_3wire = 100; //assume same as 269
// I_hold_3wire = .75;
// I_trip_3wire= 1.5 amp;
// t_trip_3wire = .2 sec @8 amps
// tau_3wire = (8amp/.75amp)^2*t_trip =22.8 sec;   factor = 1.
// c_1_3wire  75/(.75^2)= 133.33
// c_2_3wire  1/22800.= .000043945

#define c_1_cortex  (8.3333)
#define c_2_cortex  (.000047)
#define c_1_393  (95.59)
#define c_2_393  (.0000113)
#define c_1_269  (133.33)
#define c_2_269  (.00004)
#define c_1_3wire  (133.33)
#define c_2_3wire  (.000044)

//End PTC calculations ***************************************************************

typedef enum{
  none, //=0
  cortex1_5=100,
  cortex6_10=200,
  pwr_exp=300,
  motorservo = tmotorServoStandard ,//=3
  motor3wire =tmotorServoContinuousRotation, //=4

  motor269 =tmotorVex269,//=8
  motor393 =tmotorVex393,//=9
  motor393highspeed =tmotorVex393HighSpeed  //=10

}object_type;

typedef struct // Object State
{
  object_type obj;
  int PE_port; // = 1 if power expander is connected to the port , else 0.
  int cmd_out; // = motor_input[port] unless current limited
  int cmd_out_last ; // last cmd_out used for slew rate limiting
  float speed;//rpm  positive rpm for positive motor command
  float I ; // object current
  //PTC data
  float I_safe; // motor safe current...typically = I_hold of motor PTC
  float I_safe_alloc; //current allocation to keep a high level PTC (cortex or PE)  under I_safe.
  float cum_I_safe;  //this is the sum of all motor I_safe feeding a cortex1_5, cortex6_10 or PE group.

  float T; //PTC temperature deg C
  float T_m;//Temperature(degC) threshold for the PTC trip logic
  bool   PTC_triped;  // true if T>T_m , reset to false if T< .9*T_m



} object_state;

//PTC monitor function prototypes
void PTC_monitor_threshold_check();
void current_limiter();
int command_calc(int i, float I_safe , float v_battery = 7.2);
float CalcCurrent269Vamfun( tMotor m, int cmd, int rpm, float v_battery  );
float CalcCurrent393Vamfun( tMotor m, int cmd, int rpm, float v_battery  );
float CalcCurrent393HighSpeedVamfun( tMotor m, int cmd, int rpm, float v_battery  );
float CalcCurrent3WIREVamfun( tMotor m, int cmd, int rpm, float v_battery  );
void initialize_object_state();
void clear_motor_input_cmds();
float temp_rate_PTC(int j);
void update_object_state();
void setup_port_current_parameters( );

//PTC Monitor Global variables
float Time_sec = 0;
float v_battery=7.2;
int slew = MOTOR_SLEW_RATE ;   // maximum change in motor command with each iteration.  127/slew*dt_ms = time to max command
int j = 0;

bool  PTC_monitor_engage=true;
bool switch_state_last = false; //used in toggle function of monitor engage logic
bool switch_state = false;
object_state M[13];  // M[0] to M[9] are motors, M[10] to M[12] are Cortex1_5,cortex6_10 and PE PTCs respectively.
int  motor_input[10];  //Motor input array... user motor commands are give to this array instead of motor[port] array



float T_0 = (T_0_DEG_F - 32.)/1.8; //Deg F to Deg C
float dt_ms = 0; // measured periodic update rate used in smart_PTC_monitor task

//////////////////////////////////////////////////////////////////////////////////////////
//// Insert user prototype and global functions here
//void my_autonomous();
//void my_user();
//int turn = 0;
//int fwd = 0;
//int lift = 0;

///////////////////////////////////////////////////////////////////////////////////////////


task smart_PTC_monitor()
{
  /*-----------------------------------------------------------------------------*/
  /*                                                                             */
  /*  task smart_PTC_monitor()                                                         */
  /*                                                                             */
  /*  task written by Vamfun...Mentor Vex 1508, 599.                             */
  /*  8.21.2012  vamfun@yahoo.com... blog info  http://vamfun.wordpress.com      */
  /*                                                                             */
  /*  Sets up periodic loop to calculate object states used in PTC monitor.      */
  /*  States are passed to a current limiter that incorportes user logic to      */
  /*  determine what motors should be limited when a PTC monitor trips.          */
  /*  The current_limiter limits motor commands to keep currents below PTC safe  */
  /*  limits.  It also slew rate limits the motor inputs.   The current_limiter  */
  /*  loop runs at PTC_TASK_DELAY_MS update rate.  The current and temperatures  */
  /*  are updated 6 times slower.                                                */
  /*                                                                             */
  /*-----------------------------------------------------------------------------*/
  //Monitor code
  int loop_cnt = 0;
  bStopTasksBetweenModes = false;  //This keeps monitor running between  autonomous and user tasks to keep temperature
  // models working
  // Initialize
  ClearTimer(T1);//T1 is reserved for state update timer

  initialize_object_state();

  while(true)
  {
    //we need this because we have are no longer stopping all tasks between modes
    //there might be a better way to do this without stopping tasks every cycle.  CDS..action
    if(bIfiRobotDisabled || !bIfiAutonomousMode)
    {
      StopTask(autonomous);

    }
    if(bIfiRobotDisabled || bIfiAutonomousMode)
    {
      StopTask(usercontrol);

    }

    Time_sec = (float)time100[T2]/10.;  //Run time in sec
    v_battery = nImmediateBatteryLevel/1000.;
    //Create logic to toggle the engage logic based upon push of switch_PTC
    switch_state_last = switch_state;
    switch_state = (bool)SensorValue(switch_PTC);
    if( switch_state && !switch_state_last )  //toggle PTC_monitor_engage and debounce switch
    {
      PTC_monitor_engage =  !PTC_monitor_engage;
    }

    if(loop_cnt > 5)  //Run the current computations and temperature updates at  90ms.  Adjust PTC_TASK_DELAY_MS
      //until dt_ms reads 90ms.   This will ensure that the current limiter runs at about
    //6 times faster or about 15ms.

    {
      PTC_monitor_threshold_check(); //if T> Tm then set trip = 1, else if T< .9*Tm reset trip.
      dt_ms = time1[T1] ;// update time msec

      update_object_state(); //compute currents and use these to update PTC temperatures

      ClearTimer(T1);//reset loop timer
      loop_cnt = 1;
    }
    else
    {
      loop_cnt ++;
    }

    current_limiter(); // this is where M[port].cmd_out -> current limit-> motor[port]

    wait1Msec(PTC_TASK_DELAY_MS);

  } //end while

}  //end smart_PTC_monitor() task

void compute_speeds()
{
  //This routine is called every dt_ms by smart_PTC_monitor() task.  The user computes the speeds and assigns
  //them to the proper motors.
  //read encoders
  int left_count ;
  int right_count ;
  int lift_count;
  static int left_count_last ;
  static int right_count_last ;
  static int lift_count_last ;


  left_count = SensorValue[Encoder_left];
  right_count = SensorValue[Encoder_right];
  lift_count = SensorValue[Encoder_lift];

  // load speeds to M array
  M[Left_motor].speed = (left_count - left_count_last)*166.67/dt_ms;  //speed = delta_cnts/360*60*1000/dt_ms
  M[Right_motor].speed = (right_count - right_count_last)*166.67/dt_ms;
  M[Lift_motor_3wire].speed = (lift_count - lift_count_last)*166.67/dt_ms;

  left_count_last = left_count;
  right_count_last = right_count;
  lift_count_last = lift_count;
}
void current_limiter()
{
  /*-----------------------------------------------------------------------------*/
  /*                                                                             */
  /*  void current_limiter()                                           */
  /*                                                                             */
  /*  subroutine written by Vamfun...Mentor Vex 1508, 599.                       */
  /*  8.21.2012  vamfun@yahoo.com... blog info  http://vamfun.wordpress.com      */
  /*  Routine used in support of smart_PTC_monitor.c template                    */
  /*  This function is where the user places an action script that changes       */
  /*  the motor commands to limit the currents                                   */
  /*                                                                             */
  /*                                                                             */
  /*-----------------------------------------------------------------------------*/
  //

  //Default logic scheme
  // This scheme ignores functional motor cmd groups.  Typically, one would reduce all drive or lift
  // commands when any PTC monitor trips from that functional group.  This keeps the robot more balanced and is recommended.
  // Without functional grouping information, the best one can do is keep the tripped monitor at a safe temperature by reducing the
  // motor commands feeding the monitor. I.E. if a cortex1_5 monitor trips, all motors feeding it are reduced to safe commands.
  // So...
  // if any motor, cortex or PE PTC monitor trips, then set the cmd_out = cmd_safe for all the motors feeding it.
  // cmd_safe is computed from I_safe or I_safe_allocated for each motor using an approximate current formula
  // cmd_safe = 127.*(sgn(motor_input)*(I_s*R_motor+ V_DIODE)  + speed*Ke )/(v_battery + V_DIODE );
  // where I_s = I_safe or I_safe_allocated depending on which PTC object is tripped.  If an individual motor trips
  // then I_s =I_safe of the motor.  If a higher level cortex or PE PTC monitor trips then I_s = I_safe_allocated. which is a weighted
  // allocation of the high level PTC I_safe to the motors feeding it.  If at any time , the abs(safe_cmd)> abs(motor_input[port])
  // the cmd_out = motor_input[port] since a limit is no longer needed.

  int cmd_safe_trial = 0;
  if(PTC_monitor_engage)
  {
    for( j = 0; j<13;j++)
    {
      if(M[j].obj == 0) continue; // if there is no object , then skip to the next object
        if(j<10)
      {
        // Any single motor PTC tripped , if so reduce command to create i_safe levels.
        if( M[j].PTC_triped == true )
        {
          cmd_safe_trial =command_calc( j, M[j].I_safe, v_battery); // use this motor's safe current to compute cmd.

          if(abs(cmd_safe_trial)< abs(motor_input[j]))//is the input cmd greater than the safe cmd then
          {
            M[j].cmd_out = cmd_safe_trial;    // use the safe cmd
          }
          else
          {
            M[j].cmd_out = motor_input[j];// else use the input cmd.
          }
        }
        else
        {
          M[j].cmd_out = motor_input[j];// no monitors tripped pass the user input to the motors
        }
      }//end j<10

      else  //now j = 10,11,12 which are the cortex and PE monitors.
      {
        //Cortex or PE Monitor tripped??? If so overwrite safe_cmds from single motors if there is common motor to both

        if(M[j].PTC_triped == true )  // any cortex or PE monitor tripped?
        {
          switch(M[j].obj)
          {
          case cortex1_5:
            for (int i = 0; i<5; i ++)
            {
              if(M[i].PE_port == 0) //exclude PE port
              {

                // reduce the commands to the motors feeding this object.  Use I.safe_alloc current if < I.safe for motor
                if(M[i].I_safe_alloc < M[i].I_safe && M[i].PTC_triped || !M[i].PTC_triped)
                {

                  cmd_safe_trial =command_calc(i, M[i].I_safe_alloc, v_battery); // use allocated safe current to compute cmd.

                  if(abs(cmd_safe_trial)< abs(motor_input[i]))//is the input cmd greater than the safe cmd then
                  {
                    M[i].cmd_out = cmd_safe_trial;    // use the  trial safe cmd
                  }
                  else
                  {
                    M[i].cmd_out = motor_input[i];// else use the input cmd.
                  }

                }
              }// exclude PE ports
            }
            break;
          case cortex6_10:
            for (int i = 5; i<10; i ++)
            {
              if(M[i].PE_port == 0)
              {
                if(M[i].I_safe_alloc < M[i].I_safe && M[i].PTC_triped || !M[i].PTC_triped)
                {
                  cmd_safe_trial =command_calc(i, M[i].I_safe_alloc, v_battery); // use allocated safe current to compute cmd.

                  if(abs(cmd_safe_trial)< abs(motor_input[i]))//is the input cmd greater than the safe cmd then
                  {
                    M[i].cmd_out = cmd_safe_trial;    // use the safe cmd
                  }
                  else
                  {
                    M[i].cmd_out = motor_input[i];// else use the input cmd.
                  }
                }
              }
            }
            break;
          case pwr_exp:
            for (int i = 0; i<10; i ++)
            {
              if(M[i].PE_port == 1)

              {

                if(M[i].I_safe_alloc < M[i].I_safe && M[i].PTC_triped || !M[i].PTC_triped)
                {
                  cmd_safe_trial =command_calc(  i, M[i].I_safe , v_battery ); // use safe current to compute cmd.

                  if(abs(cmd_safe_trial)< abs(motor_input[i]))//is the input cmd greater than the safe cmd then
                  {
                    M[i].cmd_out = cmd_safe_trial;    // use the safe cmd
                  }
                  else
                  {
                    M[i].cmd_out = motor_input[i];// else use the input cmd.
                  }
                }
              }
            }
            break;
          }//end case


        }// end IF
      } //end if(j<10) else
    }//end j for loop
  }//end if (PTC_monitor_engage)
  else
  {
    for(j = 0 ; j< 10; j++)
    {
      M[j].cmd_out = motor_input[j];  //not engaged so set  outputs equal to inputs for each motor port

    }
  }//end if (PTC_monitor_engage) else
  //write object motor cmds to the motors
  //limit the maximum command per update cycle to slew.   This creates a command rate limit or slew rate limit.
  for( j = 0; j<10; j++ )
  {
    if(M[j].cmd_out > (M[j].cmd_out_last + slew))
    {
      motor[(tMotor)j] = M[j].cmd_out_last + slew;
      M[j].cmd_out_last  = M[j].cmd_out_last + slew;
    }
    else if (M[j].cmd_out < (M[j].cmd_out_last - slew))
    {
      motor[(tMotor)j] = M[j].cmd_out_last - slew;
      M[j].cmd_out_last  = M[j].cmd_out_last - slew;
    }
    else
    {
      motor[(tMotor)j] = M[j].cmd_out ;
      M[j].cmd_out_last  = M[j].cmd_out ;
    }
  }

}//end PTC_current_limiter_logic


float temp_rate_PTC(int j)
{

  //This function returns the temperature rate as a function of last T and I
  switch(M[j].obj)
  {

  case motor269:
    // PTC HR16-075 used in 269
    return c_2_269*(M[j].I*M[j].I*c_1_269 -(M[j].T - T_0)); // temperature rate
    break;
    // PTC HR30-090 used in 393
  case motor393:
  case motor393highspeed:
    return c_2_393*(M[j].I*M[j].I*c_1_393 -(M[j].T - T_0)); // temperature rate
    break;
  case motor3wire:
    // PTC  used in 3 wire
    return c_2_3wire*(M[j].I*M[j].I*c_1_3wire -(M[j].T - T_0)); // temperature rate
    break;
  case cortex1_5:
  case cortex6_10:
  case pwr_exp:
    // PTC HR16-400 used in cortex and power expander
    return c_2_cortex*(M[j].I*M[j].I*c_1_cortex -(M[j].T - T_0)); // temperature rate
    break;
  default:
    //servo
    return 0;//this is a potential problem... we dont have servo current to compute
    //temperature
  } //end switch

}


int command_calc(int i, float I_safe , float v_battery )
{
  //subroutine written by Vamfun...Mentor Vex 1508, 599.
  //7.13.2012  vamfun@yahoo.com... blog info  http://vamfun.wordpress.com

  //Vex motor assumed
  // inputs:  speed, motor speed in rpm  Note:***Program assumes that with positive command, rpm is positive.***
  //          I, motor current
  //          v_battery, average battery voltage >0
  // Output:  cmd, motor command necessary to create current I, [127 ,-127]


  I_safe = sgn(motor_input[i])*I_safe;
  if(i > 0 && i < 9)
  {
    I_safe = I_safe*90./128.;  // This compensates for the jpearlman  amplification correction in the current calculation functions.
  }

  switch(M[i].obj)//This model assumes that lamda >> 1
  //i_avg = dir*(V_b +V_d)*D/R  ? dir*V_d + V_bemf)/R
  // D = cmd/127 = (i_avg*R + dir*V_d + V_bemf)/dir*(V_b + V_d)
  // where dir = sign of input command.   +1  ==> V_bemf >0 in the steady state free speed.

  {
  case motor269:

    return  127.*(I_safe*R_269 + M[i].speed*Ke_269 + sgn(motor_input[i])*V_DIODE)/(v_battery + V_DIODE );
    break;
  case motor393:
    return  127.*(I_safe*R_393 + M[i].speed*Ke_393 + sgn(motor_input[i])*V_DIODE)/(v_battery + V_DIODE );
    break;
  case motor393highspeed:
    return  127.*(I_safe*R_393 + M[i].speed*Ke_393/1.6 + sgn(motor_input[i])*V_DIODE)/(v_battery + V_DIODE );
    break;

  case motor3wire:
    return  127.*(I_safe*R_3WIRE + M[i].speed*Ke_3WIRE + sgn(motor_input[i])*V_DIODE)/(v_battery + V_DIODE );
    break;

  default:

    return 0.;

    //servo
    //this is a potential problem... we dont have servo current to compute cmd  Assume 0.
  } //end switch

  //alternate model for testing.  This was not used due to large swings and sign changes when the command was near zero.
  // switch(M[i].obj)  //this model assumes that lamda <<1 or  pwm controller period T  is small with
  // //respect to motor inductive time constant L/R.
  // {
  // case motor269:

  // return  127.*I_safe*R_269/(sgn(motor_input[i])*v_battery - M[i].speed*Ke_269);
  // break;
  // case motor393:
  // return 127.*I_safe*R_393/(sgn(motor_input[i])*v_battery - M[i].speed*Ke_393);
  // break;
  // case motor393highspeed:
  // return 127.*I_safe*R_393/(sgn(motor_input[i])*v_battery - M[i].speed/1.6*Ke_393);
  // break;

  // case motor3wire:
  // return  127.*I_safe*R_3WIRE/(sgn(motor_input[i])*v_battery - M[i].speed*Ke_3WIRE);
  // break;

  // default:

  // return 0.;

  // //servo
  // //this is a potential problem... we dont have servo current to compute cmd  Assume 0.
  // } //end switch
}

float CalcCurrent269Vamfun( tMotor m, int cmd, int rpm, float v_battery  )
{
  /*-----------------------------------------------------------------------------*/
  /*                                                                             */
  /*  Estimate current in 269 motor using vamfun's algorithm                     */
  /*  subroutine written by Vamfun...Mentor Vex 1508, 599.                       */
  /*  7.13.2012  vamfun@yahoo.com... blog info  http://vamfun.wordpress.com      */
  /*                                                                             */
  /*  Modified by James Pearman 7.28.2012                                        */
  /*                                                                             */
  /*-----------------------------------------------------------------------------*/
  float   v_bemf;
  float   c1, c2;
  float   lamda;

  float   duty_on, duty_off;

  float   i_max, i_bar, i_0;
  float   i_ss_on, i_ss_off;


  // rescale control value
  // ports 2 through 9 behave a little differently
  if( m > port1 && m < port10 )
    cmd = (cmd * 128) / 90;

  // clip control value to +/- 127
  if( abs(cmd) > 127 )
    cmd = sgn(cmd) * 127;

  // which way are we turning ?
int dir = (cmd >= 0) ? 1 : (-1);

  duty_on = abs(cmd)/127.0;

  // constants for this pwm cycle
  lamda = R_269/((float)PWM_FREQ * L_269);
  c1    = exp( -lamda *    duty_on  );
  c2    = exp( -lamda * (1-duty_on) );

  // Calculate back emf voltage
  v_bemf  = Ke_269 * rpm;


  // Calculate staady state current for on and off pwm phases
  i_ss_on  =  ( v_battery * dir - v_bemf ) / (R_269 + R_SYS);
  i_ss_off = -( V_DIODE   * dir + v_bemf ) / R_269;

  // compute trial i_0
  i_0 = (i_ss_on*(1-c1)*c2 + i_ss_off*(1-c2))/(1-c1*c2);

  //check to see if i_0 crosses 0 during off phase if diode were not in circuit
  if(i_0*dir < 0)
  {
    // waveform reaches zero during off phase of pwm cycle hence
    // ON phase will start at 0
    // once waveform reaches 0, diode clamps the current to zero.
    i_0 = 0;

    // peak current
    i_max = i_ss_on*(1-c1);

    //where does the zero crossing occur
    duty_off = -log(-i_ss_off/(i_max-i_ss_off))/lamda ;
  }
  else
  {
    // peak current
    i_max = i_0*c1 + i_ss_on*(1-c1);

    // i_0 is non zero so final value of waveform must occur at end of cycle
    duty_off = 1 - duty_on;
  }


  // Average current
  i_bar = i_ss_on*duty_on + i_ss_off*duty_off;

  return i_bar;
}
float CalcCurrent393HighSpeedVamfun( tMotor m, int cmd, int rpm, float v_battery  )
{
  return CalcCurrent393Vamfun(m,cmd,(int)( (float)rpm/1.6),v_battery);
}
float CalcCurrent393Vamfun( tMotor m, int cmd, int rpm, float v_battery  )
{
  /*-----------------------------------------------------------------------------*/
  /*                                                                             */
  /*  Estimate current in Vex motor using vamfun's algorithm                     */
  /*  subroutine written by Vamfun...Mentor Vex 1508, 599.                       */
  /*  7.13.2012  vamfun@yahoo.com... blog info  http://vamfun.wordpress.com      */
  /*                                                                             */
  /*  Modified by James Pearman 7.28.2012                                        */
  /*                                                                             */
  /*-----------------------------------------------------------------------------*/
  float   v_bemf;
  float   c1, c2;
  float   lamda;

  float   duty_on, duty_off;

  float   i_max, i_bar, i_0;
  float   i_ss_on, i_ss_off;


  // rescale control value
  // ports 2 through 9 behave a little differently
  if( m > port1 && m < port10 )
    cmd = (cmd * 128) / 90;

  // clip control value to +/- 127
  if( abs(cmd) > 127 )
    cmd = sgn(cmd) * 127;

  // which way are we turning ?
int dir = (cmd >= 0) ? 1 : (-1);

  duty_on = abs(cmd)/127.0;

  // constants for this pwm cycle
  lamda = R_393/((float)PWM_FREQ * L_393);
  c1    = exp( -lamda *    duty_on  );
  c2    = exp( -lamda * (1-duty_on) );

  // Calculate back emf voltage
  v_bemf  = Ke_393 * rpm;


  // Calculate staady state current for on and off pwm phases
  i_ss_on  =  ( v_battery * dir - v_bemf ) / (R_393 + R_SYS);
  i_ss_off = -( V_DIODE   * dir + v_bemf ) / R_393;

  // compute trial i_0
  i_0 = (i_ss_on*(1-c1)*c2 + i_ss_off*(1-c2))/(1-c1*c2);

  //check to see if i_0 crosses 0 during off phase if diode were not in circuit
  if(i_0*dir < 0)
  {
    // waveform reaches zero during off phase of pwm cycle hence
    // ON phase will start at 0
    // once waveform reaches 0, diode clamps the current to zero.
    i_0 = 0;

    // peak current
    i_max = i_ss_on*(1-c1);

    //where does the zero crossing occur
    duty_off = -log(-i_ss_off/(i_max-i_ss_off))/lamda ;
  }
  else
  {
    // peak current
    i_max = i_0*c1 + i_ss_on*(1-c1);

    // i_0 is non zero so final value of waveform must occur at end of cycle
    duty_off = 1 - duty_on;
  }


  // Average current
  i_bar = i_ss_on*duty_on + i_ss_off*duty_off;

  return i_bar;
}
float CalcCurrent3WIREVamfun( tMotor m, int cmd, int rpm, float v_battery  )
{
  /*-----------------------------------------------------------------------------*/
  /*                                                                             */
  /*  Estimate current in Vex motor using vamfun's algorithm                     */
  /*  subroutine written by Vamfun...Mentor Vex 1508, 599.                       */
  /*  7.13.2012  vamfun@yahoo.com... blog info  http://vamfun.wordpress.com      */
  /*                                                                             */
  /*  Modified by James Pearman 7.28.2012                                        */
  /*                                                                             */
  /*-----------------------------------------------------------------------------*/

  float   v_bemf;
  float   c1, c2;
  float   lamda;

  float   duty_on, duty_off;

  float   i_max, i_bar, i_0;
  float   i_ss_on, i_ss_off;


  // rescale control value
  // ports 2 through 9 behave a little differently
  if( m > port1 && m < port10 )
    cmd = (cmd * 128) / 90;       //CDS todo.. this needs to be verified for three wire

  // clip control value to +/- 127
  if( abs(cmd) > 127 )
    cmd = sgn(cmd) * 127;

  // which way are we turning ?
int dir = (cmd >= 0) ? 1 : (-1);

  duty_on = abs(cmd)/127.0;

  // constants for this pwm cycle
  lamda = R_3WIRE/((float)PWM_FREQ * L_3WIRE);
  c1    = exp( -lamda *    duty_on  );
  c2    = exp( -lamda * (1-duty_on) );

  // Calculate back emf voltage
  v_bemf  = Ke_3WIRE * rpm;


  // Calculate staady state current for on and off pwm phases
  i_ss_on  =  ( v_battery * dir - v_bemf ) / (R_3WIRE + R_SYS);
  i_ss_off = -( V_DIODE   * dir + v_bemf ) / R_3WIRE;

  // compute trial i_0
  i_0 = (i_ss_on*(1-c1)*c2 + i_ss_off*(1-c2))/(1-c1*c2);

  //check to see if i_0 crosses 0 during off phase if diode were not in circuit
  if(i_0*dir < 0)
  {
    // waveform reaches zero during off phase of pwm cycle hence
    // ON phase will start at 0
    // once waveform reaches 0, diode clamps the current to zero.
    i_0 = 0;

    // peak current
    i_max = i_ss_on*(1-c1);

    //where does the zero crossing occur
    duty_off = -log(-i_ss_off/(i_max-i_ss_off))/lamda ;
  }
  else
  {
    // peak current
    i_max = i_0*c1 + i_ss_on*(1-c1);

    // i_0 is non zero so final value of waveform must occur at end of cycle
    duty_off = 1 - duty_on;
  }


  // Average current
  i_bar = i_ss_on*duty_on + i_ss_off*duty_off;

  return i_bar;
}
void initialize_object_state()
{
  /*-----------------------------------------------------------------------------*/
  /*                                                                             */
  /*  Initialize_object_state()                                                  */
  /*                                                                             */
  /*  subroutine written by Vamfun...Mentor Vex 1508, 599.                       */
  /*  8.21.2012  vamfun@yahoo.com... blog info  http://vamfun.wordpress.com      */
  /*  Routine used in support of PTC_current_monitor_template                    */
  /*  Initializes the object state names and initial temperatures                */
  /*  Takes #define data and sets up i_safe and i_safe_allocated for PTC objects */
  /*                                                                             */
  /*-----------------------------------------------------------------------------*/
  for(int j = 0;j<10;j++)
  {
    M[j].PE_port = 0;
    M[j].obj= motorType[(tMotor)j];
    motor_input[j]= 0;
    M[j].cmd_out =0;
    M[j].cmd_out_last = 0;
    M[j].speed = 0;
    M[j].I = 0;
    M[j].I_safe_alloc = 0.;
    M[j].I_safe=0;
    M[j].T = T_0;
    M[j].T_m = T_M;
    M[j].PTC_triped = false;


  }
  //set up the cortex and PE parameters
  M[10].T = T_0;
  M[10].T_m = T_M;
  M[10].obj = cortex1_5;
  M[11].T = T_0;
  M[11].T_m = T_M;
  M[11].obj = cortex6_10;
  M[12].T = T_0;
  M[12].T_m = T_M;
  M[12].obj = pwr_exp;

  setup_port_current_parameters();

}
void clear_motor_input_cmds()
{
  for(int j = 0;j<10;j++)
  {

    motor_input[j]= 0;
    // M[j].cmd_out =0;
    // M[j].cmd_out_last = 0;
  }
}
void update_object_state()
{
  /*-----------------------------------------------------------------------------*/
  /*                                                                             */
  /*  void update_object_state()                                                 */
  /*                                                                             */
  /*  subroutine written by Vamfun...Mentor Vex 1508, 599.                       */
  /*  8.21.2012  vamfun@yahoo.com... blog info  http://vamfun.wordpress.com      */
  /*  Routine used in support of PTC_current_monitor_template                    */
  /*  updates the object state currents and temperatures with period dt_ms       */
  /*                                                                             */
  /*                                                                             */
  /*-----------------------------------------------------------------------------*/

  compute_speeds();// this routine is written by user to load speeds into M[port].speed array variables

  //Reset cortex and PE object currents
  M[10].I= 0.;
  M[11].I= 0.;
  M[12].I= 0.;

  for(int j = 0;j<10;j++)
  {
    //compute port currents and temperatures
    if(M[j].obj == 0) continue; // if there is no object , then skip to the next object
      //port currents first
    switch(M[j].obj)
    {
    case motor269:
      // PTC HR30-090 used in 269
      M[j].I = CalcCurrent269Vamfun( j, M[j].cmd_out, M[j].speed, v_battery);
      break;
    case motor393:

      M[j].I = CalcCurrent393Vamfun( j, M[j].cmd_out, M[j].speed, v_battery);//CDS replace with actual 393
      break;
    case motor393highspeed:
      M[j].I = CalcCurrent393HighSpeedVamfun( j, M[j].cmd_out, M[j].speed, v_battery);//CDS replace with actual 393
      break;
    case motor3wire:
      // PTC  used in 3 wire
      M[j].I = CalcCurrent3WIREVamfun( j, M[j].cmd_out, M[j].speed, v_battery);//CDS replace with actual 3 wire
      break;

    default:
      //servo
      M[j].I = 0;//this is a potential problem... we dont have servo current to compute.  Assume 0.
      //temperature
    } //end switch

    //Update motor object temperatures
    M[j].T =M[j].T + temp_rate_PTC(j)*dt_ms;

    //Cumulate cortex and PE PTC currents.

    if(M[j].PE_port>0) // is it a Power Expander port
    {
      M[12].I = M[12].I + abs(M[j].I) ;  //save in PE max_PTC_I
    }
    else if(j>=0 && j<=4) // is it a cortex1_5 port
    {
      M[10].I = M[10].I + abs(M[j].I) ;  //save in cortex1_5 max_PTC_I
    }
    else  //it is a cortex6_10 port
    {
      M[11].I = M[11].I + abs(M[j].I) ;  //save in cortex1_5 max_PTC_I
    }
  }


  //update cortex and PE PTC temperatures
  M[10].T =M[10].T + temp_rate_PTC(10)*dt_ms;
  M[11].T =M[11].T + temp_rate_PTC(11)*dt_ms;
  M[12].T =M[12].T + temp_rate_PTC(12)*dt_ms;



  // write object motor cmds to the motors
  // limit the maximum command per update cycle to slew.   This creates a command rate limit or slew rate limit.
  // for( j = 0; j<10; j++ )
  // {
  // if(M[j].cmd_out > (M[j].cmd_out_last + slew))
  // {
  // motor[(tMotor)j] = M[j].cmd_out_last + slew;
  // M[j].cmd_out_last  = M[j].cmd_out_last + slew;
  // }
  // else if (M[j].cmd_out < (M[j].cmd_out_last - slew))
  // {
  // motor[(tMotor)j] = M[j].cmd_out_last - slew;
  // M[j].cmd_out_last  = M[j].cmd_out_last - slew;
  // }
  // else
  // {
  // motor[(tMotor)j] = M[j].cmd_out ;
  // M[j].cmd_out_last  = M[j].cmd_out ;
  // }
  // }

}//end update_object_state

void PTC_monitor_threshold_check()
{
  /*-----------------------------------------------------------------------------*/
  /*                                                                             */
  /*  void PTC_monitor_threshold_check()                                         */
  /*                                                                             */
  /*  subroutine written by Vamfun...Mentor Vex 1508, 599.                       */
  /*  8.21.2012  vamfun@yahoo.com... blog info  http://vamfun.wordpress.com      */
  /*  Routine used in support of PTC_current_monitor_template                    */
  /*  This function sets a PTC_tripped flag =1 if T_m for the PTC is exceeded    */
  /*  It resets the flag if T < .9*T_m                                             */
  /*                                                                             */
  /*-----------------------------------------------------------------------------*/

  for(int j = 0;j<13;j++)
  {
    if(M[j].obj == 0) continue; // if there is no object , then skip to the next object

    if(M[j].T > M[j].T_m)
    {

      M[j].PTC_triped = 1;
    }
    else if(M[j].T < .9*M[j].T_m)
    {

      M[j].PTC_triped = 0;
    }
  }

}
void setup_port_current_parameters(  )
{
  // This routine initializes the object_state array with PTC monitor parameters:
  // safe_current,allocated safe_current


  int j = 0.;
  //Assign Power Expander Ports to object_state array

  if(PE_CH1 <10 ) M[PE_CH1].PE_port = 1; //if a channel is unused then PE_CHx = 99 = UNUSED
    if(PE_CH2 <10 ) M[PE_CH2].PE_port = 1;
  if(PE_CH3 <10 ) M[PE_CH3].PE_port = 1;
  if(PE_CH4 <10 ) M[PE_CH4].PE_port = 1;

  //Assign spec and safe currents to object_state array motors

  for  (j = 0;j<10;j++)
  {
    if(M[j].obj == 0) continue; // if there is no object , then skip to the next object
      switch(M[j].obj){
    case motor393highspeed:
    case motor393:
      M[j].I_safe=  SAFE_CURRENT_393;
      break;
    case motor269:
      M[j].I_safe=  SAFE_CURRENT_269;
      break;
    case motor3wire:
      M[j].I_safe=  SAFE_CURRENT_3WIRE;
      break;
    case motorservo:
      M[j].I_safe=  SAFE_CURRENT_SERVO;
      break;
    default:
      M[j].I_safe=  0;

    }//end switch



    //Accumulate motor PTC safe currents for cortex and PE PTC objects.

    if(M[j].PE_port>0) // is it a Power Expander port
    {
      M[12].cum_I_safe = M[12].cum_I_safe + M[j].I_safe ;  //save in PE cum_I_safe

    }
    else if(j>=0 && j<=4) // is it a cortex1_5 port
    {
      M[10].cum_I_safe = M[10].cum_I_safe + M[j].I_safe ;  //save in cortex1_5 cum_I_safe

    }
    else  //it is a cortex6_10 port
    {
      M[11].cum_I_safe = M[11].cum_I_safe + M[j].I_safe ;  //save in cortex6_10 cum_I_safe

    }
  }
  M[10].I_safe = SAFE_CURRENT_CORTEX;
  M[11].I_safe = SAFE_CURRENT_CORTEX;
  M[12].I_safe = SAFE_CURRENT_PE;
  //Compute default safe commands for motors feeding cortex and PE

  for( j = 0; j<10; j++)
  {
    if(M[j].obj == 0 ) continue; // if there is no object , then skip to the next object

    if( j< 5 && M[j].PE_port==0)
    {
      //Allocate    I_safe_alloc = ratio*I_safe[cortex or PE]  where ratio = I_safe[cortex or PE]/cum_I_safe
      //and cum_I_safe = sum (I_safe of all motors feeding the cortex or PE PTC)
      //This is necessary because the sum of all I_safe of motors can exceed the I_safe[cortex or PE] PTC
      //So we reduce the individual motor safe commands based upon how much they contribute to the total safe current.
      //We could just divide by the I_safe[cortex or PE] by the number of motors feeding it
      //but we need to account for differnt types of motors.  I could have used I_stall to ratio it but I_safe is just as good.

      M[j].I_safe_alloc = M[j].I_safe*M[10].I_safe/M[10].cum_I_safe;
    }
    else if (j >4 && M[j].PE_port==0)
    {
      M[j].I_safe_alloc = M[j].I_safe*M[11].I_safe/M[11].cum_I_safe;
    }
    else if (M[j].PE_port == 1)
    {
      M[j].I_safe_alloc = M[j].I_safe*M[12].I_safe/M[12].cum_I_safe; //Power expander
    }

  }

}//end Setup_current_parameters
