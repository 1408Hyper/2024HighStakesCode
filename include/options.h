// All the options for the robot

#ifndef _HYPER_OPTIONS_H_
#define _HYPER_OPTIONS_H_

// Variables (u can change these!!)

// Whether to make the torus sensor reject red or blue colors
#define REJECT_COLOR_RED false
#define DO_REJECT_COLOR false

// Main opcontrol function to use
#define CURRENT_OPCONTROL mainControl

// Digital sensor port for pneumatics mogo mech
#define MOGO_MECH_PORT 'A'

// Digital sensor port for pneumatics conveyor lift
#define LIFT_MECH_PORT 'B'

//optical sensor for colour
#define CONV_TORUS_PORT 20

// Digital sensor port for pneumatics doinker
#define DOINKER_PORT 'B'

// Hanging Mehcanism
#define HANGING_MECH_PORT 'E'

#define ODOM_ENC_PORTS {'C', 'D'}

// Ultrasound (1st is ping port, 2nd is echo port)
#define BACK_ULTRA_PORTS {'G', 'F'}

// Motor ports for the conveyer (real is 10 but use 11 to just turn it off for debugging)
#define CONVEYER_PORTS {4, -10}

// Motor ports for the intake motor group
#define INTAKE_PORTS {11, -12}

// Motor ports for the lady brown mech
#define LADY_BROWN_PORTS {1}

// Port for the distance sensor
#define DIST_SENSOR_PORT 16

// Port for the mogo sensor
#define MOGO_SENSOR_PORT 17

#define LADY_BROWN_ROT_SENSOR_PORT 9

//Port for the GPS Sensor
#define GPS_SENSOR_PORT 2

// Ports for telemetry
// IMU
#define IMU_PORT 12

// Turn on/off auton and opcontrol
#define DO_MATCH_AUTON false
#define DO_SKILLS_AUTON false

// Turn on for match auton to be run at the start of opcontrol
#define MATCH_AUTON_TEST false
// Turn on for skills prep/post auton/opcontrol functions to be run on components
#define DO_SKILLS_PREP true
#define DO_POST_AUTON true
#define DO_OP_CONTROL true

// Ports for the drivetrain motors
#define LEFT_DRIVE_PORTS {8, 21, 18}
#define RIGHT_DRIVE_PORTS {-13, -14, -15}

// Chassis class to use (default is initDefaultChassis)
#define INIT_CHASSIS initDefaultChassis

#endif // _HYPER_OPTIONS_H_
