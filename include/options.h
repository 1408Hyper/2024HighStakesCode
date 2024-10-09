// All the options for the robot
// (experimental)

// Variables (u can change these!!)
#define DELAY_TIME_MS 20
// turn on for auton to be run at the start of opcontrol
#define AUTON_TEST false

#define CURRENT_OPCONTROL mainControl

// Digital sensor port for pneumatics mogo mech
#define MOGO_MECH_PORT 'A'

//Digital sensor port for pneumatics conveyor lift
#define CONV_MECH_PORT 'B'

// Motor ports for the conveyer
#define CONVEYER_PORTS {11, -12}

// Motor ports for the conveyer

// Turn on/off auton and opcontrol
// Both DO_AUTON and AUTON_TEST must be true for auton to run at the start of opcontrol
#define DO_AUTON true
#define DO_OP_CONTROL true

// Ports for the drivetrain motors
#define LEFT_DRIVE_PORTS {10, 9, 8}
#define RIGHT_DRIVE_PORTS {20, 19, 18}

#define INIT_CHASSIS initDefaultChassis
