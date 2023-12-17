#pragma once

#define LOOPTIME (0.02)

#define ONE_SEC ((int)(1.0 / LOOPTIME))
#define HALF_SEC ((int)(0.5 * ONE_SEC))
#define QUART_SEC ((int)(0.25 * ONE_SEC))

// * Resets all CTRE Falcons and encoders to factory state
// #define CTRE_FACTORY_RESET

// absolute encoder constants
#define DMAX (1024.0 / 1025.0)
#define DMIN (1.0 / 1025.0)

// DIO assignments
typedef enum
{
    ARM_ENCODER = 0,
    WRIST_ENCODER = 2,
    INTAKE_BEAM_BREAK = 3,
    LED_LIGHT_OUTPUT_0 = 4,
    LED_LIGHT_OUTPUT_1 = 5,
    LED_LIGHT_OUTPUT_2 = 6,
    EXTENDER_ENCODER_A = 7,
    EXTENDER_ENCODER_B = 8,
} digitals;

// (FC) - Swerve scale factors
//* Normal Drive Team: 
#define XY_SFACTOR (200.0)
#define XY_SFACTOR_LOW (100.0)
#define R_SFACTOR (200.0)
#define R_SFACTOR_LOW (100.0)
//* Practice Drive Team
// #define XY_SFACTOR (125.0)
// #define XY_SFACTOR_LOW (75.0)
// #define R_SFACTOR (125.0)
// #define R_SFACTOR_LOW (75.0)

// use numbers below for slowing drivetrain
// #define XY_SFACTOR (50.0)
// #define R_SFACTOR (50.0)

// (FC) - Flight controller
#define FC_X_DEADBAND (0.1)
#define FC_Y_DEADBAND (0.1)
#define FC_R_DEADBAND (0.1)
#define FC_X_SHAPING (0.6)
#define FC_Y_SHAPING (0.6)
#define FC_R_SHAPING (1.0)

#define SIGN(x) (((x) > 0) ? (1) : (-1))

// CAN device IDs
typedef enum
{
    PDP_MODULE = 0,
    // Swerve
    // Module 1
    FRONT_RIGHT_DRIVE = 1,
    FRONT_RIGHT_STEER = 5,
    FRONT_RIGHT_ABSOLUTE_ENCODER = 9,
    // Module 2
    BACK_RIGHT_DRIVE = 2,
    BACK_RIGHT_STEER = 6,
    BACK_RIGHT_ABSOLUTE_ENCODER = 10,
    // Module 3
    BACK_LEFT_DRIVE = 3,
    BACK_LEFT_STEER = 7,
    BACK_LEFT_ABSOLUTE_ENCODER = 11,
    // Module 4
    FRONT_LEFT_DRIVE = 4,
    FRONT_LEFT_STEER = 8,
    FRONT_LEFT_ABSOLUTE_ENCODER = 12,

    PCM_PNEUMATICS_MODULE = 13,

    CAN_ARM_MOTOR = 14,
    CAN_EXTENDER_MOTOR = 15,

} can;
typedef enum
{
    PITCH_GYRO = 0
} analog;
typedef enum
{
    ROLLER_CURRENT_IN = 9,
} pdp;

typedef enum
{
    PCM_WRIST_1 = 0,
    PCM_WRIST_2 = 1,
} pcm;

typedef enum
{
    PWM_WRIST_MOTOR = 0,
    PWM_ROLLER_MOTOR = 1,
} pwm;

typedef enum
{
    X_AXIS = 1,
    Y_AXIS = 5,
    Z_AXIS = 0

} flightctrl_axis;

typedef enum
{
    CONE_ARM = 1,
    CUBE_ARM = 3
} gamepad_axis;

typedef enum
{
    GYRO_BUTTON_SWITCH = 1,
    FIELD_ROBOT_SWITCH = 2,
    RESET_GYRO_BUTTON = 3,
    // VISION_RIGHT_BUTTON = 7,
    GRID_ALIGN_BUTTON_1 = 6,
    GRID_ALIGN_BUTTON_2 = 7,
    WRIST_BIAS_INCREASE = 8,
    WRIST_BIAS_DECREASE = 9,
    BEEP_FOREWARD_BUTTON = 10,
    BEEP_BACKWARD_BUTTON = 11,
    BEEP_RIGHT_BUTTON = 12,
    BEEP_LEFT_BUTTON = 13,
    VISION_LEFT_BUTTON = 14,
    VISION_CENTER_BUTTON = 15,


} flightctrl_buttons;

typedef enum
{
    STOW_POS_BUTTON = 1,
    TOP_ROW_BUTTON = 2,
    MIDDLE_ROW_BUTTON = 3,
    BOTTOM_ROW_BUTTON = 4,
    COLLECT_CUBE_BUTTON = 5,
    COLLECT_CONE_BUTTON = 6,
    EJECT_CUBE_BUTTON = 7,
    EJECT_CONE_BUTTON = 8,
    // VISION_LEFT_BUTTON = 9,
    // VISION_CENTER_BUTTON = 10,
    SUB_OPEN_BUTTON = 9,
    SUB_BIAS_CLOSE_BUTTON = 10,


} gamepad_buttons;

typedef enum
{
    STOW_POS = 0,
    HIGH_POS = 1,
    MID_POS = 2,
    LOW_POS = 3,
    PICKUP_POS = 4,
    SUB_POS = 5,
    REARPICKUP_POS = 6,
} Position;

typedef enum
{
    NONE = 0,
    CONE = 1,
    CUBE = 2
} Gamepiece;

// Moved to digitals enum
//  typedef enum
//  {
//      LED_LIGHT_OUTPUT_0 = 0,
//      LED_LIGHT_OUTPUT_1 = 1,
//      LED_LIGHT_OUTPUT_2 = 2,

// } digital_out;

typedef struct
{
    double x;
    double y;
    double z;
    double rx;
    double ry;
    double rz;
} Pose;

typedef struct
{
    Pose pose;
    double fID;
    double ta;
} Target;

typedef enum
{
    ROBOT_FRAME = 0,
    FIELD_FRAME = 1

} VisionFrame;
typedef enum
{
    BEST_TARGET = 0,
    LOCK_TARGET = 1
} TargetFilter;
typedef enum
{
    DRIVER_CAM = 0,
    APRIL_TAG_CAM = 1
} CameraMode;

#define GRID_ALIGN_ANGLE (0.0)
#define LOADING_ZONE_ANGLE (180)
#define GYRO_ANGLE_KP (2.0)