// Describes messages between computers and the UDP/TCP functions for moving those messages

#ifndef __FIELD_REMOTE_H
#define __FIELD_REMOTE_H
#include <vector>


enum MSG_TYPE2
{
    // Messages for sending on the slow link to the OCU
    MSG_TYPE2_PACKED_MSG_ONE,
    MSG_TYPE2_JPEG_USER_REQUESTED,
    MSG_TYPE2_SHOW_STEPS,
    MSG_TYPE2_STEP_GENERATOR_BOUNDARY,
    MSG_TYPE2_DEBRIS_FSM_EVENTS,    // /decision_making/debris_fsm/events of type std_msgs::String
    MSG_TYPE2_VALVE2_FSM_EVENTS,
    MSG_TYPE2_VALVE_FSM_EVENTS,    // /decision_making/valve_fsm/events of type std_msgs::String
    MSG_TYPE2_SURPRISE_FSM_EVENTS, 
    MSG_TYPE2_VALVE_GRIP_TORUS,
    MSG_TYPE2_TRAJOPT_TRAJ_ARRAY,
    MSG_TYPE2_GUI_DRILL_STRING,
    MSG_TYPE2_GUI_DRILL_FSM_FEEDBACK,           // Status of drill task, used to update GUI
    MSG_TYPE2_PIPELINE_IMG,
    MSG_TYPE2_WALL_POINTS,
    MSG_TYPE2_DETECTED_CYLINDER,
    MSG_TYPE2_LAST_MESSAGE_SLOW_TO_OCU,  // Messages on the slow link to the OCU must be before this line

    // Messages for sending on the fast link to the OCU
    MSG_TYPE2_FIRST_MESSAGE_FAST_TO_OCU,
    MSG_TYPE2_ASSEMBLED_CLOUD_TRANSFORM,
    MSG_TYPE2_POINT_CLOUD_L_FOOT,  // point cloud referenced to /l_foot
    MSG_TYPE2_POINT_CLOUD_L_FOOT_DENSE,  // point cloud referenced to /l_foot
    MSG_TYPE2_POINT_CLOUD_L_FOOT_DENSE_LAST,  // point cloud referenced to /l_foot
    MSG_TYPE2_POINT_CLOUD_CMU_ROOT_DENSE,
    MSG_TYPE2_POINT_CLOUD_CMU_ROOT_DENSE_LAST,
    MSG_TYPE2_POINT_CLOUD_ATLAS_ODOM_DENSE,
    MSG_TYPE2_POINT_CLOUD_ATLAS_ODOM_DENSE_LAST,
    MSG_TYPE2_POINT_CLOUD_ASSEMBLED_DENSE,
    MSG_TYPE2_POINT_CLOUD_ASSEMBLED_DENSE_LAST,
    MSG_TYPE2_POINT_CLOUD_STEPGEN_DEBUG,
    MSG_TYPE2_POINT_CLOUD_STEPGEN_DEBUG_LAST,
    MSG_TYPE2_JPEG_LHEAD,          // jpeg image from left head camera (color)
    MSG_TYPE2_JPEG_RHEAD,          // jpeg image from right head camera (mono)
    MSG_TYPE2_JPEG_LLHAND,         // jpeg image from left hand, left camera
    MSG_TYPE2_JPEG_RLHAND,         // jpeg image from right hand, left camera
    MSG_TYPE2_JPEG_SITCAM_LEFT,    // jpeg image from left situational awareness camera
    MSG_TYPE2_JPEG_SITCAM_RIGHT,   // jpeg image from right situational awareness camera
    MSG_TYPE2_JPEG_SITCAM_BACK,    // jpeg image from back / rear situational awareness camera
    MSG_TYPE2_JPEG_FOLLOW_LEFT_HAND, // jpeg following left hand
    MSG_TYPE2_JPEG_FOLLOW_RIGHT_HAND, // jpeg following right hand
    MSG_TYPE2_LHEAD_CI,            // camera info from left head camera (color)
    MSG_TYPE2_RHEAD_CI,            // camera info from right head camera (mono)
    MSG_TYPE2_LLHAND_CI,           // camera info from left hand, left camera
    MSG_TYPE2_RLHAND_CI,           // camera info from right hand, left camera
    MSG_TYPE2_SITCAM_LEFT_CI,      // camera info from left situational awareness camera
    MSG_TYPE2_SITCAM_RIGHT_CI,     // camera info from right situational awareness camera
    MSG_TYPE2_SITCAM_BACK_CI,      // jpeg image from back / rear situational awareness camera
    MSG_TYPE2_DATA_TEST,
    MSG_TYPE2_GUI_PERCEPTION_FEEDBACK,
    MSG_TYPE2_DOOR_NORMALS,        // /door_normals [visualization_msgs/MarkerArray]
    MSG_TYPE2_DOOR_UPDATE,         // /door_task/door_controller_topic_status_update [wrecs_msgs/DoorMotion]
    MSG_TYPE2_DOOR_ESTIMATED_MODEL,// /estimated_model_ [visualization_msgs::MarkerArray] - From that same test_mbd node
    MSG_TYPE2_DOOR_CAL_HELPER,     // /calibration_helper_head_model_guess_frame/update [visualization_msgs/InteractiveMarker]
    MSG_TYPE2_TRAJOPT_TRAJ_ARRAY_WITH_POLYGON_MESH,
    MSG_TYPE2_SF_CONTROLLER_OUT,
    MSG_TYPE2_CMU_POSE,
    MSG_TYPE2_UDP_BIG_FIRST,       // Used for re-assembling big packets
    MSG_TYPE2_UDP_BIG_MIDDLE,      // Used for re-assembling big packets
    MSG_TYPE2_UDP_BIG_LAST,        // Used for re-assembling big packets
    MSG_TYPE2_UDP_BIG_FIRST_SLOW,       // Used for re-assembling big packets
    MSG_TYPE2_UDP_BIG_MIDDLE_SLOW,      // Used for re-assembling big packets
    MSG_TYPE2_UDP_BIG_LAST_SLOW,        // Used for re-assembling big packets
    MSG_TYPE2_CAR_EGRESS_STATE,
    MSG_TYPE2_DEPTH_IMAGE,
    MSG_TYPE2_WALK_PERCPTION_GOAL,
    MSG_TYPE2_BATTERY_STATUS,
    MSG_TYPE2_KNEE_IMAGE_RIGHT,
    MSG_TYPE2_KNEE_IMAGE_LEFT,
    MSG_TYPE2_HAND_IMAGE_RIGHT,
    MSG_TYPE2_HAND_IMAGE_LEFT,
    MSG_TYPE2_ATLAS_ODOMETRY,      // odometry for driving
    MSG_TYPE2_DETECTED_DOOR_AND_HANDLE,   // /door_task/detected_door_and_handle [type: sensor_msgs::Image]
    MSG_TYPE2_VISUAL_SERVO_OBJECTS,
    MSG_TYPE2_FOREARM_STATUS,
    MSG_TYPE2_MULTISENSE_CONTROL_RETURN,
    MSG_TYPE2_VALVE_CLOUD,
    MSG_TYPE2_PING,
    MSG_TYPE2_DRILL_STATUS,
    MSG_TYPE2_STEP_GENERATOR_STAIR_DATA,             // For the stairs
    MSG_TYPE2_CAR_POSITION,
    MSG_TYPE2_EGRESS_POINTS2, // cga egress percpetion data
    MSG_TYPE2_EGRESS_MARKERS,
    MSG_TYPE2_EGRESS_LASER_TRACK,
    // plugh - add your new enum right above this line


    // Should any of these be on slow link rather than the intermittent fast link?
    MSG_TYPE2_STRING,              // string to print out
    MSG_TYPE2_PUMP_STATUS,
    MSG_TYPE2_SMODEL_RIGHT_INPUT,
    MSG_TYPE2_SMODEL_LEFT_INPUT,
    MSG_TYPE2_STEP_INDEX,
    MSG_TYPE2_CAR_VELOCITY_FEEDBACK, // How fast is the car going
    MSG_TYPE2_CAR_STEERING_WHEEL_POINTS, // Points used for steering - debug topic
    MSG_TYPE2_STEERING_ANGLE_FEEDBACK,


    // Messages for sending on the slow link to the onboard field computers
    MSG_TYPE2_FOLLOW_JOINT,
    MSG_TYPE2_SMODEL_RIGHT_OUTPUT,
    MSG_TYPE2_SMODEL_LEFT_OUTPUT,
    MSG_TYPE2_STEP_RELATIVE_LEFT_FOOT,
    MSG_TYPE2_FIELD_PARAM_CMU_WALK,
    MSG_TYPE2_LADAR_FILTER_DISTANCES,
    MSG_TYPE2_SITCAM_SETTINGS_RIGHT,
    MSG_TYPE2_SITCAM_SETTINGS_LEFT,
    MSG_TYPE2_MOVEIT_TRAJECTORY,
    MSG_TYPE2_SIMPLE_WALK_PARAM,
    MSG_TYPE2_FOVEAL_PARAM,
    MSG_TYPE2_SF_CAM_PARAM,
    MSG_TYPE2_FIELD_COMMAND,
    MSG_TYPE2_COMM_SIM_ON,  // Turns simulation of comm. link of and off
    MSG_TYPE2_COMM_SIM_BPS, // Bits per second of comm. link - expected 2000 to 9600
    MSG_TYPE2_START_STEP,
    MSG_TYPE2_MAIN_MODE,
    MSG_TYPE2_CAR_STEERING, // range is a bit more than +-3 PI
    MSG_TYPE2_CAR_GAS_BRAKE,
    MSG_TYPE2_CAR_DRIVE_AHEAD_TIME,
    MSG_TYPE2_CAR_COMMAND,
    MSG_TYPE2_CAR_STEERING_WHEEL_LOCATION,
    MSG_TYPE2_GUI_DRILL_SHAKE,
    MSG_TYPE2_REQUEST_JPEG,
    MSG_TYPE2_ATLAS_SENSOR_WRIST_FORCE_RIGHT_FILTERED,
    MSG_TYPE2_ATLAS_SENSOR_WRIST_FORCE_RIGHT,
    MSG_TYPE2_GUI_DRILL_TRAJECTORY,
    MSG_TYPE2_STEP_GENERATOR_START,
    MSG_TYPE2_STEP_GENERATOR_CLEAR,
    MSG_TYPE2_STEP_GENERATOR_DELETE_STEP,
    MSG_TYPE2_STEP_GENERATOR_INSERT_STEP,
    MSG_TYPE2_STEP_GENERATOR_CHANGE_STEP,
    MSG_TYPE2_STEP_GENERATOR_CHANGE_STEPS,
    MSG_TYPE2_STEP_GENERATOR_Z_HEIGHT,
    MSG_TYPE2_STEP_GENERATOR_REPLAN,
    MSG_TYPE2_STEP_GENERATOR_GOAL,
    MSG_TYPE2_STEP_GENERATOR_BOUNDARY_CHANGE,
    MSG_TYPE2_STEP_GENERATOR_STAIR_CHANGE,
    MSG_TYPE2_DEBRIS_FSM,          // /debris_fsm  of type debris::DebrisFsm
    MSG_TYPE2_DEBRIS_FSM_MANUAL,   // /debris_fsm_manual of type debris::DebrisFsm
    MSG_TYPE2_TRAJOPT_REMOTE_CONFIRM, // /confirm_trajectory of type trajopt_msgs::RemoteConfirm
    MSG_TYPE2_GUI_PERCEPTION_SCRIBBLE,
    MSG_TYPE2_GUI_PERCEPTION_OBJ_SETTING,
    MSG_TYPE2_GUI_DRILL_FSM_CONTROL,
    MSG_TYPE2_DOOR_GUI_EVENT_UPDATE, // /door_task/door_gui_event_update [wrecs_msgs/DoorMotion]
    MSG_TYPE2_DOOR_BODY_HANDLE,      // /model_based_detection/door_body_handle [geometry_msgs/PoseStamped]
    MSG_TYPE2_VALVE_GRIP_TORUS_GUI,
    MSG_TYPE2_VALVE_FSM,
    MSG_TYPE2_VALVE_FSM_MANUAL,
    MSG_TYPE2_VALVE2_FSM,
    MSG_TYPE2_VALVE2_FSM_MANUAL,
    MSG_TYPE2_SURPRISE_FSM,
    MSG_TYPE2_SURPRISE_FSM_MANUAL,
    MSG_TYPE2_NODE_MANAGEMENT,
    MSG_TYPE2_TRAJOPT_COMPUTE_TRAJ,
    MSG_TYPE2_TRAJOPT_LOAD_POINT_CLOUD,
    MSG_TYPE2_TRAJOPT_LOAD_OBJECT,
    MSG_TYPE2_TRAJOPT_PROBLEM_PARAMETERS,
    MSG_TYPE2_TRAJOPT_SET_VIEWER,
    MSG_TYPE2_TRAJOPT_NUDGE,
    MSG_TYPE2_TRAJOPT_GRAB,
    MSG_TYPE2_TRAJOPT_OTHER_HAND,
    MSG_TYPE2_TRAJOPT_SAVE_TRAJECTORY,
    MSG_TYPE2_TRAJOPT_LOAD_TRAJECTORY,
    MSG_TYPE2_PLUG_FSM,
    MSG_TYPE2_PLUG_FSM_MANUAL,
		MSG_TYPE2_BX_INITEKF,
    MSG_TYPE2_SF_MANIP_NUDGE,
    MSG_TYPE2_SF_MANIP_NUDGE_THREED,
    MSG_TYPE2_SF_MANIP_SETHANDPOSITION,
    MSG_TYPE2_SF_MANIP_SETHANDORIENTATION,
    MSG_TYPE2_SF_MANIP_SETHAND,
    MSG_TYPE2_SF_MANIP_SETAXIALGAIN,
    MSG_TYPE2_SF_MANIP_SETFTSERVO,
    MSG_TYPE2_SF_MANIP_DISABLEFTSERVO,
    MSG_TYPE2_SF_MANIP_RESETPOSTURE,
    MSG_TYPE2_SF_MANIP_COMHOME,
    MSG_TYPE2_SF_MANIP_NUDGECOM,
    MSG_TYPE2_SF_MANIP_PELVZ,
    MSG_TYPE2_SF_MANIP_UTORSOYAW,
    MSG_TYPE2_SF_MANIP_RESUMECOMMANDS,
    MSG_TYPE2_SF_MANIP_STOP,
    MSG_TYPE2_SF_MANIP_SETJOINTS,
    MSG_TYPE2_ROBOTIQ_RIGHT_RESET,
    MSG_TYPE2_ROBOTIQ_RIGHT_SETFORCE,
    MSG_TYPE2_ROBOTIQ_RIGHT_SETPERSISTANTGRASP,
    MSG_TYPE2_ROBOTIQ_RIGHT_SETPOSITION,
    MSG_TYPE2_ROBOTIQ_RIGHT_SETSCISSOR,
    MSG_TYPE2_ROBOTIQ_RIGHT_SETSTATE,
    MSG_TYPE2_ROBOTIQ_RIGHT_SETVELOCITY,
    MSG_TYPE2_ROBOTIQ_LEFT_RESET,
    MSG_TYPE2_ROBOTIQ_LEFT_SETFORCE,
    MSG_TYPE2_ROBOTIQ_LEFT_SETPERSISTANTGRASP,
    MSG_TYPE2_ROBOTIQ_LEFT_SETPOSITION,
    MSG_TYPE2_ROBOTIQ_LEFT_SETSCISSOR,
    MSG_TYPE2_ROBOTIQ_LEFT_SETSTATE,
    MSG_TYPE2_ROBOTIQ_LEFT_SETVELOCITY,
    MSG_TYPE2_VALVE_VALVEPARA,
    MSG_TYPE2_ROBOTIQ_POWER_CYCLE,
    MSG_TYPE2_LATENCY_ACK_FIELD,
    MSG_TYPE2_LATENCY_ACK_OCU,  // Need different message type to correctly simulate latency
    MSG_TYPE2_TASKUTILS_DRILL,
    MSG_TYPE2_MULTISENSE_CONTROL,
    MSG_TYPE2_SHUFFLE,
    MSG_TYPE2_EGRESS_LASER_CONTROL, // remote control of cga egress perception
    MSG_TYPE2_ASK_FOR_IMG,


    MSG_TYPE2_STATUS,
    MSG_TYPE2_H264,
    MSG_TYPE2_COMMAND,

    MSG_TYPE2_CRITICAL_REPORT,
    MSG_TYPE2_INJURY_REPORT,
    MSG_TYPE2_VITALS_REPORT,
    MSG_TYPE2_RTCM,
    MSG_TYPE2_INITIAL_REPORT,


    // xyzzy - add your new enum right above this line

    // Not currently used
    MSG_TYPE2_JPEG_FALSE_COLOR,    // jpeg false color image
    MSG_TYPE2_SF_IMG,              // custom Siyuan message
    MSG_TYPE2_EW_IMG,
    MSG_TYPE2_JPEG_PERC,           // jpeg perception image
    MSG_TYPE2_HANDLE_CONTROL,
    MSG_TYPE2_FIELD_STATE,
    MSG_TYPE2_ACK,
    MSG_TYPE2_DRILL_SHAKE,
    MSG_TYPE2_LOGGING_SYSTEM,

    MSG_TYPE2_MAX // add any additional types before this
};


static const int max_UDP_size = 1456; // Calculated as 1500 - 28 UDP/IP - 10 our header = 1462, add 6 bytes for margin (may not be needed)
//static const unsigned int maximumMessageSize = 65000;
static const unsigned int maximumMessageSize = 3*1024*1024;
static const int LOW_LEVEL_MSG_HEADER_SIZE = 10;
// This limits the maximum image size to 65000 bytes
// Notes - UDP or sockets on Linux has problems with sizes larger than 65000 (66000 fails even locally)
// Also, this may not work over the internet - dropped and out of order packets may make it fail
// TCP/IP, RTP, UDT or another protocol may be required
struct LOW_LEVEL_MSG {
    unsigned short startBytes;
    unsigned short msgType;
    unsigned int   bytesOfData;
    unsigned short msgID;
    //int ackID;
    //short numParts;
    //short thisPartNumber;
    //double timeStamp;
    unsigned char data[maximumMessageSize];
};


// Function to send UDP packet
// Return value is actual number of bytes sent
int send_UDP(LOW_LEVEL_MSG *msg, const char *dest, int port);


// Returns sockfd for UDP reception (<0 indicates failure)
int init_recv_UDP(int port);

// Function to receive LOW_LEVEL_MSG, initializes connection on first call
// Return value of 0 - no message received, *msg is unchanged
// Return value of 1 - message received, *msg contains new message
int recv_UDP(LOW_LEVEL_MSG *msg, int sockfd);

#endif // __FIELD_REMOTE_H
