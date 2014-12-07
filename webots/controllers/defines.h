#ifndef CONN_SWARM_DEFINES
#define CONN_SWARM_DEFINES

// Simulation parameters
#define EXP_TIME          10000 // Duration of the experiment (in seconds)
#define TIME_STEP         25 // Simulation time step (ms), must be a multiple of basicTimeStep in WorldInfo
#define NUM_ROBOTS        4 // Total number of robots in the scene
// Alpha algorithm
#define ALPHA                3 // Minimum number of neighbors to maintain
#define BETA				         5
#define MAX_RANDOM_TURN      50 // Maximum value of the random turn (in degrees)
#define MAX_COHERENCE_TIME   80 // Maximum number of timesteps spent in coherence state
#define MAX_AVOIDANCE_TIME   5 // Maximum number of timesteps spent in avoidance sub-state
// Communication
#define COMM_RADIUS           0.7    // Range of radio communication (in meters)
#define COMMUNICATION_CHANNEL 1      // Channel to use for inter-robot communication
#define COMMUNICATION_CHANNEL_STAT 2 // Another channel to send stats & state to the supervisor
#define COMMUNICATION_CHANNEL_DEBUG 3 // channel for debugging messages
// Interval between two broadcasts (in number of timesteps)
#define COMMUNICATION_PERIOD 20

// Enable / disable experiment logging
#define LOG_EXPERIMENT   true
// Detailed logs contain the state of each particular robot
#define LOG_DETAILS      false
// Path to the logs (relative to the supervisor's directory)
#define LOG_FILES_FOLDER "../../data"

// Initial agents distribution parameters
#define INITIAL_BOX_SIDE  50
#define NEIGHBOURHOOD     5

// AUXILIARY
#define NB_SENSORS           8   // number of sensors
#define BIAS_SPEED           400 // robot bias speed
#define MAXSPEED             800 // maximum robot speed
#define SENSOR_NORMALIZATION 100 // Normalisation constant for the IR sensors measurements
#define OBSTACLE_THRESHOLD   100 // IR sensor value from which we consider there is an obstalce
#define LIGHT_THRESHOLD 100 // light intensity value larger than which illuminates the robot

/**
 * FORWARD_AVOIDANCE corresponds to 1
 * COHERENCE_AVOIDANCE corresponds to 3
 * But in practice, we use a boolean flag to represent
 * the subsumed AVOIDANCE state.
 */
typedef enum {FORWARD = 0, COHERENCE = 2} State;

#endif
