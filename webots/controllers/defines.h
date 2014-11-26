#ifndef CONN_SWARM_DEFINES
#define CONN_SWARM_DEFINES

// Simulation parameters
#define EXP_TIME          900 // Duration of the experiment (in seconds)
#define TIME_STEP         64 // Simulation time step (ms)
#define NUM_ROBOTS        10 // Total number of robots in the scene
// Alpha algorithm
#define ALPHA                4 // Minimum number of neighbors to maintain
#define MAX_RANDOM_TURN      60 // Maximum value of the random turn (in degrees)
#define MAX_COHERENCE_TIME   15 // Maximum number of timesteps spent in coherence state
#define MAX_AVOIDANCE_TIME   5 // Maximum number of timesteps spent in avoidance sub-state
// Communication
#define COMM_RADIUS           0.7 // Range of radio communication (in meters)
#define COMMUNICATION_CHANNEL 1

// Initial agents distribution parameters
#define INITIAL_BOX_SIDE  50
#define NEIGHBOURHOOD     5

// AUXILIARY
#define NB_SENSORS           8   // number of sensors
#define BIAS_SPEED           400 // robot bias speed
#define MAXSPEED             800 // maximum robot speed
#define SENSOR_NORMALIZATION 100 // Normalisation constant for the IR sensors measurements
#define OBSTACLE_THRESHOLD   100 // IR sensor value from which we consider there is an obstalce

#endif
