#ifndef CONN_SWARM_DEFINES
#define CONN_SWARM_DEFINES

// Simulation parameters
#define EXP_TIME          900 // Duration of the experiment (in seconds)
#define TIME_STEP         64 // Simulation time ste
#define NUM_ROBOTS        10 // Total number of robots in the scene
// Alpha algorithm
#define ALPHA             7 // Minimum number of neighbors to maintain
// Communication
#define COMM_RADIUS       0.7 // Range of radio communication (in meters)
#define COMMUNICATION_CHANNEL 1

// Initial agents distribution parameters
#define INITIAL_BOX_SIDE  30
#define NEIGHBOURHOOD     5

// AUXILIARY
#define NB_SENSORS        8   // number of sensors
#define BIAS_SPEED        400 // robot bias speed
#define MAXSPEED          800 // maximum robot speed
#define RANGE     100 // Normalisation constant for the IR sensors measurements

#endif
