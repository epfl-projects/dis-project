#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include <time.h>
#include <math.h>
#include <assert.h>
#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

#include "../defines.h"


// Robot devices
WbDeviceTag sensors[NB_SENSORS];
WbDeviceTag emitterTag;
WbDeviceTag receiverTag;

// A unique id from 001 to NUM_ROBOTS (included), allowing leading zeros
const char *robotName;

// for sensor readings (bigger when an obstacle is closer)
int distances[NB_SENSORS];
// for obstacle avoidance
int speed[2];

// neighborhood information
int k = 0; // current number of neighbors
int LastK = 0;


State currentState;
bool AVOIDANCE = false;
int coherenceTime = 0; // Time left remaining in state COHERENCE
int avoidanceTime = 0; // Time left remaining in sub-state AVOIDANCE
bool isTurning = false;
int turningTime = 0; // Time left to turn in number of TIME_STEP
bool react = false; // reaction indicator


void setSpeed(int leftSpeed, int rightSpeed) {
  if (leftSpeed < -MAXSPEED) {leftSpeed = -MAXSPEED;}
  if (leftSpeed >  MAXSPEED) {leftSpeed =  MAXSPEED;}
  if (rightSpeed < -MAXSPEED) {rightSpeed = -MAXSPEED;}
  if (rightSpeed >  MAXSPEED) {rightSpeed =  MAXSPEED;}

  wb_differential_wheels_set_speed(leftSpeed, rightSpeed);
}


/**
 * Turn by `angle` degrees in a second or less.
 * The speeds are chosen so that a 360° turns takes 1 second.
 * @param angle The rotation angle between 0 and 360 degrees.
 */
void initiateTurn(int angle) {
  if (angle == 180)
    turningTime = 1000 / TIME_STEP; // this many time steps
  else
    turningTime = (rand() % 10); // [0 .. 9] time steps
  isTurning = true;
}


/**
 * Assuming a turn is in progress
 * @param speed [OUTPUT] Array of 2 ints. The corresponding speed for each wheel
 */
void turn(int * speed) {
  // TODO: fine-tune
  speed[0] = -490;
  speed[1] = 490;
  //printf("Robot %s has %d turning time left\n", robotName, turningTime);
  turningTime--;
  if(turningTime <= 0) {
    isTurning = false;
    // printf("Robot %s is done turning.\n", robotName);
  }
}

void setState(State newState) {
  if(currentState != newState) {
    switch(newState) {

      case FORWARD:
        // FORWARD takes precedence over AVOIDANCE
        if (AVOIDANCE) {
          AVOIDANCE = false;
        }
        // Pick a new random orientation
        initiateTurn(-1);
        break;

      case COHERENCE:
        // COHERENCE takes precedence over AVOIDANCE
        if (AVOIDANCE) {
          AVOIDANCE = false;
        }
        initiateTurn(180);
        coherenceTime = MAX_COHERENCE_TIME;
        break;
    }
    currentState = newState;
  }
}


//*******************************************************
//      MOVEMENT
//*******************************************************


/**
 * Detect the presence of an obstacle in front based on sensor readings.
 * The sensor values must be updated first with `getSensorValues`.
 */
bool hasObstacle() {
  int maxValue = 0;
  for(unsigned int i = 0; i < NB_SENSORS; i++) {
    // Note that `distances` holds sensor readings
    maxValue = fmax(maxValue, distances[i]);
  }
  return (maxValue >= OBSTACLE_THRESHOLD);
}



/**
 * Obstacle avoidance Breitenberg
 */
void avoidObstacle(int * speed, const int* distances){
  int brait_coef[8][2] = {
    {140, -35}, {110, -15}, {80,-10},{-10, -10},
    {-15, -10}, {-5, 80}, {-30, 90}, {-20, 160}
  };

  unsigned int i, j;
  for(i = 0; i < 2; i++){
    speed[i] = BIAS_SPEED / 2;
    for (j = 0; j < 8; j++){
      speed[i] += brait_coef[j][i] * (1.0 - (distances[j] / SENSOR_NORMALIZATION));
    }
  }
}



void getSensorValues(int *sensorTable) {
  unsigned int i;
  for(i = 0; i < NB_SENSORS; i++) {
    sensorTable[i] = wb_distance_sensor_get_value(sensors[i]);
  }
  if (hasObstacle()) { // if has obstacle
    if (!AVOIDANCE) { // if not already in avoidance
      AVOIDANCE = true;
      avoidanceTime = MAX_AVOIDANCE_TIME;
    }
  } else {
    AVOIDANCE = false;
  }
}


void move() {
  if(isTurning){
    turn(speed);
    setSpeed(speed[0], speed[1]);
  }
  else if(AVOIDANCE) {
    avoidObstacle(speed, distances);
    setSpeed(speed[0], speed[1]);
  }
  else {
    // Just move forward
    setSpeed(BIAS_SPEED, BIAS_SPEED);
  }
}



//*******************************************************
//      COMMUNICATION
//*******************************************************


void sendMessage(char const * message) {
  wb_emitter_send(emitterTag, message, strlen(message) + 1);
}


void broadcast() {
  // Setup inter-robot, imperfect communication
  wb_emitter_set_channel(emitterTag, COMMUNICATION_CHANNEL);
  wb_emitter_set_range(emitterTag, COMM_RADIUS);

  sendMessage(robotName);
}


/**
 * From the received broadcasts, deduce the number of nearby agents and their neighborhood information
 * Each robot is expected to broadcast its `robotName` and also `robotName` of its neighbors, which is expected
 * to be parseable as a positive number in 1..NUM_ROBOTS.
 */
void listen() {
  // Reset counters
  LastK = k;
  k = 0;

  while(wb_receiver_get_queue_length(receiverTag) > 0) {
    char * neighborName = (char *)wb_receiver_get_data(receiverTag);
    int id = (int)strtol(neighborName, NULL, 10);
    k++;
    //printf("Robot %s received: %d present!\n", robotName, id);
    wb_receiver_next_packet(receiverTag);
  }

  assert(k <= NUM_ROBOTS - 1);
  // printf("Robot %s has %d neighbors\n", robotName, k);
}


void sendStateToSupervisor() {
  // Change channel temporarily to communicate with the supervisor
  wb_emitter_set_channel(emitterTag, COMMUNICATION_CHANNEL_STAT);
  // Allow infinite communication range
  wb_emitter_set_range(emitterTag, -1);

  // Message format: robot name [space] state [space] avoidance(0 or 1) [space] nNeighbors
  char message[100];

  // since only two states FORWARD and COHERENCE are effectively used in addition to the boolean variable AVOIDANCE, adapt states
  sprintf(message, "%s %d %d", robotName, AVOIDANCE ? currentState + 1 : currentState, k);
  sendMessage(message);
}



//*******************************************************
//      RUN
//*******************************************************


void run(){

  static int time_step_counter = 0;
  time_step_counter++;

  // Move
  move();

  // keep the clock ticking is AVOIDANCE or COHERENCE
  if (AVOIDANCE) {
    avoidanceTime--;
    if(avoidanceTime <= 0) {
      setState(FORWARD);
    }
  }

  if (currentState == COHERENCE) { // keep coherence clock ticking
    coherenceTime--;
    if (coherenceTime <= 0)
      setState(FORWARD);
  }

  // Rate limiting
  if(time_step_counter >= 20) {
    // printf("robot %s is broadcasting\n", robotName);
    broadcast();
    listen(); // ! going to listen to pings from previous step (50 time step in the past)


    if (k < LastK && k < ALPHA) { // if less neighbors than last time and below threshold
      // printf("Robot %s has %d neighbors!\n", robotName, k);
      // printf("robot %s is moving to coherence state\n", robotName);
      setState(COHERENCE);
    }
    else if (k > LastK) {
      // printf("Robot %s has %d neighbors!\n", robotName, k);
      // printf("robot %s is moving to forward state\n", robotName);
      setState(FORWARD); // make random turn
    }


    time_step_counter = 0; // reset time step counter

  }
  // Send logging info to the supervisor
  // It needs to be done in a different timestep, otherwise
  // the channel & range switching come in conflict
  else if(time_step_counter == 10 && LOG_EXPERIMENT) {
    sendStateToSupervisor();
  }
}




//*******************************************************
//      RESET
//*******************************************************

void reset()
{
  int i;
  robotName = wb_robot_get_name();

  currentState = FORWARD;
  k = 0;
  LastK = 0;

  // Make sure to initialize the RNG with different values for each thread
  srand(time(NULL) + (int)&robotName);

  char e_puck_name[] = "ps0";
  char sensorsName[5];

  // Emitter and receiver device tags
  emitterTag = wb_robot_get_device("emitter");
  receiverTag = wb_robot_get_device("receiver");

  // Configure communication devices
  wb_receiver_enable(receiverTag, TIME_STEP);
  wb_emitter_set_range(emitterTag, COMM_RADIUS);
  wb_emitter_set_channel(emitterTag, COMMUNICATION_CHANNEL);
  wb_receiver_set_channel(receiverTag, COMMUNICATION_CHANNEL);

  sprintf(sensorsName, "%s", e_puck_name);
  for (i = 0; i < NB_SENSORS; i++) {
    sensors[i] = wb_robot_get_device(sensorsName);
    wb_distance_sensor_enable(sensors[i], TIME_STEP);

    if ((i + 1) >= 10) {
      sensorsName[2] = '1';
      sensorsName[3]++;

      if ((i + 1) == 10) {
        sensorsName[3] = '0';
        sensorsName[4] = (char) '\0';
        }
    } else {
        sensorsName[2]++;
    }
  }

  wb_differential_wheels_enable_encoders(TIME_STEP);

  printf("Robot %s is reset\n", robotName);
  return;
}



//*******************************************************
//      MAIN
//*******************************************************

int main(int argc, char *argv[]) {
  /* initialize Webots */
  wb_robot_init();

  reset();

  /* main loop */
  for (;;) {
    getSensorValues(distances);
    run();

    /* perform a simulation step */
    wb_robot_step(TIME_STEP);
  }

  return 0;
}
