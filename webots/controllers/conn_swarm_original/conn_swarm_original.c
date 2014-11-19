#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include <time.h>
#include <assert.h>
//#include <sys/time.h>
#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

#include "../defines.h"

// ALPHA ALGORITHM
#define ALPHA             2 // Minimum number of neighbors

// AUXILIARY
#define NB_SENSORS        8   // number of sensors
#define BIAS_SPEED        400 // robot bias speed
#define MAXSPEED          800 // maximum robot speed
#define RANGE		  100 // normalisation of the IR sensors for obstacle avoidance

//robot variables
WbDeviceTag sensors[NB_SENSORS];
WbDeviceTag emitterTag;
WbDeviceTag receiverTag;
/**
 * A unique id from 001 to NUM_ROBOTS (included), allowing leading zeros
 */
const char *robotName;

int distances[NB_SENSORS];    // for sensor readings
int speed[2];		              // for obstacle avoidance

/* ****************************** MOVEMENT ****************************** */
void setSpeed(int leftSpeed, int rightSpeed) {
  if (leftSpeed < -MAXSPEED) {leftSpeed = -MAXSPEED;}
  if (leftSpeed >  MAXSPEED) {leftSpeed =  MAXSPEED;}
  if (rightSpeed < -MAXSPEED) {rightSpeed = -MAXSPEED;}
  if (rightSpeed >  MAXSPEED) {rightSpeed =  MAXSPEED;}

  wb_differential_wheels_set_speed(leftSpeed, rightSpeed);
}

void getSensorValues(int *sensorTable) {
  unsigned int i;
  for(i = 0; i < NB_SENSORS; i++) {
    sensorTable[i] = wb_distance_sensor_get_value(sensors[i]);
  }
}

/**
 * Obstacle avoidance (Braitenberg controller)
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
      speed[i] += brait_coef[j][i] * (1.0 - (distances[j] / RANGE));
    }
  }
}

/**
 * Turn by `angle` degrees in a second or less.
 * The speeds are chosen so that a 360° turns takes 1 second.
 * @param angle The rotation angle between 0 and 360 degrees.
 */
bool isTurning = false;
/** Time left to turn in number of TIME_STEP */
int turningTime = 0;
void initiateTurn(int angle) {
  angle = (angle % 360);
  printf("Robot %s is starting a turn of %d degrees\n", robotName, angle);
  turningTime = (int)(50 * angle / 360.f);
  isTurning = true;
}
/**
 * Assuming a turn is in progress
 * @param speed [OUTPUT] The corresponding speed for each wheel
 */
void turn(int * speed) {
  // TODO: fine-tune
  speed[0] = -450;
  speed[1] = 450;
  //printf("Robot %s has %d turning time left\n", robotName, turningTime);
  turningTime--;
  if(turningTime <= 0) {
    isTurning = false;
    printf("Robot %s is done turning.\n", robotName);
  }
}

void move() {
  if(isTurning)
    turn(speed);
  else
    avoidObstacle(speed, distances);
  setSpeed(speed[0], speed[1]);
}

/* ****************************** BEHAVIORS ***************************** */
typedef enum {RANDOM, SEARCH_SWARM} State;
State currentState;

void setState(State newState) {
  if(currentState != newState) {
    switch(newState) {
      // Pick a new random orientation
      case RANDOM:
        initiateTurn(rand() % 180 + 90);
        break;

      // Make a 180° turn
      case SEARCH_SWARM:
        initiateTurn(180);
        break;
    }
    currentState = newState;
  }
}

/* ************************* COMMUNICATION ****************************** */
void sendMessage(char const * message) {
  wb_emitter_send(emitterTag, message, strlen(message) + 1);
  // printf("Robot %s tried to send message (status %d, 0 means fail)\n", robotName, status);
}
void broadcastMyId() {
  sendMessage(robotName);
}
/**
 * From the received broadcasts, deduce the number of nearby agents.
 * Each robot is expected to broadcast its `robotName`, which is expected
 * to be parsable as a positive number in 1..NUM_ROBOTS.
 */
bool isPresent[NUM_ROBOTS+1];
int countNeighbors() {
  // Resent counters
  for(int i = 1; i <= NUM_ROBOTS; ++i)
    isPresent[i] = false;
  int n = 0;

  while(wb_receiver_get_queue_length(receiverTag) > 0) {
    char * neighborName = (char *)wb_receiver_get_data(receiverTag);
    int id = (int)strtol(neighborName, NULL, 10);
    isPresent[id] = true;
    //printf("Robot %s received: %d present!\n", robotName, id);

    wb_receiver_next_packet(receiverTag);
  }

  for(int i = 1; i <= NUM_ROBOTS; ++i) {
    if(isPresent[i])
      n++;
  }

  assert(n <= NUM_ROBOTS - 1);
  return n;
}

/* ****************************** RUN ****************************** */
/**
 * @return Whether or not the adjacency criterion is met
 */
void alphaAlgorithm() {
  int n = countNeighbors();

  // Lost the swarm
  if(currentState == RANDOM && n < ALPHA) {
    setState(SEARCH_SWARM);
    printf("Robot %s is turning back (%d neighbors).\n", robotName, n);
  }
  // Found the swarm back
  else if(currentState == SEARCH_SWARM && n >= ALPHA) {
    setState(RANDOM);
    printf("Robot %s found back %d neighbors :)\n", robotName, n);
  }
}

long long previousSecond = -1;
void run(){
  // Movement
  move();

  // Periodically check the current value of alpha
  // TODO: better rate limiting mechanism
  double t = wb_robot_get_time();
  long long second = (long long)t;
  if(second != previousSecond) {
    broadcastMyId();

    // Leave time for first broadcasts to arrive
    if(second > 1)
      alphaAlgorithm();

    previousSecond = second;
  }
}

/* ****************************** RESET ****************************** */

void reset()
{
  int i;
  robotName = wb_robot_get_name();
  currentState = RANDOM;

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


/******************************** MAIN ******************************/

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
