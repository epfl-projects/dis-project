#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include <time.h>
#include <math.h>
#include <assert.h>
//#include <sys/time.h>
#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

#include "../defines.h"

/** Robot devices */
WbDeviceTag sensors[NB_SENSORS];
WbDeviceTag emitterTag;
WbDeviceTag receiverTag;

/**
 * A unique id from 001 to NUM_ROBOTS (included), allowing leading zeros
 */
const char *robotName;
/**
 * Current number of neighbors of this robot
 * TODO: is it needed for the Alpha algorithm?
 */
int nNeighbors;

/**
 * For sensor readings
 * Values are bigger when an obstacle is closer (non linear increase)
 */
int distances[NB_SENSORS];
/** For obstacle avoidance **/
int speed[2];

/* ****************************** BEHAVIORS ***************************** */
typedef enum {FORWARD, FORWARD_AVOIDANCE, COHERENCE, COHERENCE_AVOIDANCE} State;
State currentState;
/** Time left remaining in state COHERENCE */
int coherenceTime = 0;
/** Time left remaining in sub-state AVOIDANCE */
int avoidanceTime = 0;

void setState(State newState) {
  if(currentState != newState) {
    switch(newState) {
      case FORWARD_AVOIDANCE:
      case COHERENCE_AVOIDANCE:
        avoidanceTime = MAX_AVOIDANCE_TIME;
        break;

      case FORWARD:
        // We may have failed or succeeded a coherence state
        coherenceTime = 0;
        avoidanceTime = 0;
        break;

      case COHERENCE:
        // TODO: check this makes sense, in particular in the case ov
        // transitions COHERENCE <=> COHERENCE_AVOIDANCE
        coherenceTime = MAX_COHERENCE_TIME;
        avoidanceTime = 0;
        break;
    }
    currentState = newState;
  }
}


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
 * Obstacle avoidance (rule-based)
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
  // printf("Robot %s is starting a turn of %d degrees\n", robotName, angle);
  turningTime = (int)(50 * angle / 360.f);
  isTurning = true;
}
/**
 * Assuming a turn is in progress
 * @param speed [OUTPUT] Array of 2 ints. The corresponding speed for each wheel
 */
void turn(int * speed) {
  // TODO: fine-tune
  speed[0] = -450;
  speed[1] = 450;
  //printf("Robot %s has %d turning time left\n", robotName, turningTime);
  turningTime--;
  if(turningTime <= 0) {
    isTurning = false;
    // printf("Robot %s is done turning.\n", robotName);
  }
}

void move() {
  if(isTurning)
    turn(speed);
  else if(currentState == FORWARD_AVOIDANCE || currentState == COHERENCE_AVOIDANCE) {
    avoidObstacle(speed, distances);
    setSpeed(speed[0], speed[1]);
  }
  else {
    // Just move forward
    setSpeed(BIAS_SPEED, BIAS_SPEED);
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

/* **************************** LOGGING **************************** */
void sendStateToSupervisor() {
  // TODO: need perfect infinite communication range!

  // Change channel temporarily to communicate with the supervisor
  wb_emitter_set_channel(emitterTag, COMMUNICATION_CHANNEL_STAT);
  // Message format: robot name [space] state [space] nNeighbors
  char message[100];
  sprintf(message, "%s %d %d", robotName, currentState, nNeighbors);
  sendMessage(message);
  // Back to the inter-robot communication channel
  wb_emitter_set_channel(emitterTag, COMMUNICATION_CHANNEL);
}

/* ****************************** RUN ****************************** */
/**
 * Update the state depending on time spent in the current state,
 * the number of neighbors and the presence of obstacles.
 */
void alphaAlgorithm() {
  nNeighbors = countNeighbors();

  if(currentState == FORWARD || currentState == FORWARD_AVOIDANCE) {
    // Lost the swarm --> jump to coherence state
    if(nNeighbors < ALPHA) {
      // Lost the swarm: make a 180° turn
      initiateTurn(180);

      setState(COHERENCE);
      // printf("Robot %s is turning back (%d neighbors).\n", robotName, nNeighbors);
    }
    // Obstacle avoidance timed out --> go back to forward state
    else if(currentState == FORWARD_AVOIDANCE) {
      avoidanceTime--;
      if(avoidanceTime <= 0) {
        setState(FORWARD);
      }
    }
    // Encountered an obstacle --> jump to obstacle avoidance
    else if(hasObstacle()) {
      setState(FORWARD_AVOIDANCE);
    }
    // Otherwise, persist in forward state
  }
  else if(currentState == COHERENCE || currentState == COHERENCE_AVOIDANCE) {
    // Successful coherence --> pick a random orientation and jump to state forward
    if(nNeighbors >= ALPHA) {
      // Pick a new random orientation
      initiateTurn(rand() % MAX_RANDOM_TURN);
      setState(FORWARD);
      // printf("Robot %s found back %d neighbors :)\n", robotName, nNeighbors);
    }
    else {
      if(currentState == FORWARD_AVOIDANCE) {
        avoidanceTime--;
        // Obstacle avoidance timed out --> go back to forward state
        if(avoidanceTime <= 0) {
          setState(FORWARD);
        }
      }

      // Regardless of avoidance time, the coherence time counter keeps running
      coherenceTime--;
      // Failed coherence --> go back to state FORWARD
      if(coherenceTime <= 0) {
        setState(FORWARD);
        // printf("Robot %s failed coherence :(\n", robotName);
      }
    }
  }
}

long long previousSecond = -1;
void run(){
  // Movement
  move();

  // Periodically check the current value of alpha
  // TODO: make rate limiting consistent with the article
  double t = wb_robot_get_time();
  long long second = (long long)t;
  if(second != previousSecond) {
    broadcastMyId();

    // Leave time for first broadcasts to arrive
    if(second > 1)
      alphaAlgorithm();

    previousSecond = second;

    // Communicate current state (for performance measures)
    sendStateToSupervisor();
  }

}

/* ****************************** RESET ****************************** */

void reset()
{
  int i;
  robotName = wb_robot_get_name();
  currentState = FORWARD;

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
