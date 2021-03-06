#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include <time.h>
#include <math.h>
#include <assert.h>
#include <limits.h>
#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/led.h> 
#include "../defines.h" 



//******************************************************* 
//      ROBOT VARIABLES
//*******************************************************

WbDeviceTag sensors[NB_SENSORS];
WbDeviceTag light_sensors[NB_SENSORS]; // Device variables for light sensors
WbDeviceTag emitterTag;
WbDeviceTag receiverTag;
WbDeviceTag led;

const char *robotName; // A unique id from 001 to NUM_ROBOTS (included), allowing leading zeros

int distances[NB_SENSORS]; // for sensor readings (bigger when an obstacle is closer)
int light_intensity[NB_SENSORS]; // light sensor readings
int speed[2];              // for obstacle avoidance


//beta algorithm parameters
int k; // current number of neighbors 
int Nlist[NUM_ROBOTS+1][NUM_ROBOTS+1]; // list of neighbors
int LastK; // past number of neighbors
int OldList[NUM_ROBOTS+1][NUM_ROBOTS+1]; // old list of neighbors



State currentState;
bool AVOIDANCE = false;
int coherenceTime = 0; // Time left remaining in state COHERENCE 
int avoidanceTime = 0; // Time left remaining in sub-state AVOIDANCE
bool isTurning = false;
int turningTime = 0; // Time left to turn in number of TIME_STEP 
bool react = false; // reaction indicator 

char neighborhood[500]; //first id is the robot's own id, subsequent ids are the neighbors'




//******************************************************* 
//      MOVEMENT
//*******************************************************

void setSpeed(int leftSpeed, int rightSpeed) {
  if (leftSpeed < -MAXSPEED) {leftSpeed = -MAXSPEED;}
  if (leftSpeed >  MAXSPEED) {leftSpeed =  MAXSPEED;}
  if (rightSpeed < -MAXSPEED) {rightSpeed = -MAXSPEED;}
  if (rightSpeed >  MAXSPEED) {rightSpeed =  MAXSPEED;}

  wb_differential_wheels_set_speed(leftSpeed, rightSpeed);
}



// Obstacle avoidance Breitenberg
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

  
 // Assuming a turn is in progress
 // @param speed [OUTPUT] Array of 2 ints. The corresponding speed for each wheel
void turn(int * speed) {
  speed[0] = -500;
  speed[1] = 500;
  turningTime--;
  if(turningTime <= 0) {
    isTurning = false;
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




// Turn by `angle` degrees in a second or less.
// The speeds are chosen so that a 360° turns takes 1 second.
// @param angle The rotation angle between 0 and 360 degrees.
void initiateTurn(int angle) {
  if (angle == 180)
    turningTime = 1000 / TIME_STEP; // this many time steps
  else 
    turningTime = (rand() % 10); // [0 .. 9] time steps   
  isTurning = true;
}





void setState(State newState) {
  if(currentState != newState) { // if a new state
    switch(newState) {

      case FORWARD:
        if (AVOIDANCE)
          AVOIDANCE = false; // FORWARD, COHERENCE take precedence over AVOIDANCE
        // Pick a new random orientation
        initiateTurn(-1);
        break;

      case COHERENCE:
        if (AVOIDANCE) 
          AVOIDANCE = false; // FORWARD, COHERENCE take precedence over AVOIDANCE
        initiateTurn(180);
        coherenceTime = MAX_COHERENCE_TIME;
        break;
    }
    currentState = newState;
  }
}



//******************************************************* 
//      SENSOR READINGS
//*******************************************************


// Detect the presence of an obstacle in front based on sensor readings.
// The sensor values must be updated first with `getSensorValues`.
bool hasObstacle() {
  int maxValue = 0;
  for(unsigned int i = 0; i < NB_SENSORS; i++) {
    // Note that `distances` holds sensor readings
    maxValue = fmax(maxValue, distances[i]);
  }
  return (maxValue >= OBSTACLE_THRESHOLD);
}



bool isIlluminated() {
  int minValue = INT_MAX;
  for(unsigned int i = 0; i < NB_SENSORS; i++) {
    //`distances` holds sensor readings
    minValue = fmin(minValue, light_intensity[i]);
  }
  return (minValue < LIGHT_THRESHOLD);
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


void lightGetSensorValues(int *sensorTable)
{
  unsigned int i;
  for (i=0; i < NB_SENSORS; i++) {
    sensorTable[i] = wb_light_sensor_get_value(light_sensors[i]);
  }
  if (isIlluminated()) { // if a robot is illuminated turn on the LEDS
    wb_led_set(led, 1); 
  } else 
    wb_led_set(led, 0); // else switch of the LEDS (just for debugging purposes)
}





//******************************************************* 
//      COMMUNICATIONS
//*******************************************************


void sendMessage(char const * message) {
  wb_emitter_send(emitterTag, message, strlen(message) + 1);
}


void broadcast() {
  sendMessage(neighborhood); // broadcast neighborhood information
}


// fills up array with id of neighbors, first element is the robot_id of the robot sending this message, second number tells if the robot is illuminated
// returns number of elements in the array
int parseMessage(char* message, int* neighbors) {
  char* token = strtok(message, " ");
  int i = 0;
  while (token) {
    neighbors[i++] = atoi(token);
    token = strtok(NULL, " ");
  }
  return i;
}



// From the received broadcasts, deduce the number of nearby agents and their neighborhood information
// Each robot is expected to broadcast its `robotName` and also `robotName` of its neighbors, which is expected
// to be parseable as a positive number in 1..NUM_ROBOTS.
void listen() {

  // copy previous neighborhood info
  LastK = k;
  for (int i = 0; i <= NUM_ROBOTS; i++) 
    for (int j = 0; j <= NUM_ROBOTS; j++)
      OldList[i][j] = Nlist[i][j];

  // clear Nlist
  k = 0;
  for (int i = 0; i <= NUM_ROBOTS; i++) 
    for (int j = 0; j <= NUM_ROBOTS; j++)
      Nlist[i][j] = 0;

  
  while(wb_receiver_get_queue_length(receiverTag) > 0) {
    char * message = (char *)wb_receiver_get_data(receiverTag);

    int neighbors[NUM_ROBOTS+2];
    int n = parseMessage(message, neighbors);

    if (neighbors[1]) // if robot illuminated
      Nlist[neighbors[0]][0] = 2;
    else // if not illuminated
      Nlist[neighbors[0]][0] = 1;

    for(int i = 2; i < n; i++) {
      Nlist[neighbors[0]][neighbors[i]] = 1;
    }
    k++;
    wb_receiver_next_packet(receiverTag);
  }

  char temp[500];
  sprintf(neighborhood, "%s %d", robotName, isIlluminated() ? 1 : 0);
  for (int i = 1; i <= NUM_ROBOTS; i++) {
    if (Nlist[i][0]){ // go through the first column
      sprintf(temp, "%s 00%d", neighborhood, i);
      strcpy(neighborhood, temp);
    }
  }
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

  if (currentState == COHERENCE) {
    coherenceTime--; // keep coherence clock ticking 
    if (coherenceTime <= 0)
      setState(FORWARD);
  }

  if(time_step_counter >= 20) { // every 20 time step
    broadcast();
    listen(); // ! going to listen to pings from previous step (100 time step in the past)

    // create lost list
    int lostList[NUM_ROBOTS+1];
    for (int i = 0; i<=NUM_ROBOTS; i++) // clear lost list
      lostList[i] = 0;
    for (int i = 1; i<=NUM_ROBOTS; i++) {
      if (OldList[i][0] && !Nlist[i][0]) { // robot lost
        lostList[i] = OldList[i][0]; // 1 or 2 depending on whether the lost robot was illuminated or not
      }
    }


    // for each robot in lost list:
    //  check if reaction is triggered
    bool Back = false;
    for (int i = 0; i<=NUM_ROBOTS; i++) {
      if (lostList[i] == 1) { // if robot is lost and not illuminated
        int nShared = 0; // number of shared connections
        // go through all the current neighbors and see if they have this lost robot covered
        for (int j = 1; j<=NUM_ROBOTS; j++) {
          if (Nlist[j][0] == 1) {
            if (Nlist[j][i])
              nShared++;
          }
        }

        if (nShared <= BETA) {
          Back = true;
        }
      } else if (lostList[i] == 2) { // if robot is lost and illuminated treat in a special way
        Back = true;
      }
    }


    if (Back) {
      setState(COHERENCE);
    }
    else if (k > LastK) {
      setState(FORWARD); // make random turn
    }

    time_step_counter = 0; // reset time step counter 
  }
 
}




//******************************************************* 
//      RESET
//*******************************************************

void reset()
{
  int i;
  robotName = wb_robot_get_name();

  // reset sensor readings
  for (i=0; i < NB_SENSORS; i++) {
    distances[i] = 0;
    light_intensity[i] = 0;
  }

  currentState = FORWARD;
  sprintf(neighborhood, "%s %d", robotName, isIlluminated() ? 1 : 0); // initially no neighborhood info to broadcast


  k = 0;
  for (int i = 0; i <= NUM_ROBOTS; i++) 
    for (int j = 0; j <= NUM_ROBOTS; j++)
      Nlist[i][j] = 0;

  // Make sure to initialize the RNG with different values for each thread
  srand(time(NULL) + (int)&robotName);

  char e_puck_name[] = "ps0";
  char sensorsName[5];

  char light_name[] = "ls0";
  char light_sensors_name[5];

  // Emitter and receiver device tags
  emitterTag = wb_robot_get_device("emitter");
  receiverTag = wb_robot_get_device("receiver");
  led = wb_robot_get_device("led0");

  // Configure communication devices
  wb_receiver_enable(receiverTag, TIME_STEP);
  wb_emitter_set_range(emitterTag, COMM_RADIUS);
  wb_emitter_set_channel(emitterTag, COMMUNICATION_CHANNEL);
  wb_receiver_set_channel(receiverTag, COMMUNICATION_CHANNEL);

  sprintf(sensorsName, "%s", e_puck_name);
  sprintf(light_sensors_name, "%s", light_name);


  // Initialize proximity and light sensors
  for (i = 0; i < NB_SENSORS; i++) {
    sensors[i] = wb_robot_get_device(sensorsName);
    wb_distance_sensor_enable(sensors[i], TIME_STEP);
    light_sensors[i] = wb_robot_get_device(light_sensors_name);
    wb_light_sensor_enable(light_sensors[i], TIME_STEP);

    if ((i + 1) >= 10) {
      sensorsName[2] = '1';
      sensorsName[3]++;
      light_sensors_name[2] = '1';
      light_sensors_name[3]++;

      if ((i + 1) == 10) {
        sensorsName[3] = '0';
        sensorsName[4] = (char) '\0';
        light_sensors_name[3] = '0';
        light_sensors_name[4] = (char) '\0';
        }
    } else {
        sensorsName[2]++;
        light_sensors_name[2]++;
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
  // initialize Webots
  wb_robot_init();

  reset();

  // main loop 
  for (;;) {
    getSensorValues(distances); 
    lightGetSensorValues(light_intensity);
    run();

    // perform a simulation step
    wb_robot_step(TIME_STEP);
  }

  return 0;
}
