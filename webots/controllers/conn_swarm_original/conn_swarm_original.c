#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include <time.h>
//#include <sys/time.h>
#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

// Global Defines

#define TIME_STEP         64

// AUXILIARY
#define NB_SENSORS        8   // number of sensors
#define BIAS_SPEED        400 // robot bias speed
#define MAXSPEED          800 // maximum robot speed
#define COMM_RADIUS       0.7 // radius of radio communication
#define RANGE		  100 // normalisation of the IR sensors for obstacle avoidance
/*
  **
  **  Auxiliary
  **
  **
*/
//robot variables
WbDeviceTag sensors[NB_SENSORS];
WbDeviceTag emitterir;
WbDeviceTag receiverir;
const char *robot_name;

int distances[NB_SENSORS];    // for sensor readings
int speed[2];		      // for obstacle avoidance

void setSpeed(int LeftSpeed, int RightSpeed) {
  if (LeftSpeed < -MAXSPEED) {LeftSpeed = -MAXSPEED;}
  if (LeftSpeed >  MAXSPEED) {LeftSpeed =  MAXSPEED;}
  if (RightSpeed < -MAXSPEED) {RightSpeed = -MAXSPEED;}
  if (RightSpeed >  MAXSPEED) {RightSpeed =  MAXSPEED;}


  wb_differential_wheels_set_speed(LeftSpeed,RightSpeed);

}


void getSensorValues(int *sensorTable) {
  unsigned int i;
  for (i=0; i < NB_SENSORS; i++) {
    sensorTable[i] = wb_distance_sensor_get_value(sensors[i]);
  }
}


void sendMessage() {
  // TODO
  char message[128];
  sprintf(message, "hello my name is %s", robot_name);
  wb_emitter_send(emitterir, message, strlen(message) + 1);
}

void receiveMessage() {
  while(wb_receiver_get_queue_length(receiverir) > 0) {
    //int size = wb_receiver_get_data_size(receiverir);
    char * contents = (char *)wb_receiver_get_data(receiverir);
    printf("Robot %s received: %s", robot_name, contents);
  }
}

/*
  **
  **  E-Puck
  **
  **
*/
void move(){

  setSpeed(BIAS_SPEED,BIAS_SPEED);
}
/* ****************************** RESET ****************************** */

void reset()
{
  int i;
  robot_name = wb_robot_get_name();

  char e_puck_name[] = "ps0";
  char sensors_name[5];

  // Emitter and receiver device tags
  // TODO
  //emitterir = wb_robot_get_device("EPUCK_EMITTER");
  //receiverir = wb_robot_get_device("EPUCK_RECEIVER");

  // Update communication radius to COMM_RADIUS
  //wb_emitter_set_range(emitterir, COMM_RADIUS);

  sprintf(sensors_name, "%s", e_puck_name);
  for (i = 0; i < NB_SENSORS; i++) {
    sensors[i] = wb_robot_get_device(sensors_name);
    wb_distance_sensor_enable(sensors[i], TIME_STEP);

    if ((i + 1) >= 10) {
      sensors_name[2] = '1';
      sensors_name[3]++;

      if ((i + 1) == 10) {
        sensors_name[3] = '0';
        sensors_name[4] = (char) '\0';
        }
    } else {
        sensors_name[2]++;
    }
  }

  wb_differential_wheels_enable_encoders(TIME_STEP);

  printf("Robot %s is reset\n",robot_name);
  return;
}


/* ****************************** RUN ****************************** */

void run(){

  move();
}


/***************OBSTACLE AVOIDANCE (Braitenberg)*********************/

void avoid_obstacle(int* speed, const int* ds_value){

  int brait_coef[8][2] =
  {{140, -35}, {110, -15}, {80,-10},{-10, -10},
     {-15, -10}, {-5, 80}, {-30, 90}, {-20, 160} };

  unsigned int i, j;
  for (i = 0; i < 2; i++){
    speed[i] = BIAS_SPEED / 2;
    for (j = 0; j < 8; j++){
      speed[i] += brait_coef[j][i] * (1.0 - (ds_value[j] / RANGE));
    }
  }
}


/******************************** MAIN ******************************/

int main(int argc, char *argv[]) {

  /* initialize Webots */
  wb_robot_init();

  reset();

  sendMessage();
  receiveMessage();

  /* main loop */
  for (;;) {

    getSensorValues(distances);
    avoid_obstacle(speed,distances);
    setSpeed(speed[0], speed[1]);
    //run();

    // TODO: communication
    // sendMessage();
    // receiveMessage();

    /* perform a simulation step */
    wb_robot_step(TIME_STEP);

  }

  return 0;
}
