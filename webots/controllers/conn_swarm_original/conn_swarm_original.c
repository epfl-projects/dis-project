 
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
#define NB_SENSORS        8		  // number of sensors
#define BIAS_SPEED        400		// robot bias speed
#define MAXSPEED          800		// maximum robot speed
#define COMM_RADIUS       0.7

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

void setSpeed(int LeftSpeed, int RightSpeed)
{
	if (LeftSpeed < -MAXSPEED) {LeftSpeed = -MAXSPEED;}
	if (LeftSpeed >  MAXSPEED) {LeftSpeed =  MAXSPEED;}
	if (RightSpeed < -MAXSPEED) {RightSpeed = -MAXSPEED;}
	if (RightSpeed >  MAXSPEED) {RightSpeed =  MAXSPEED;}
  
  wb_differential_wheels_set_speed(LeftSpeed,RightSpeed);

}

void getSensorValues(int *sensorTable)
{
	unsigned int i;
	for (i=0; i < NB_SENSORS; i++) {
		sensorTable[i] = wb_distance_sensor_get_value(sensors[i]);
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

int main(int argc, char *argv[]) {

  /* initialize Webots */
  wb_robot_init();
  
  reset();
  
  /* main loop */
  for (;;) {
  
  run();
  
  /* perform a simulation step */
  wb_robot_step(TIME_STEP);
  
  }
  

  return 0;
}
