#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/supervisor.h>
#include <webots/receiver.h>

#include "../defines.h"

WbNodeRef epucks[NUM_ROBOTS];
WbFieldRef locfield[NUM_ROBOTS];

float finalTime;

void waitaux(long num) {
	long i;
	for(i=0;i<num;i++);
}

void change_robot_positions(){

  int i,j;
  int nooverlap;

  WbFieldRef rotfield;

  int location[NUM_ROBOTS][2];
  int rotation;
  double newlocation[3]={0.0, 0.0, 0.0};
  double newrotation[4]={0.0, 1.0, 0.0, 0.0};

  for(i=0;i<NUM_ROBOTS;i++){

    nooverlap=0;
    while(nooverlap==0){

      location[i][0]=rand()%(2*INITIAL_BOX_SIDE)-INITIAL_BOX_SIDE;
      location[i][1]=rand()%(2*INITIAL_BOX_SIDE)-INITIAL_BOX_SIDE;

      for(j=0;j<i;j++){
        if(location[i][0]<location[j][0]+NEIGHBOURHOOD && location[i][0]>location[j][0]-NEIGHBOURHOOD)
          if(location[i][1]<location[j][1]+NEIGHBOURHOOD && location[i][1]>location[j][1]-NEIGHBOURHOOD)
            break;
      }
      if(j==i)
        nooverlap=1;
    }

    newlocation[0]=((double)location[i][0])/100;
    newlocation[1]=0.0;
    newlocation[2]=((double)location[i][1])/100;

    wb_supervisor_field_set_sf_vec3f(locfield[i],newlocation);

    rotation=rand()%100;
    newrotation[3]=(double)rotation;
    //newrotation[3]=0.0;

    rotfield=wb_supervisor_node_get_field(epucks[i],"rotation");
    wb_supervisor_field_set_sf_rotation(rotfield,newrotation);
  }
}

void reset(void) {
  int i;
  char stringaux[20];

  for(i=0;i<NUM_ROBOTS;i++){
    sprintf(stringaux,"E_PUCK_%d",i+1);
    epucks[i]=wb_supervisor_node_get_from_def(stringaux);
    locfield[i]=wb_supervisor_node_get_field(epucks[i],"translation");
  }

  srand(time(NULL));
  change_robot_positions();

  finalTime = wb_robot_get_time() + EXP_TIME; // 15 MIN == 900
}

void run() {

  if(wb_robot_get_time() > finalTime){

    wb_supervisor_simulation_revert();
  }

}


int main(int argc, char *argv[]) {

  /* initialize Webots */
  wb_robot_init();

  reset();

  /* perform a simulation step */
  wb_robot_step(TIME_STEP);

  /* main loop */
  for (;;) {
    run();

    /* perform a simulation step */
    wb_robot_step(TIME_STEP);

  }

  return 0;
}

