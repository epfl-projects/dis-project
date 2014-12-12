#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/supervisor.h>
#include <webots/receiver.h>
#include "../utils.h"
#include "../defines.h"

WbDeviceTag receiverTag;


char filename_metrics[255];


// beacon and its location
WbNodeRef epucks[NUM_ROBOTS], beacon;
WbFieldRef locfield[NUM_ROBOTS];
WbFieldRef beacon_location;
double beacon_coord[2];

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


  // Configure receiver device
  receiverTag = wb_robot_get_device("receiver");
  wb_receiver_enable(receiverTag, TIME_STEP);
  wb_receiver_set_channel(receiverTag, COMMUNICATION_CHANNEL_STAT);
  // wb_receiver_set_channel(receiverTag, COMMUNICATION_CHANNEL_DEBUG);



  // get beacon
  beacon=wb_supervisor_node_get_from_def("BEACON1");
  beacon_location=wb_supervisor_node_get_field(beacon, "translation");

  double* location = wb_supervisor_field_get_sf_vec3f(beacon_location);
  beacon_coord[0] = location[0];
  beacon_coord[1] = location[1];



  sprintf(filename_metrics, "%s/metrics.csv", LOG_FILES_FOLDER);
  FILE * logFile = fopen(filename_metrics, "w+");
  fprintf(logFile, "Time, area_coverage, distance_to_beacon\n");
  fclose(logFile);
}

/* **************************** LOGGING **************************** */
int nReceived = 0;
int robotsStates[NUM_ROBOTS];
int robotsNeighborsCount[NUM_ROBOTS];
void resetLogs() {
  nReceived = 0;
  for(int i = 0; i < NUM_ROBOTS; ++i) {
    robotsStates[i] = -1;
    robotsNeighborsCount[i] = -1;
  }
}

/**
 * Stats file logging format: CSV
 * Filename: simulation-[number of robots]-.csv
 */
char filename[255];
void createNewLogFile() {
  resetLogs();

  time_t currentTime;
  currentTime = time(NULL);

  sprintf(filename, "%s/simulation-%d-%ld.csv", LOG_FILES_FOLDER, NUM_ROBOTS, currentTime);

  FILE * logFile = fopen(filename, "w+");
  // Specify CSV columns' title
  fprintf(logFile, "Time, Robot ID, Robot state, Number of neighbors\n");
  fclose(logFile);
}
void writeStats() {
  FILE * logFile = fopen(filename, "a+");
  for(int i = 0; i < NUM_ROBOTS; ++i) {
    fprintf(logFile, "%f, %d, %d, %d\n", wb_robot_get_time(), i, robotsStates[i], robotsNeighborsCount[i]);
  }
  fprintf(logFile, "\n");
  fclose(logFile);
}

/**
 * Each time period, receive and aggregate stats from the robots.
 * Write them out to a file.
 */
void receiveRobotsStates() {
  while(wb_receiver_get_queue_length(receiverTag) > 0) {
    // Message format: see robot's controller
    char * stats = (char *)wb_receiver_get_data(receiverTag);

    // Position of the next separator
    int separatorPosition = 0;

    // Parse robot index from its name
    int robotId = (int)strtol(stats, NULL, 10);
    // Map the ID to an index
    robotId--;

    // Count each robot only once
    if(robotsStates[robotId] == -1) {
      nReceived++;
    }


    // Parse state
    while(stats[separatorPosition] != ' ')
      separatorPosition++;
    robotsStates[robotId] = (int)strtol(stats + separatorPosition, NULL, 10);
    separatorPosition++;

    // Parse number of neighbors
    while(stats[separatorPosition] != ' ')
      separatorPosition++;
    robotsNeighborsCount[robotId] = (int)strtol(stats + separatorPosition, NULL, 10);

    // printf("%d %d %d\n", robotId, robotsStates[robotId], robotsNeighborsCount[robotId]);

    wb_receiver_next_packet(receiverTag);
  }

  // Done receiving all robots states for this timestep
  if(nReceived == NUM_ROBOTS) {
    writeStats();
    // printf("Received %d messages at time %f\n", nReceived, wb_robot_get_time());

    resetLogs();
  }
}


// void getDebugMessages() {
//   while(wb_receiver_get_queue_length(receiverTag) > 0) {
//     char * illuminatedRobot = (char *)wb_receiver_get_data(receiverTag);
//     int robot = (int)strtol(illuminatedRobot, NULL, 10);
//     WbFieldRef pos;
//     pos = wb_supervisor_node_get_field(epucks[robot-1],"translation");
//     wb_supervisor_set_label(0,"RED",pos[0],pos[1],0.1,0xff0000,0);
//     wb_receiver_next_packet(receiverTag);
//   }

// }


void computeMetrics() {
  Point vertices[NUM_ROBOTS];
  Point convex_hull[NUM_ROBOTS];
  double* location;
  for(int i = 0; i < NUM_ROBOTS; i++){
    location = wb_supervisor_field_get_sf_vec3f(locfield[i]);
    // get only x and y
    vertices[i].x = location[0];
    vertices[i].y = location[1];
  }
  int n = convexHull(vertices, NUM_ROBOTS, convex_hull);
  // get area coverage
  double area_coverage = PolygonArea(convex_hull, n);
  // get centroid 
  Point centeroid = getCentroid(convex_hull, n);
  // get beacon's location
  

  double distance = sqrt(((centeroid.x - beacon_coord[0]) * (centeroid.x - beacon_coord[0])) + ((centeroid.y - beacon_coord[1]) * (centeroid.y - beacon_coord[1])));

  // log to file
  FILE * logFile = fopen(filename_metrics, "a");
  fprintf(logFile, "%f, %lf, %lf\n", wb_robot_get_time(), area_coverage*1000, distance);
  fclose(logFile);
  
}

/* **************************** RUN ******************************* */
void run() {
  static int timestep_counter = 0;
  // End of the experiment
  if(wb_robot_get_time() > finalTime){
    wb_supervisor_simulation_revert();
  }


  if (timestep_counter >= 200) {
    computeMetrics();
    timestep_counter = 0;
  }

  timestep_counter++;
  


  // receiveRobotsStates();
  // getDebugMessages();
}


int main(int argc, char *argv[]) {
  /* initialize Webots */
  wb_robot_init();
  createNewLogFile();

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

