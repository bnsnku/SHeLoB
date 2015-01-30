/* 	Leg code for hexapod (SHeLoB)
	Jeff Yu, Dan Sun, Xuan Lin, Alexie Pogue
  	December 2014
 */

#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <math.h>
#include <termio.h>
#include <dynamixel.h>
#include "BioloidController.h"
#include "nuke.h"

// Define one or the other depending upon which servo type you are using.
#define AX12_HEXAPOD
//#define AX18_HEXAPOD

  clock_t start;  // Initialize the clock so we can take time readings between each movement. (Arduino uses millis())

int main(){
	// setup IK
  	setupIK();
    // printf("Setup complete.\n");
  	gaitSelect(RIPPLE_SMOOTH);
    // Grab a clock reading
    start = clock();
    
 	// stand up slowly
    bioloid.poseSize = 18;
    bioloid.readPose();
    //printf("Pose Read\n");
    doIK();
    bioloid.interpolateSetup(1000);
    while(bioloid.interpolating > 0){
      bioloid.interpolateStep();
      usleep(10000); // Delay 10 milliseconds between each move to slow down the motion
    }
    printf("Setup Complete\n");
    /*
    // Store the goal position
	int xpos, ypos, zpos;
	int check;
	printf("Please input an X Y Z position\n");
	check = scanf("%d %d %d",&xpos,&ypos,&zpos);
	if (check < 3) {
		printf("Invalid input. Please input three integers separaated by spaces.\n");
	}
	else {
		printf("Goal position set to (%d,%d,%d)\n", xpos, ypos, zpos);
	}
    // Set pose for one leg and write the position
    singleIK(xpos,ypos,zpos);
    bioloid.readPose();
    doIK();
    bioloid.interpolateSetup(1000);
    while(bioloid.interpolating > 0){
      bioloid.interpolateStep();
      usleep(10); // Delay 10 milliseconds between each move to slow down the motion
    }
	*/

	// Define positions for a circle
    int radius = 20;
    int slope = 1;
    int xpos, ypos, zpos;
    double theta = 0.2;
    for(int t = 0;t<100;t++){
		//xpos = (int) radius*cos(theta*t);
		//ypos = (int) radius*sin(theta*t);
		xpos = (int) radius/sqrt(slope*slope+1)*cos(theta*t);
		ypos = slope * xpos;
		zpos = (int) radius*sin(theta*t);
		//printf("(%d,%d,%d)\n",xpos,ypos,zpos);
		singleIK(xpos,ypos,zpos);
		doIK();
   		bioloid.interpolateSetup(99);
    	while(bioloid.interpolating > 0){
      	bioloid.interpolateStep();
      	//usleep(10); // Delay 10 milliseconds between each move to slow down the motion
    	}
	}

	dxl_terminate();
}