/* 	Main code for hexapod (SHeLoB)
	Jeff Yu, Dan Sun, Xuan Lin
  December 2014
	*/

#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <math.h>
#include <termio.h>
#include <dynamixel.h>
#include "BioloidController.h"
//#include <Commander.h>
#include "nuke.h"

// Define one or the other depending upon which servo type you are using.
#define AX12_HEXAPOD
//#define AX18_HEXAPOD

//Commander command = Commander();
  int multiplier;
  clock_t start;  // Initialize the clock so we can take time readings between each movement. (Arduino uses millis())
  int control;
  extern ik_req_t gaits[];

#define RIPPLE_SPEED    1
#define AMBLE_SPEED     3
#define TRIPOD_SPEED    5

#ifdef AX12_HEXAPOD
#define TOP_SPEED      10
#endif

#ifdef AX18_HEXAPOD
#define TOP_SPEED      12
#endif

  void loop();

  int main() {
	// setup IK
  	setupIK();
    //printf("Setup complete.\n");
  	gaitSelect(RIPPLE_SMOOTH);
     //Grab a clock reading
    start = clock();
  	/*
  	// wait, then check the voltage (LiPO safety)
  	delay (1000);
  	float voltage = (ax12GetRegister (1, AX_PRESENT_VOLTAGE, 1)) / 10.0;
  	Serial.print ("System Voltage: ");
  	Serial.print (voltage);
  	Serial.println (" volts.");
  	if (voltage < 10.0)
    	while(1);
	*/
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
    printf("Setup Complete\nUse w, a, s, d to control direction\nq to quit\n");
    multiplier = RIPPLE_SPEED;

	// Repeatedly call this function
    control = 1;
    int c;
    // Set terminal to take continuous input without having to press enter each time
    c = system("/bin/stty raw");
    while(1) {
      if(control == 1){
        loop();
      }
      else {
        // Return terminal to standard input
        c = system("/bin/stty cooked");
        printf("Terminal set to %d\n",c);
        break;
      }
    }
    dxl_terminate();
  }

  void loop(){
    int stepcount = 0;
    int flag = 0;  //add by Xuan
  // take commands 
    int key;
    key = getchar();
    switch(key)
    {
      case 'w':
      Xspeed = 50;
      Yspeed = 0;
      Rspeed = 0;
      break;
      case 's':
      Xspeed = -50;
      Yspeed = 0;
      Rspeed = 0;
      break;
      case 'd':
      Xspeed = 0;
      Yspeed = 50;
      Rspeed = 0;
      break;
      case 'a':
      Xspeed = 0;
      Yspeed = -50;
      Rspeed = 0;
      break;
      case 'z':
      gaitSelect(RIPPLE_SMOOTH);
      multiplier = RIPPLE_SPEED;
      flag = 1;
      break;
      case 'x':
      gaitSelect(AMBLE_SMOOTH);
      multiplier = AMBLE_SPEED;
      flag = 1;
      break;
      case 'c':
      gaitSelect(RIPPLE);
      multiplier = RIPPLE_SPEED;
      flag = 1;
      break;
      case 'v':
      gaitSelect(AMBLE);
      multiplier = AMBLE_SPEED;
      flag = 1;
      break;
      case 'b':
      gaitSelect(TRIPOD);
      multiplier = TRIPOD_SPEED;
      flag = 1;
      break;
      case 'q':
      control = 0;
      return;
      break;
      case 'o':     //add by Xuan
      Xspeed = 0;
      Yspeed = 0;
      Rspeed = 0;
      flag = 2;
      break;

      case 'p':
      gaitSelect(BACK);
      flag = 0;
      break;

      default:
      break;
    }
	/*
  if(command.ReadMsgs() > 0){
    digitalWrite(0,HIGH-digitalRead(0));
    // select gaits
    if(command.buttons&BUT_R1){ 
      gaitSelect(RIPPLE_SMOOTH); 
      multiplier=RIPPLE_SPEED;
    }
    if(command.buttons&BUT_R2){ 
      gaitSelect(AMBLE_SMOOTH); 
      multiplier=AMBLE_SPEED;
    }
    if(command.buttons&BUT_R3){ 
      gaitSelect(RIPPLE); 
      multiplier=RIPPLE_SPEED;
    }
    if(command.buttons&BUT_L4){ 
      gaitSelect(AMBLE); 
      multiplier=AMBLE_SPEED;
    }
    if(command.buttons&BUT_L5){ 
      gaitSelect(TRIPOD); 
      multiplier=TRIPOD_SPEED;
    }
    if(command.buttons&BUT_L6){ 
      gaitSelect(TRIPOD); 
      multiplier=TOP_SPEED;
    } 
    // set movement speed
    if((command.walkV) > 40 || (command.walkV < -5) ){
      Xspeed = (multiplier*command.walkV)/2;
    }
    else
    {
      Xspeed = 0;
    }
    
    if((command.walkH) > 40 || (command.walkH < -35) ){   
    Yspeed = (multiplier*command.walkH)/2;
    }
    else
    {
     Yspeed = 0;
    }
    
    if((command.lookH) > 40 || (command.lookH < -40) ){
    Rspeed = -(command.lookH)/100.0;
    }
    else
    {
      Rspeed = 0;
    }
  */
  //Xspeed = 200; //CHANGE THIS
  //Yspeed = 0;
  //Rspeed = 0;
  // if our previous interpolation is complete, recompute the IK
    //while(stepcount != (stepsInCycle)) {
  if (flag == 0)
    {
      if(bioloid.interpolating == 0){
      doIK();
      stepcount++;
    //printf("Inverse Kinetmatics Done for Iteration %d\n",iter);
      bioloid.interpolateSetup(tranTime);
    }
  // update joints
    bioloid.interpolateStep();
    }
  else if (flag == 1)
  {  
  }
  else if(flag == 2)    //add by Xuan
  {
    if(bioloid.interpolating == 0){
      doIK_Z0();
      stepcount++;
    //printf("Inverse Kinetmatics Done for Iteration %d\n",iter);
      bioloid.interpolateSetup(tranTime);
    }
  // update joints
    bioloid.interpolateStep();
  }

  }
  //}
