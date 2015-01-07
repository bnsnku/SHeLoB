/* 	Main code for hexapod (SHeLoB)
	Jeff Yu, Dan Sun, Xuan Lin
	*/

//#include <ax12.h>
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
// Initialize the clock so we can take time readings between each movement. (Arduino uses millis())
  clock_t start;
  int control;

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
  	// setup serial
  	//Serial.begin(38400);
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
    //printf("Pose size set.\n");
    bioloid.readPose();
    printf("Pose Read\n");
    doIK();
    printf("inverse kinematics done.\n");
    bioloid.interpolateSetup(1000);
    printf("Interpolate Setup done.\n");
    while(bioloid.interpolating > 0){
      bioloid.interpolateStep();
    //delay(3); Probably don't need for linux machine.
      usleep(3000); //microseconds
    }
    multiplier = RIPPLE_SPEED;

	// Repeatedly call this function
    control = 1;
    int c;
    c = system("/bin/stty raw");
    while(1) {
    //for(int i = 0;i<100;i++) {
      //printf("Iteration %d\n",i);
      if(control == 1){
        loop();
      }
      else {
        c = system("/bin/stty cooked");
        printf("Terminal set to %d\n",c);
        break;
      }
    }
    dxl_terminate();
  }

  void loop(){
  // take commands 
  int key;
  key = getchar();
  switch(key)
  {
    case 'w':
    Xspeed = 80;
    break;
    case 's':
    Xspeed = -80;
    break;
    case 'd':
    Yspeed = 80;
    break;
    case 'a':
    Yspeed = -80;
    break;
    case 'z':
    gaitSelect(RIPPLE_SMOOTH);
    multiplier = RIPPLE_SPEED;
    break;
    case 'x':
    gaitSelect(AMBLE_SMOOTH);
    multiplier = AMBLE_SPEED;
    break;
    case 'c':
    gaitSelect(RIPPLE);
    multiplier = RIPPLE_SPEED;
    break;
    case 'v':
    gaitSelect(AMBLE);
    multiplier = AMBLE_SPEED;
    break;
    case 'b':
    gaitSelect(TRIPOD);
    multiplier = TRIPOD_SPEED;
    break;
    case 'q':
    control = 0;
    return;
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
  if(bioloid.interpolating == 0){
    doIK();
    //printf("Inverse Kinetmatics Done for Iteration %d\n",iter);
    bioloid.interpolateSetup(tranTime);
  }
  // update joints
  bioloid.interpolateStep();
}
