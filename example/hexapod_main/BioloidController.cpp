/*
  BioloidController.cpp - ArbotiX Library for Bioloid Pose Engine
  Copyright (c) 2008-2012 Michael E. Ferguson.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Modified by Jeff Yu January 2015 to use the Linux SDK functions
*/

#include "BioloidController.h"
#include <stdio.h>

  void PrintCommStatus(int CommStatus);
  void PrintErrorCode(void);

/* initializes serial1 transmit at baud, 8-N-1 */
  BioloidController::BioloidController(long baud){
    int i;
    // setup storage
    id_ = (unsigned char *) malloc(AX12_MAX_SERVOS * sizeof(unsigned char));
    pose_ = (unsigned int *) malloc(AX12_MAX_SERVOS * sizeof(unsigned int));
    nextpose_ = (unsigned int *) malloc(AX12_MAX_SERVOS * sizeof(unsigned int));
    speed_ = (int *) malloc(AX12_MAX_SERVOS * sizeof(int));
    // initialize
    printf("Initialize Poses\n");
    for(i=0;i<AX12_MAX_SERVOS;i++){
        id_[i] = i+1;
        pose_[i] = 512;
        printf("pose_[%d] : %d\n",id_[i],pose_[i]);
        nextpose_[i] = 512;
    }
    interpolating = 0;
    playing = 0;
    //lastframe_ = millis(); //Replace millis() with [(clock()-start)*1000/CLOCKS_PER_SEC]
    lastframe_ = (int) (clock()-start)*1000/CLOCKS_PER_SEC;
    //ax12Init(baud);
    dxl_initialize(0,baud);
}

/* new-style setup */
void BioloidController::setup(int servo_cnt){
    int i;
    // setup storage
    id_ = (unsigned char *) malloc(servo_cnt * sizeof(unsigned char));
    pose_ = (unsigned int *) malloc(servo_cnt * sizeof(unsigned int));
    nextpose_ = (unsigned int *) malloc(servo_cnt * sizeof(unsigned int));
    speed_ = (int *) malloc(servo_cnt * sizeof(int));
    // initialize
    poseSize = servo_cnt;
    for(i=0;i<poseSize;i++){
        id_[i] = i+1;
        pose_[i] = 512;
        nextpose_[i] = 512;
    }
    interpolating = 0;
    playing = 0;
    lastframe_ = (int) (clock()-start)*1000/CLOCKS_PER_SEC;
}
void BioloidController::setId(int index, int id){
    id_[index] = id;
}
int BioloidController::getId(int index){
    return id_[index];
}

/* load a named pose from FLASH into nextpose. (Don't think you necessarily need this unless you have saved poses) */
/*
void BioloidController::loadPose( const unsigned int * addr ){
    int i;
    poseSize = pgm_read_word_near(addr); // number of servos in this pose
    for(i=0; i<poseSize; i++)
        nextpose_[i] = pgm_read_word_near(addr+1+i) << BIOLOID_SHIFT;
}
*/
/* read in current servo positions to the pose. */
// New readPose function
void BioloidController::readPose(){
    printf("Reading Pose\n");
    for(int i=0;i<poseSize;i++){
        pose_[i] = dxl_read_word(id_[i],P_PRESENT_POSITION_L) << BIOLOID_SHIFT;
        printf("pose_[%d] : %d\n",id_[i],(pose_[i]>>BIOLOID_SHIFT));
        while(pose_[i] == 0) {    // 
            pose_[i] = dxl_read_word(id_[i],P_PRESENT_POSITION_L) << BIOLOID_SHIFT;
            printf("pose_[%d] : %d\n",id_[i],pose_[i]);
        }
        usleep(25000);
    }
}
/*
void BioloidController::readPose(){
    for(int i=0;i<poseSize;i++){
        pose_[i] = ax12GetRegister(id_[i],AX_PRESENT_POSITION_L,2)<<BIOLOID_SHIFT;
        //delay(25);   //Replace delay with usleep()
        usleep(25000); 
    }
}
*/
/* write pose out to servos using sync write. */
// New writePose using Syncwrite example
void BioloidController::writePose(){
    // Make syncwrite packet
    int temp;
    int CommStatus;
    dxl_set_txpacket_id(BROADCAST_ID);
    dxl_set_txpacket_instruction(INST_SYNC_WRITE);
    dxl_set_txpacket_parameter(0, P_GOAL_POSITION_L);
    dxl_set_txpacket_parameter(1, 2);
    //printf("Iteration setup\n");
    for( int i=0; i<poseSize; i++ )
    {
        temp = (pose_[i] >> BIOLOID_SHIFT);
        dxl_set_txpacket_parameter(2+3*i, id_[i]);
        dxl_set_txpacket_parameter(2+3*i+1, dxl_get_lowbyte(temp));
        dxl_set_txpacket_parameter(2+3*i+2, dxl_get_highbyte(temp));
        //printf("ID%d, Position %d; ",id_[i],temp);
    }
    dxl_set_txpacket_length((2+1)*poseSize+4);
    //printf( "\n\n" );

    dxl_txrx_packet();
    CommStatus = dxl_get_result();
    if( CommStatus == COMM_RXSUCCESS )
    {
        PrintErrorCode();
    }
    else
    {
        PrintCommStatus(CommStatus);
    }
}
/*
void BioloidController::writePose(){
    int temp;
    int length = 4 + (poseSize * 3);   // 3 = id + pos(2byte)
    int checksum = 254 + length + INST_SYNC_WRITE + 2 + P_GOAL_POSITION_L;
    setTXall();
    ax12write(0xFF);
    ax12write(0xFF);
    ax12write(0xFE);
    ax12write(length);
    ax12write(INST_SYNC_WRITE);
    ax12write(P_GOAL_POSITION_L);
    ax12write(2);
    for(int i=0; i<poseSize; i++)
    {
        temp = pose_[i] >> BIOLOID_SHIFT;
        checksum += (temp&0xff) + (temp>>8) + id_[i];
        ax12write(id_[i]);
        ax12write(temp&0xff);
        ax12write(temp>>8);
    } 
    ax12write(0xff - (checksum % 256));
    setRX(0);
}
*/
/* set up for an interpolation from pose to nextpose over TIME 
    milliseconds by setting servo speeds. */
void BioloidController::interpolateSetup(int time){
    int i;
    int frames = (time/BIOLOID_FRAME_LENGTH) + 1;
    lastframe_ = (int) (clock()-start)*1000/CLOCKS_PER_SEC;
    // set speed each servo...
    for(i=0;i<poseSize;i++){
        if(nextpose_[i] > pose_[i]){
            speed_[i] = (nextpose_[i] - pose_[i])/frames + 1;
        }else{
            speed_[i] = (pose_[i]-nextpose_[i])/frames + 1;
        }
    }
    interpolating = 1;
}
/* interpolate our pose, this should be called at about 30Hz. */
void BioloidController::interpolateStep(){
    if(interpolating == 0) return;
    int i;
    int complete = poseSize;
    while(((int) (clock()-start)*1000/CLOCKS_PER_SEC) - lastframe_ < BIOLOID_FRAME_LENGTH);
    lastframe_ = (int) (clock()-start)*1000/CLOCKS_PER_SEC;
    // update each servo
    for(i=0;i<poseSize;i++){
        int diff = nextpose_[i] - pose_[i];
        if(diff == 0){
            complete--;
        }else{
            if(diff > 0){
                if(diff < speed_[i]){
                    pose_[i] = nextpose_[i];
                    complete--;
                }else
                pose_[i] += speed_[i];
            }else{
                if((-diff) < speed_[i]){
                    pose_[i] = nextpose_[i];
                    complete--;
                }else
                pose_[i] -= speed_[i];                
            }       
        }
    }
    if(complete <= 0) interpolating = 0;
    writePose();      
}

/* get a servo value in the current pose */
int BioloidController::getCurPose(int id){
    for(int i=0; i<poseSize; i++){
        if( id_[i] == id )
            return ((pose_[i]) >> BIOLOID_SHIFT);
    }
    return -1;
}
/* get a servo value in the next pose */
int BioloidController::getNextPose(int id){
    for(int i=0; i<poseSize; i++){
        if( id_[i] == id )
            return ((nextpose_[i]) >> BIOLOID_SHIFT);
    }
    return -1;
}
/* set a servo value in the next pose */
void BioloidController::setNextPose(int id, int pos){
    for(int i=0; i<poseSize; i++){
        if( id_[i] == id ){
            nextpose_[i] = (pos << BIOLOID_SHIFT);
            return;
        }
    }
}

/* play a sequence.*/
/*
void BioloidController::playSeq( const transition_t  * addr ){
    sequence = (transition_t *) addr;
    // number of transitions left to load
    transitions = pgm_read_word_near(&sequence->time);
    sequence++;    
    // load a transition
    loadPose((const unsigned int *)pgm_read_word_near(&sequence->pose));
    interpolateSetup(pgm_read_word_near(&sequence->time));
    transitions--;
    playing = 1;
}
*/
/* keep playing our sequence */
/*
void BioloidController::play(){
    if(playing == 0) return;
    if(interpolating > 0){
        interpolateStep();
    }else{  // move onto next pose
        sequence++;   
        if(transitions > 0){
            loadPose((const unsigned int *)pgm_read_word_near(&sequence->pose));
            interpolateSetup(pgm_read_word_near(&sequence->time));
            transitions--;
        }else{
            playing = 0;
        }
    }
}
*/

// Some error checking functions from the dynamixel SyncWrite examples
// Print communication result
void PrintCommStatus(int CommStatus)
{
    switch(CommStatus)
    {
        case COMM_TXFAIL:
        printf("COMM_TXFAIL: Failed transmit instruction packet!\n");
        break;
        case COMM_TXERROR:
        printf("COMM_TXERROR: Incorrect instruction packet!\n");
        break;
        case COMM_RXFAIL:
        printf("COMM_RXFAIL: Failed get status packet from device!\n");
        break;
        case COMM_RXWAITING:
        printf("COMM_RXWAITING: Now recieving status packet!\n");
        break;
        case COMM_RXTIMEOUT:
        printf("COMM_RXTIMEOUT: There is no status packet!\n");
        break;
        case COMM_RXCORRUPT:
        printf("COMM_RXCORRUPT: Incorrect status packet!\n");
        break;
        default:
        printf("This is unknown error code!\n");
        break;
    }
}

// Print error bit of status packet
void PrintErrorCode()
{
    if(dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
        printf("Input voltage error!\n");
    if(dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
        printf("Angle limit error!\n");
    if(dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
        printf("Overheat error!\n");
    if(dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
        printf("Out of range error!\n");
    if(dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
        printf("Checksum error!\n");
    if(dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
        printf("Overload error!\n");
    if(dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
        printf("Instruction code error!\n");
}