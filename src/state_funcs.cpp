/*
* @file: state_funcs.cpp
* @author: B. de Wit <b.a.g.d.wit@student.tue.nl> & EVC Group 1 2016 (L. van Harten)
* @course: Embedded Visual Control 5LIA0
*
* This is the FSM main file. All choices are made here. The Arduino commands
* are sent in this file. This file also contains debug output to the 
* console.
*/


#include "state_funcs.h"
using namespace std;
VisionToolkit *VisionInfo;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*add straight to sendqueue function																							//
  this function creates a string which will be added to a queue to send over to the Arduino                    //
  format used = [id][instr][left sign][steps left][right sign][steps right][time], example 0DRI+123-456789    */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void straightFunc(int distance, state_data *data)
{
    int servoDistance = (distance * REV_TO_DIST);
    cout<<"--- subcase: added instruction: " << data->inst_cnt << " = go straight to sendqueue" << endl;

    data->all_packets[data->inst_cnt].id = '0' + data->inst_cnt;

    data->all_packets[data->inst_cnt].instr_type[0] = 'D';
    data->all_packets[data->inst_cnt].instr_type[1] = 'R';
    data->all_packets[data->inst_cnt].instr_type[2] = 'I';
    data->all_packets[data->inst_cnt].instr_type[3] = 0;
    data->all_packets[data->inst_cnt].ldir = '+';
    data->all_packets[data->inst_cnt].rdir = '+';
    data->all_packets[data->inst_cnt].lspd = servoDistance;
    data->all_packets[data->inst_cnt].rspd = servoDistance;
    data->all_packets[data->inst_cnt].duration = 0;
    data->inst_cnt ++;

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*add turn to sendqueue function																							//
  this function creates a string which will be added to a queue to send over to the Arduino                    //
  format used = [id][instr][left sign][steps left][right sign][steps right][time], example 0DRI+123-456789    */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void angleFunc(int radius, state_data *data)
{
    int servoDistance = (radius * REV_TO_ANGLE);
    servoDistance = abs(servoDistance);
    cout<<"--- subcase: added instruction: " << data->inst_cnt << " = turn: " << radius << " degrees to sendqueue \n";

    data->all_packets[data->inst_cnt].id = '0' + data->inst_cnt;

    data->all_packets[data->inst_cnt].instr_type[0] = 'D';
    data->all_packets[data->inst_cnt].instr_type[1] = 'R';
    data->all_packets[data->inst_cnt].instr_type[2] = 'I';
    data->all_packets[data->inst_cnt].instr_type[3] = 0;

    if(radius >0){
        data->all_packets[data->inst_cnt].ldir = '+';
        data->all_packets[data->inst_cnt].rdir = '-';
    }
    else{
        data->all_packets[data->inst_cnt].ldir = '-';
        data->all_packets[data->inst_cnt].rdir = '+';
    }
    data->all_packets[data->inst_cnt].lspd = servoDistance;
    data->all_packets[data->inst_cnt].rspd = servoDistance;
    data->all_packets[data->inst_cnt].duration = 0;
    data->inst_cnt++;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*Ultrasound sensor check 																						//
//Function which is not being used right now, can be used to check the difference between a shadow and a line  //
//If you want to use this function you should get a HC-SR04 sensor module and connect this to the Arduino     */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool ultraCheck(int distance, state_data *data){
//    cout << "ULTRACHECK\n";
//
//    cout<<"FLUSHING for ultra \n";
//
    bool block = true;
//    int length = 4;
//    char flushCOM[4];
//    flushCOM[0]= 128;
//    flushCOM[1]= 'F';
//    flushCOM[2]= 'L';
//    flushCOM[3]= 'U';
//    if (write(data->file_i2c, flushCOM, length) != length){      //write() returns the number of bytes actually written, if it doesn't match then an error occurred (e.g. no response from the device)
//       /* ERROR HANDLING: i2c transaction failed */
//        printf("--- subcase: Failed to write to the i2c bus flush ultra.\n");
//    }
//
//    char ultraCOM[5];
//    ultraCOM[0]= 128;
//    ultraCOM[1]= 'O';
//    ultraCOM[2]= 'B';
//    ultraCOM[3]= 'J';
//    ultraCOM[4] = distance;
//    length = 5;
//    /*send request to check on ultra and store result*/
//    if (write(data->file_i2c, ultraCOM, length) != length){      //write() returns the number of bytes actually written, if it doesn't match then an error occurred (e.g. no response from the device)
//       /* ERROR HANDLING: i2c transaction failed */
//        printf("--- subcase: Failed to write to the i2c bus ultra.\n");
//    }
//    usleep(10000);
//    if (read(data->file_i2c, ultraCOM, length) != length)       //read() returns the number of bytes actually read, if it doesn't match then an error occurred (e.g. no response from the device)
//    {
//        //ERROR HANDLING: i2c transaction failed
//        printf("--- subcase: Failed to read from the i2c bus.\n");
//        //allAcks=false;
//    }
//    else
//    {
//        cout << "Read result from ultrasound for distance: " << distance << "answer = ";
//        printf("%d ", ultraCOM[0]);
//        printf("\n");
//   }
//
//    if(ultraCOM[0] == 1)
//    {
//        block = true;
//    }
    return block;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Function which isn't used currently. This is used to translate pixels in the y direction to a distance.          ///
//To get this function print out a grid, place this under your camera and compare pixels to grid distance.        ///
//This should end up having a curve which can be translated to a polynomial and then used to calculate distance. ///
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int tl_y_to_dist(int tl_y)
{
    int dist = tl_y*tl_y* 0.0131 + tl_y * -0.2081 + 19.5494;
    return dist;
}

///////////////////////////
/* Initialization State */
/////////////////////////
global_state init(state_data *data) {

	
    cout << "============================================================" << endl;
    cout << "____________________________________________________________" << endl;
    cout << "...FSM:INIT \n" << endl;
    VisionInfo = new VisionToolkit();
    for(int i=0; i<MAX_PACKETS_IN_AVR; i++) {
        data->all_packets[i].id = 'a'+i;
        data->all_packets[i].in_use = false;
    }

    //////////////////////////////////
    //DO NOT TOUCH THIS IS I2C SETUP//
    //////////////////////////////////
    //----- OPEN THE I2C BUS -----

    char *filename = (char*)"/dev/i2c-1";
    if ((data->file_i2c = open(filename, O_RDWR)) < 0)
    {
        //ERROR HANDLING: you can check errno to see what went wrong
        printf("ERROR: Failed to open the i2c bus at startup");
        exit(10);
    }
    data->addr = 0x08;          //<<<<<The I2C address of the slave
    if (ioctl(data->file_i2c, I2C_SLAVE, data->addr) < 0)
    {
        printf("ERROR: Failed to acquire bus access and/or talk to slave at startup.\n");
        //ERROR HANDLING; you can check errno to see what went wrong
        exit(11);
    }

	//set some values to zero as init
    data->blue_sign.type = none;
	data->blueSignFlag = none;
    data->redSignFlag = false;
    data->redSignFlagEx = false;
	data->whereToGo = 0;
	data->whereToGoCount = 0;
	data->lastTurnFlag = none2;
	data->lastTurnFlag_other = none2;
	
    return PROCESSINGIMG;
}

/////////////////////////////
/* Processing Image State */
///////////////////////////
global_state processingImage(state_data *data) {
	#ifdef WAITKEY
		cout << "============================================================" << endl;
		cout << "waiting for key input" << endl;
		int c = false;
		while(!c){c = getchar();}
		cout << "============================================================" << endl;
	#endif //WAITKEY
	//cout<< "&&& start of processing IMG" << endl;
    usleep(300000);
    // global variable noAck2_count = 0;
    cout << "____________________________________________________________" << endl;
    cout << "... FSM:PROCESSING" << endl;
    VisionInfo->process_frame();
    data->red_sign = VisionInfo->red_sign;
    data->blue_sign = VisionInfo->blue_sign;
    data->yellow_sign = VisionInfo->yellow_sign;
    data->cur_lane = VisionInfo->current_lane;

	//reset the ack counter
    data->noAck2_count = 0;

	//variables which are not being used at the moment
    data->cur_lane.w = 0;
    data->cur_lane.h = 0;
    data->cur_lane.size = 0;

	//blue sign flag to remember a sign which was seen last time
    if(data->blue_sign.type != none){
        data->blueSignFlag = data->blue_sign.type;
    }

    cout << "PROCESSIMG current redSignflag = " << data->redSignFlag << " and red_sign.type = " << data->red_sign.type << "\n";
    if(data->red_sign.type == stopsign){
        data->redSignFlag = true;
    }
	//cout<< "&&& end of processing IMG" << endl;
    return CREATEQUEUE;
}

///////////////////////
/*Create Queue State*/
/////////////////////
global_state createQueue(state_data *data){
    cout << "____________________________________________________________" << endl;
    cout<<"... FSM:CREATING \n";
	
	cout << "/////////FLAGS before processing//////////////////////////////////////////////////////////////////" << endl;
	cout << "0 = lturn, 1 = fturn, 2 = rturn, 3 = stopsign, 4 = uturn, 5 = none" << endl;
	cout << "the blue sign flag currently is set at " << data->blueSignFlag << endl;
	cout << "the red  sign flag currently is set at " << data->redSignFlag<< endl;
	cout << "the blue sign registered by the vision part " << data->blue_sign.type << endl;
	cout << "the yellow sign registered by the vision part " << data->yellow_sign.type << endl;
	cout << "the red  sign registered by the vision part "<< data->red_sign.type  <<endl <<endl;
	cout << "last turn flag options: " << "0 = l, 1 = r, 2 = lf, 3 = rf, 4 = f, 5 = lrf, 6 = lr, 7 = dead, 8 = in_corner, 9 = none" << endl;
	cout << "lastTurnFlag is set at " << data->lastTurnFlag << endl;
	cout << "lastTurnFlag_other is set at: " << data->lastTurnFlag_other << endl << endl;
	cout << "whereToGo counter: " << data->whereToGo << endl;
	cout << "whereToGoCount counter: " << data->whereToGoCount << endl;
	cout << "//////////////////////////////////////////////////////////////////////////////////////////////////" << endl << endl;

    data->inst_cnt = 0;
	
	//whereToGoReset
	if(data->cur_lane.type != dead && data->lastDecisionFlag != lturn && data->lastDecisionFlag != rturn){
		data->whereToGoCount ++;
		if(data->whereToGoCount > 3)
		{
			data->whereToGo = 0;
			data->whereToGoCount = 0;
		}
	}
	
    //there are 9 possible versions of the road ahead.
    //using a camera and a switch case, the queue will be filled
    //decisions will be made depending on the traffic sign

    //step 1: add alignment angle to the queue buffer
    int angle;
    int temp;
    if(data->cur_lane.tl_x > 50) //means that the middle is on the right of the car
    { 
        //positive angle
        temp = data->cur_lane.tl_x - 50;
        angle = (int) floor(atan2(temp,data->cur_lane.tl_y) * 180 / new_PI);
    }
    else //means that the middle is on the left of the car
    {
		//negative engle
        temp = -(data->cur_lane.tl_x - 50);
        angle = (int) -(floor(atan2(temp,data->cur_lane.tl_y) * 180 / new_PI));
    }
    angle *= 0.6; //less agressive angle correction
    //angle *= 0.8; //little less agressive angle correction
    if(angle!=0) angleFunc(angle, data);
    data->cur_lane.approachAngle = angle;
    cout << "ANGLE of calibration turn: " << angle << endl;


    //step 2: calculate amount of straight parts
	//can be used when working with longer queues
    int straightDistance = tl_y_to_dist(data->cur_lane.tl_y);
    cout << "tlx: " << data->cur_lane.tl_x << ", tly: " << data->cur_lane.tl_y << endl;


    //step 3: check if either the stop or U-turn sign should be executed
    cout << "+++ FLAG: current redSignflag = " << data->redSignFlag << " and red_sign.type = " << data->red_sign.type << "\n";
    if(data->red_sign.type == none && data->redSignFlag == true){ //the car should wait for about 10 sec
        data->redSignFlagEx = true;
        angleFunc(10,data);
        angleFunc(-10, data);
        cout << "###--- case:STOP redSignFlagEx = " << data->redSignFlagEx << endl;
        return FLUSH;
    }
    if(data->yellow_sign.type == uturn){ // if we should execute a yellow sign, turn for 180 degrees
        angleFunc(90-data->cur_lane.approachAngle, data);
		angleFunc(90, data);
        data->lastDecisionFlag = l; // in case we slip
        cout << "--- case:U-TURN" << endl;
        return FLUSH;
    }

    //step 4: no flags raised, continue with calculating the right steps to be performed
    switch(data->cur_lane.type){
        case r: //turn right
            cout<<"### case:r "<<endl;
            data->lastTurnFlag = r; // register that there is an option to go right
            straightFunc(STRAIGHT_CM, data);
            break;

        case l: //turn left
            cout<<"### case:l"<<endl;
            data->lastTurnFlag = l; // register that there is an option to go left
            straightFunc(STRAIGHT_CM, data);
            break;

        case lf: //turn left, straight
            cout<<"### case:lf"<<endl;
            data->lastTurnFlag_other = lf; // raise a flag that both forward and left are options
            if(data->blueSignFlag == lturn){
                cout<<"###--- subcase:lturn" << endl;
                data->lastTurnFlag = l; // last sign seen is a turn left, we can go forward and left, choose left as way to go
            }
            else
            {	if(data->blueSignFlag == fturn){
                    cout<<"###--- subcase:fturn" << endl;
                    data->lastTurnFlag = none2; // last sign seen is a go straight, we can go forward and left, choose forward as way to go
                    }
				else{cout<< "###--- subcase: no useful sign found to make decision" << endl;}
            }
			straightFunc(STRAIGHT_CM, data);
            break;

        case rf: //turn right, straight
            cout<<"### case:rf"<<endl;
            data->lastTurnFlag_other = rf;
            if(data->blueSignFlag == rturn)
            {
                cout<<"###--- subcase:rturn" << endl;
                data->lastTurnFlag = r;
            }
            else
            { 	if(data->blueSignFlag == fturn){
                    cout<<"###--- subcase:fturn" << endl;
					data->lastTurnFlag = none2;
                    }
				else{cout<< "###--- subcase: no useful sign found to make decision" << endl;}
            }
			straightFunc(STRAIGHT_CM, data);
            break;


        case f: //straight
		#ifdef CROSSOVERMODE //EXPERIMENTAL version 2 for straight road situations. will ignore not crossovers
            cout<<"### case:f"<<endl;
            if(data->blueSignFlag == fturn){
                cout<<"###--- subcase: fturn" << endl;
                data->lastTurnFlag = none2; // we want to go straight, ignore road situation and force to go straight
				data->lastTurnFlag_other = none2;
                straightFunc(STRAIGHT_CM, data);
                break;
            }
            if(data->lastTurnFlag_other == lrf || data->lastTurnFlag_other == lf || data->lastTurnFlag_other == rf){ //if we don't want to go straight, check if we have seen a f+comb road situation
                if((data->blueSignFlag == lturn) && (data->lastTurnFlag_other == lrf || data->lastTurnFlag_other == lf)){ //probably we are at a crossover, we can go left and last sign said left
                    cout<<"###--- subcase:lturn" << endl;
                    angleFunc(-90, data);
                    data->lastTurnFlag = none2;
                    data->lastTurnFlag_other = none2;
                    break;
                }
                if((data->blueSignFlag == rturn) && (data->lastTurnFlag_other == lrf || data->lastTurnFlag_other == rf)){ //probably we are at a crossover, we can go right and last sign said right
                    cout<<"###--- subcase:rturn" << endl;
                    angleFunc(90, data);
                    data->lastTurnFlag = none2;
                    data->lastTurnFlag_other = none2;
                    break;
                }
            }
            else{
				cout<< "###--- subcase: no useful sign found to make decision" << endl;
                straightFunc(STRAIGHT_CM, data); //we didn't get any usefull information, just go straight
                break;
            }
		#endif //CROSSOVERMODE
		
		#ifndef CROSSOVERMODE //version 1 for straight road situations. will ignore crossovers
		cout<<"### case:f"<<endl;
		if(data->blueSignFlag == fturn) {
			cout << "###--- subcase:fturn" << endl;
			data->lastTurnFlag = none2; //only if the sign indicates a forward, set the flag to none, otherwise keep old sign
			straightFunc(STRAIGHT_CM, data); 
			break;
			} 
		else{
			straightFunc(STRAIGHT_CM, data);
			cout<< "###--- subcase: no useful sign found to make decision" << endl;
			break;
		}
		#endif //CROSSOVERMODE not working
			
        case lrf: //turn left, turn right, straight 
            cout<<"###--- case:lrf"<<endl;
            data->lastTurnFlag_other = lrf; // indicate that we're getting close to a crossover

            if(data->blueSignFlag == lturn){
                cout<<"###--- subcase:lturn" << endl;
                data->lastTurnFlag = l; //indicate that left is an option
                straightFunc(STRAIGHT_CM, data);
				break;
            }
          	if(data->blueSignFlag == rturn){
                cout<<"###--- subcase:rturn" << endl;
                data->lastTurnFlag = r; //indicate that right is an option
                straightFunc(STRAIGHT_CM, data);
				break;
            }
            if(data->blueSignFlag == fturn){
                cout<<"###--- subcase:fturn" << endl;
                data->lastTurnFlag = none2; // ignore all other situations, we want to go straight anyway
                straightFunc(STRAIGHT_CM, data);
				break;
            }
			
			cout<< "###--- subcase: no useful sign found to make decision" << endl;
            straightFunc(STRAIGHT_CM, data); //apparently no signs seen, don't mess with current state
            break;

        case lr: //turn left, turn right
            cout<<"###--- case:lr"<<endl;
            if(data->blueSignFlag == lturn){
                cout<<"###--- subcase:lturn" << endl;
                data->lastTurnFlag = l;
                straightFunc(STRAIGHT_CM, data);
				break;
            }
            if(data->blueSignFlag == rturn){
                cout<<"###--- subcase:rturn" << endl;
                data->lastTurnFlag = r; // we can go left and right, by choice of the sign we should go right
                straightFunc(STRAIGHT_CM, data);
				break;
            }
			cout<< "###--- subcase: no useful sign found to make decision" << endl;
            straightFunc(STRAIGHT_CM, data); //apparently no signs seen, don't mess with current state
            break;

        case dead: //dead end
            cout<<"###--- case:dead"<<endl;
            switch(data->lastTurnFlag){
                case r: //running into a dead end, sign+road situation indicate a turn right
					if(data->blueSignFlag == rturn){
                    cout<<"###--- subcase:turnRight" << endl;
                    angleFunc(90, data);
                    data->lastTurnFlag = none2; //reset the flag as you have taken this turn
                    data->lastDecisionFlag = r; //remember that the last large turn taken is a turn right
                    break;
					}

                case l: //running into a dead end, sign+road situation indicate a turn left
					if(data->blueSignFlag == lturn){
                    cout<<"###--- subcase:turnLeft" << endl;
                    angleFunc(-90, data);
                    data->lastTurnFlag = none2; //reset the flag as you have taken this turn
                    data->lastDecisionFlag = l; //remember that the last large turn taken is a turn left
                    break;
					}

                case lr: //last road situation seen is a lr, 
                   /* if(data->blueSignFlag == lturn){
                        cout<<"--- subcase: turn left by blue sign flag" << endl;
                        angleFunc(-90, data);
                        data->lastTurnFlag = none2; //reset the flag as you have taken this turn
                        data->lastDecisionFlag = l; //remember that the last large turn taken is a turn left

                    }
                    else{
                        cout<<"--- subcase: turn right by blue sign flag" << endl;
                        angleFunc(90, data);
                        data->lastTurnFlag = none2;
                        data->lastDecisionFlag = r;

                    }*/
					cout << "I don't see how this could ever trigger... for real..." << endl;
                    break;

                case none2: // counterreact to last turn
                    cout<<"###--- subcase:none" << endl;
                    if(ultraCheck(20, data) == true){ //something in front of the car
                        if(data->lastDecisionFlag == lturn){
                            cout<<"###--- subsubcase:last lturn, new rturn" << endl;
							data->lastDecisionFlag = none2; // correction for oversteering should only occur once
                            angleFunc(15, data);
                        }
                        else{ if(data->lastDecisionFlag == rturn){
                            cout<<"#--- subsubcase:last rturn, new lturn" << endl;
							data->lastDecisionFlag = none2; // correction for oversteering should only occur once
                            angleFunc(-15, data);
                        }
						else{ //we either have not done a turn yet or we already corrected, so we have to find out where to go
							// first try (0) is face left, second try (1) is face right, third try (2) is go back from where we came
							if(data->whereToGo == 0){
								cout << "###??? specialCase: whereToGoLeft" << endl;
								angleFunc(-90, data);
								data->whereToGoCount = 0;
							}
							if(data->whereToGo == 1){
								cout << "-- in dead end, last sign seen is turn right, so take a 90 degrees right" << endl;
								angleFunc(90, data);
								angleFunc(90, data);
								data->whereToGoCount = 0;
							}
							if(data->whereToGo >1){
								cout << "didn't see a sign, idk what to do... just turn left a little and hope for the best?" << endl;
								angleFunc(-90, data);
								data->whereToGo = 0;
							}
							data->whereToGo++;
						}
						}
                    }
                    else{
						//will never trigger since there is no ultrasound module attached
                        cout<<"--- subcase: nope, was a shadow lulz..." << endl;
                        straightFunc(STRAIGHT_CM, data);

                    }
                    break;
            }
            break; //break out of dead case

        case in_corner: //corner dead end
            cout<<"--- case: dead end"<<endl;
            if(data->blueSignFlag == lturn){
                cout<< "--- subcase: last turn probably was left, so go right" << endl;
                angleFunc(45, data);
            }
            else{
                cout<< "--- subcase: last turn probably was right, so go left" << endl;				
                angleFunc(-45, data);
            }
            break;
        default:
            //if we reach here, it's time to cry... for real
            cout<< "--- something really went wrong...."<<endl;
            return BROKEN;
            break;
    }
    cout<<"+++ FLAG: lastTurnFlag = " << data->lastTurnFlag << " | lastTurnFlag_other = " << data->lastTurnFlag_other << " | blueSignFlag = "<< data->blueSignFlag << endl;
	
	cout << endl <<  "/////////FLAGS after processing///////////////////////////////////////////////////////////////////" << endl;
	cout << "last turn flag options: " << "0 = l, 1 = r, 2 = lf, 3 = rf, 4 = f, 5 = lrf, 6 = lr, 7 = dead, 8 = in_corner, 9 = none" << endl;
	cout << "lastTurnFlag is set at " << data->lastTurnFlag << endl;
	cout << "lastTurnFlag_other is set at: " << data->lastTurnFlag_other << endl << endl;
	cout << "whereToGo counter: " << data->whereToGo << endl;
	cout << "whereToGoCount counter: " << data->whereToGoCount << endl;
	cout << "//////////////////////////////////////////////////////////////////////////////////////////////////" << endl << endl;
	
    return FLUSH;
}

////////////////////////
/* Flush Queue State */
//////////////////////
global_state flushQueue(state_data *data){
    //send flush command to ardiuno to make sure that the next queuelist has priority_queue

    /////////////////////
    //Temp init for i2c//
    /////////////////////


    //wait ... seconds for Flush ACK to received
    cout << "____________________________________________________________" << endl;
    cout<<"...FSM:FLUSHING"<<endl;
    int time = 0;
    char buffer[2];
    bool ACK = false;
    int length = 4;
    unsigned char flushCOM[4] = {0};
    flushCOM[0]= 128;
    flushCOM[1]= 'F';
    flushCOM[2]= 'L';
    flushCOM[3]= 'U';
	
    int wat_dan_wel = write(data->file_i2c, flushCOM, length);
    if (wat_dan_wel != length){      //write() returns the number of bytes actually written, if it doesn't match then an error occurred (e.g. no response from the device)
        /* ERROR HANDLING: i2c transaction failed */
        printf("ERROR: Failed to write to the i2c bus, %d. Can't flush.\n", wat_dan_wel);
    } else {
        cout<<"### status: flushed!\n";
    }
    usleep(10000);

    length = 2;         //<<< Number of bytes to read
    if (read(data->file_i2c, buffer, length) != length)       //read() returns the number of bytes actually read, if it doesn't match then an error occurred (e.g. no response from the device)
    {
        //ERROR HANDLING: i2c transaction failed
        printf("ERROR: Failed to read from the i2c bus, flush was not found.\n");
    }
    else
    {
        printf("###: Data read: ");
        for(int i=0; i<length; i++)
            printf("%d ", buffer[i]);
        printf("\n");

        ACK=true;
    }

    cout << "+++ FLAG: the ack flag is: " << ACK << "\n";
    if(ACK == true){
        //continue to sending_queue state
		//cout<< "&&& end of flush queue" << endl;
        return SENDING;
    }
    else{
        data->noAck_count ++;
        //continue to AckError state
		//cout<< "&&& end of flush queue" << endl;
        return ACKERROR;
    }
}

///////////////////////
/* Send Queue State */
//////////////////////
global_state sendQueue(state_data *data) {
    cout << "____________________________________________________________" << endl;
    cout<<"...FSM:SENDING!" << endl;
    //send the newest queue to the arduino
    data->noAck_count = 0;
    //cout << "SENDING" << endl;

    //cout << "\n--------------: " << data->packetstring << endl;
    //write(data->serial_con,data->packetstring,11);

    //wait for .... seconds for ACKs

    bool allAcks = true;
    int time = 0;
    int length = 16;
    char buffer[28];
    for(int j = 0; j < data->inst_cnt; j++){
        length = 16;
        sprintf(data->packetstring,"%c%s%c%03d%c%03d%04d\n",data->all_packets[j].id,
                data->all_packets[j].instr_type,
                data->all_packets[j].ldir,
                data->all_packets[j].lspd,
                data->all_packets[j].rdir,
                data->all_packets[j].rspd,
                data->all_packets[j].duration);
        cout << "\n--------------: " << data->packetstring;
		
		
        if (write(data->file_i2c, data->packetstring, length) != length){      //write() returns the number of bytes actually written, if it doesn't match then an error occurred (e.g. no response from the device)
            /* ERROR HANDLING: i2c transaction failed */
            printf("--- subcase: Failed to write to the i2c bus in sending.\n");
        }
        else{
            cout<<"--- case: sent instruction number: " << j << " \n";
        }
        usleep(120000);

        length = 28;         //<<< Number of bytes to read
        if (read(data->file_i2c, buffer, length) != length)       //read() returns the number of bytes actually read, if it doesn't match then an error occurred (e.g. no response from the device)
        {
            //ERROR HANDLING: i2c transaction failed
            printf("--- subcase: Failed to read from the i2c bus.\n");
            allAcks=false;
        }
        else
        {
            printf("Data read: ");
            for(int i=0; i<2; i++)
                printf("%d ", buffer[i]);
            printf("\n");

            cout << "energy: " << (int) buffer[20] << ", " << (int) buffer[24] << endl;
            for(int i=20; i<28; i++)
                printf("%d ", buffer[i]);
            printf("\n");


        }
        usleep(150000);
    }



    if(allAcks == true)
    {
        //continue to processing_func state

        cout << "+++ FLAG SENDING: the redSignFlag currently = " << data->redSignFlag << endl;
        if(data->redSignFlagEx == true) // if there is a stop sign, wait for 10 seconds before continue 
        {
            cout <<"\n\n\n\n\nJONGE KOME DAN\n\n\n\n\n";
            sleep(10);
            data->redSignFlagEx = false;
            data->redSignFlag = false;
        }
		//cout<< "&&& end of send queue" << endl;
        return PROCESSINGIMG;
    }
    else{
        data->noAck2_count ++;
        cout<< "--- subcase: no ack received..." << endl;
        //continue to AckError state
		//cout<< "&&& end of send queue" << endl;
        return ACKERROR;
    }
}

//////////////////////
/* ACK ERROR State */
////////////////////
global_state ackError(state_data *data){
    cout << "____________________________________________________________" << endl;
    cout << "ERRORPARTY"<<endl;
    cout << "---case: the amount of flush ack errors = " << data->noAck_count << "\n";
    cout << "---case: the amount of send ack errors = " << data->noAck2_count << "\n";
    // check if the acks have been missing for more than 3 times
    if(data->noAck2_count > 3 || data->noAck_count > 3)
    {
        //continue to broken state
        return BROKEN;
    }
    else{
        //continue to sending_flush
        return FLUSH;
    }
}

///////////////////
/* BROKEN State */
/////////////////
global_state broken(state_data *data){
    cout << "BROKEN #RIPMUCH" << endl;
    //turn on red light to show shit is going on
    // wait 10 seconds
    // move to init state
    delete VisionInfo;
    sleep(10);
    return INIT;
}

