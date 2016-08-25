#include "state_funcs.h"

using namespace std;

SignDetector *SDetector;

void straightFunc(int distance)
{
		int servoDistance = distance * /*someconstant?*/;
	    data->all_packets[0].instr_type[0] = 'D';
        data->all_packets[0].instr_type[1] = 'R';
        data->all_packets[0].instr_type[2] = 'I';
        data->all_packets[0].instr_type[3] = 0;
        data->all_packets[0].ldir = '+';
        data->all_packets[0].rdir = '+';
        data->all_packets[0].lspd = servoDistance;
        data->all_packets[0].rspd = servoDistance;
}

void angleFunc(int radius)
{
		int servoDistance = distance * /*someconstant?*/;
	    data->all_packets[0].instr_type[0] = 'D';
        data->all_packets[0].instr_type[1] = 'R';
        data->all_packets[0].instr_type[2] = 'I';
        data->all_packets[0].instr_type[3] = 0;
        data->all_packets[0].ldir = '+';
        data->all_packets[0].rdir = '+';
        data->all_packets[0].lspd = servoDistance/2;
        data->all_packets[0].rspd = -servoDistance/2;
}


global_state init(state_data *data) {
    cout << "INIT" << endl;
    SDetector = new SignDetector();
    for(int i=0; i<MAX_PACKETS_IN_AVR; i++) {
        data->all_packets[i].id = 'a'+i;
        data->all_packets[i].in_use = false;
    }

    struct termios serial_options;
    data->serial_con = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
    tcgetattr(data->serial_con, &serial_options);
    cfsetispeed(&serial_options, B9600);
    cfsetospeed(&serial_options, B9600);
    serial_options.c_cflag &= ~PARENB;
    serial_options.c_cflag &= ~CSTOPB;
    serial_options.c_cflag &= ~CSIZE;
    serial_options.c_cflag |= CS8;
    serial_options.c_lflag |= ICANON;
    tcsetattr(data->serial_con, TCSANOW, &serial_options);
    cout << "serial: " << data->serial_con << "\n";
	
	//setup solar panel
	//set angle of magnetometer to bias
	//if something is wrong, move to broken state
	//else continue to processing_func
    return PROCESSINGIMG;
}

global_state processingImage(state_data *data) {
	// global variable noAck2_count = 0;
    //cout << "PROCESSING" << endl;
    SDetector->process_frame();
    data->red_sign = SDetector->red_sign;
    data->blue_sign = SDetector->blue_sign;
    data->yellow_sign = SDetector->yellow_sign;

    data->cur_lane = SDetector->cur_lane;

    data->all_packets[0].instr_type[0] = 'D';
    data->all_packets[0].instr_type[1] = 'R';
    data->all_packets[0].instr_type[2] = 'I';
    data->all_packets[0].instr_type[3] = 0;
    data->all_packets[0].ldir = '+';
    data->all_packets[0].rdir = '+';
    data->all_packets[0].lspd = 0;
    data->all_packets[0].rspd = 0;
    data->all_packets[0].duration = 500;
    data->all_packets[0].id = 128; //direct instr for now

    if(SDetector->blue_sign.size > 0) {
        data->all_packets[0].instr_type[0] = 'D';
        data->all_packets[0].instr_type[1] = 'R';
        data->all_packets[0].instr_type[2] = 'I';
        data->all_packets[0].instr_type[3] = 0;
        data->all_packets[0].ldir = '+';
        data->all_packets[0].rdir = '+';
        switch(SDetector->blue_sign.type) {
            case lturn:
                data->all_packets[0].lspd = 000;
                data->all_packets[0].rspd = 150;
                break;
            case rturn:
                data->all_packets[0].lspd = 150;
                data->all_packets[0].rspd = 000;
                break;
            case fturn:
                data->all_packets[0].lspd = 150;
                data->all_packets[0].rspd = 150;
                break;
            default:
                break;
        }
    }

    cout << "r/b/y sizes: " << SDetector->red_sign.size << "/" << SDetector->blue_sign.size << "/" << SDetector->yellow_sign.size << endl;
    
    if(SDetector->yellow_sign.size > 0) {
        data->all_packets[0].instr_type[0] = 'D';
        data->all_packets[0].instr_type[1] = 'R';
        data->all_packets[0].instr_type[2] = 'I';
        data->all_packets[0].instr_type[3] = 0;
        data->all_packets[0].ldir = '+';
        data->all_packets[0].rdir = '-';
        data->all_packets[0].lspd = 150;
        data->all_packets[0].rspd = 150;
        data->all_packets[0].duration = 500;
    }

    if(SDetector->red_sign.size > 0) {
        data->all_packets[0].instr_type[0] = 'D';
        data->all_packets[0].instr_type[1] = 'R';
        data->all_packets[0].instr_type[2] = 'I';
        data->all_packets[0].instr_type[3] = 0;
        data->all_packets[0].ldir = '-';
        data->all_packets[0].rdir = '-';
        data->all_packets[0].lspd = 150;
        data->all_packets[0].rspd = 150;
        data->all_packets[0].duration = 500;
    }

    sprintf(data->packetstring,"%c%s%c%03d%c%03d%04d\n",data->all_packets[0].id,
                                            data->all_packets[0].instr_type,
                                            data->all_packets[0].ldir,
                                            data->all_packets[0].lspd,
                                            data->all_packets[0].rdir,
                                            data->all_packets[0].rspd,
                                            data->all_packets[0].duration);
	//set sign flags
	data->noAck2_count = 0;
    return CREATEQUEUE;
}

global_state createQueue(state_data *data){
	//in the documentation the different types of track can be found.
	//there are 9 possible versions of the road ahead.
	//using a camera and a switch case, the queue will be filled
	//decisions will be made depending on the traffic sign
	
	//step 1: empty the old queue buffer
	//queue.kill();
	
	//step 2: add alignment angle to the queue buffer
	//angleFunc(cur_lane->angle);
	
	//step 3: calculate amount of straight parts (5cm each)
	int straightDistance
	int someconstant = 1; //dependent on the placement of the camera, static grid can be used, for further information see matlab script
	straightDistance = data->cur_lane->tl_y / someConstant; //depends on the angle of the camera, so for now this will do
	if(straightDistance >9) //we can have a maximum of go straight commands, dependent on the queue length
	{
		straightDistance = 9;
	}
	
	switch(data->cur_lane->type){
		case r: //turn right
			for(int i = 0; i< straightDistance; i++){
				straightFunc(5);
			}
			angleFunc(90-approachAngle);
			break;
		case l: //turn left
			for(int i = 0; i< straightDistance; i++){
				straightFunc(5);
			}
				angleFunc(-90+approachAngle);
			break;
		case lf: //turn left, straight
			if(data->blueSignFlag == 1)
			{
				for(int i = 0; i< straightDistance; i++){
					straightFunc(5);
				}
					angleFunc(-90+approachAngle);
			}
			else
			{
				for(int i = 0; i< straightDistance; i++){
					straightFunc(5);
				}
			}
			break;
		case rf: //turn right, straight
			if(data->blueSignFlag == 2)
			{
				for(int i = 0; i< straightDistance; i++){
					straightFunc(5);
				}
				angleFunc(90-approachAngle);
			}
			else{
				for(int i = 0; i< straightDistance; i++){
					straightFunc(5);
				}
			}
			break;
		case f: //straight
				for(int i = 0; i< straightDistance; i++){
					straightFunc(5);
				}				
			break;
		case lrf: //turn left, turn right, straight 
			if(data->blueSignFlag == 1)
			{
				for(int i = 0; i< straightDistance; i++){
					straightFunc(5);
				}
				angleFunc(-90+approachAngle);
			}
			else{ if(data->blueSignFlag == 2){
				for(int i = 0; i< straightDistance; i++){
					straightFunc(5);
				}
				angleFunc(90-approachAngle);
				else{
					for(int i = 0; i< straightDistance; i++){
						straightFunc(5);
					}					
				}
			}
			break;
		case lr: //turn left, turn right
			if(data->blueSignFlag == 1){
				for(int i = 0; i< straightDistance; i++){
					straightFunc(5);
				}
				angleFunc(-90+approachAngle);
			}
			else{ 
				for(int i = 0; i< straightDistance; i++){
					straightFunc(5);
				}				
			angleFunc(90-approachAngle);
			}
			
			break;
		case dead: //dead end
			if(data->blueSignFlag == 1){
				angleFunc(-90);
			}
			else{ 
				angleFunc(90);
			}
			break;
		case in_corner: //corner dead end
			if(data->blueSignFlag == 1){
				angleFunc(-45);
			}
			else{ 
				angleFunc(45);
			}
			break;
		default:
			//if we reach here, it's time to cry... for real
			return BROKEN;
			break;
	}
	
	return FLUSH;
}

global_state flushQueue(state_data *data){
	//send flush command to ardiuno to make sure that the next queuelist has priority_queue
	
	//wait ... seconds for Flush ACK to received
	
	int time = 0;
	bool ACK = FALSE;
	
	while(ACK != TRUE){
		//look for the flush ack
		//when found, ACK = TRUE;
		
		if(time > flushAckTime){
			break;
		}
		time ++;
	}
	
	if(ACK = TRUE){
		//continue to sending_queue state
		return SENDING;
	}
	else{
		data->noAck_count ++;
		//continue to AckError state
		return ACKERROR;
	}
}

global_state sendQueue(state_data *data) {
	//send the newest queue to the arduino
	data->noAck_count = 0;
    //cout << "SENDING" << endl;

    cout << "\n--------------: " << data->packetstring << endl;
    //write(data->serial_con,data->packetstring,11);
	
	//wait for .... seconds for ACKs
	
	bool allAcks = FALSE;
	int time = 0;
	while(allAcks != TRUE)
	{
		//check on acks
		//see how many acks are missing
		//no ack missing -> allAcks = TRUE
		
		//if it takes too much time to find all the acks, error
		if(time > ackWaitTime){
			break;
		}
		time ++;
	}
	
	if(allAcks = TRUE)
	{
		//continue to processing_func state
		return PROCESSING;
	}
	else{
		data->noAck2_count ++;
		//continue to AckError state
		return ACKERROR;
	}
}

global_state ackError(state_data *data){
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

global_state broken(state_data *data){
	//turn on red light to show shit is going on
    // wait 10 seconds
	// move to init state
	sleep(10000);
	return INIT;
}

