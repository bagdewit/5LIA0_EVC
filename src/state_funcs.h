#ifndef STATE_FUNC_H
#define STATE_FUNC_H



#include <iostream>
#include <stdio.h>
#include "ocv_routines.h"
#include "types.h"
#include "vision_toolkit.h"

#include <unistd.h>             //Needed for I2C port
#include <fcntl.h>              //Needed for I2C port
#include <sys/ioctl.h>          //Needed for I2C port
#include <linux/i2c-dev.h>      //Needed for I2C port
#include <stdlib.h>


#include <math.h>

#define WAITKEY 1 //1 enable wait on key, 0 disable wait on key
#define REV_TO_DIST 1 //define the amount of encoder steps it takes to go for 1cm
#define REV_TO_ANGLE 7.7 //amount of steps to rotate 1 degree
#define FLUSHACKTIME 10 //maximum time flush ack should take
#define QUEUEACKTIME 20 //maximum time queue ack should take
# define new_PI           3.14159265358979323846
#define STRAIGHT_CM 200 // the amount of cm each chunk should be

//#define CROSSOVERMODE // experimental mode for crossovers

void straightFunc(int distance, state_data *data);
void angleFunc(int radius, state_data *data);

global_state init(state_data *data);
global_state processingImage(state_data *data);
global_state createQueue(state_data *data);
global_state flushQueue(state_data *data);
global_state sendQueue(state_data *data);
global_state ackError(state_data *data);
global_state broken(state_data *data);
#endif
