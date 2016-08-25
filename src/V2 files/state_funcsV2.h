#ifndef STATE_FUNC_H
#define STATE_FUNC_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <linux/ioctl.h>
#include <unistd.h> 
#include <fcntl.h>
#include <termios.h>
#include "ocv_routines.h"
#include "types.h"
#include "SignDetector.h"

global_state init_Image(state_data *data);
global_state processingQueue_func(state_data *data);
global_state flushQueue_func(state_data *data);
global_state sendQueue_func(state_data *data);
global_state ackError_func(state_data *data);
global_state broken_func(state_data *data);
#endif
