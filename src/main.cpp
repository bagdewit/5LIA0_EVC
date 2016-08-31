/*
* @file: state_funcs.h
* @author: B. de Wit <b.a.g.d.wit@student.tue.nl> & EVC Group 1 2016 (L. van Harten)
* @course: Embedded Visual Control 5LIA0
*
* This file includes the infinite loop running the FSM
*/


#include "main.h"

using namespace std;

int main()
{
    global_state GS = INIT;
    state_data gs_data;
    state_function funcs [] = 
    {
        init,
		processingImage,
        createQueue,
        flushQueue,
        sendQueue,
        ackError,
	broken
    };

    while(1) {
        GS = funcs[GS](&gs_data);
    }

    return 0;
}
