#include "main.h"

using namespace std;

int main()
{
    global_state GS = INIT;
    state_data gs_data;
    state_function funcs [] = 
    {
        init_func,
        processingImage_func,
        createQueue_func,
        flushQueue_func,
        sendQueue_func,
        ackError_func,
		broken_func
    };

    while(1) {
        GS = funcs[GS](&gs_data);
    }

    return 0;
}
