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
