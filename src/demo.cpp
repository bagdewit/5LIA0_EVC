#include <omp.h>
#include <iomanip>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include "types.h"
#include "vision_toolkit.h"
 
using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    VisionToolkit *sd;
    if (argc == 1) {
        sd = new VisionToolkit();
    } else {
        sd = new VisionToolkit(argv[1]);
    }
    while(1) {
        sd->process_frame();
        cv::waitKey(3);
    }
    return 0;
}
