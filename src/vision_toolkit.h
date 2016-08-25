#include <omp.h>
#include "types.h"
#include <iomanip>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdlib.h>
#include <ctime>
#include <raspicam/raspicam_cv.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>


#define Y_CALIBRATION 0
#define Y_BOTRECT 270
#define BOT_HEIGHT 210-50
#define BOT_HEIGHT_ORIG 210
#define DEFAULT_GREYVAL 170

#define BLUE_H_LO 145
#define BLUE_H_HI 161
#define BLUE_S_LO 80
#define BLUE_S_HI 255
#define BLUE_V_LO 80
#define BLUE_V_HI 255

#define RED_H_LO 255
#define RED_H_HI 255
#define YELLOW_H_LO 255
#define YELLOW_H_HI 255

#define Y_PIX 480
#define X_PIX 640

//debug purposes
#define CLAHESHOW //turn on to get visual feedback on the CLAHE via x forwarding
#define CLAHEDARKSHOW // turn on to get the dark image visual feedback from either CLAHE when turned on or no CLAHE 
//#define BLUEIMSHOW //turn on to get visual feedback on the blue filter via x forwarding
//#define REDIMSHOW //turn on to get visual feedback on the red filter via x forwarding
//#define YELIMSHOW //turn on to get visual feedback on the yellow filter via x forwarding
#define FINALSHOW //turn on to get visual feedback on the final image with all decisions made via x forwarding
//#define LINEIMSHOW // turn on to get visual feedback on the line filtering via x forwarding
//
#define SAVEFRAMES // turn on to save frames to a folder with day_month_hour, will save both clean and processed images

#define CLAHEPROC // use clahe to improve the contrast of the image
#define CLAHEGRADE 4
#define CLAHETILEW 64 //64 werkt best oke
#define CLAHETILEH 64

class VisionToolkit {
    public:
        VisionToolkit();
        VisionToolkit(char *file);
        int process_frame();
        Sign red_sign;
        Sign blue_sign;
        Sign yellow_sign;
        Lane current_lane;

    private:
        int maxgreyval;
        bool save_frame;
        int frames_since_no_yellow;
        int frames_since_no_red;
        int frames_since_no_left;
        int frames_since_no_right;
        int frames_since_no_forward;
        cv::VideoCapture _cap2;
		raspicam::RaspiCam_Cv _cap;
        cv::Mat _g_final_destination;
        cv::Mat src_col,src;
        
        void initSignMems();
        void calibGrey();
        Sign blob_detect_blue_sign(cv::Mat &blue_hue_image);
        Sign edge_detect_blue_sign(cv::Mat &blue_hue_image);
        Sign edge_detect_red_sign(cv::Mat &red_hue_image);
        Sign edge_detect_yellow_sign(cv::Mat &yellow_hue_image);
        Lane lane_detect(cv::Mat &dark_image);
};
