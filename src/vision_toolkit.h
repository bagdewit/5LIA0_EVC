/*
* @file: vision_toolkit.h
* @author: B. de Wit <b.a.g.d.wit@student.tue.nl> & EVC Group 1 2016 (L. van Harten)
* @course: Embedded Visual Control 5LIA0
*
* This file contains all the includes, defines and classes needed for 
* vision_toolkit.h . This version runs a Raspberry Pi Cam in combination
* with openCV. Multiple settings can be changed for Cam calibration in 
* block #2. Visual debug feedback can be enabled in block #3. The CLAHE
* definitions can be found in block #4.
*/

////////////////////////
/* Block #1: includes*/
//////////////////////

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


//////////////////////////////////////////
/*Block #2: Pi Cam calibration settings*/
////////////////////////////////////////

//pixel size settings (box used to see road and signs)
#define Y_CALIBRATION 0
#define Y_BOTRECT 270
#define BOT_HEIGHT 210-50
#define BOT_HEIGHT_ORIG 210
#define DEFAULT_GREYVAL 170

//#define CUSTOMCOLORVAL //turn on to use own defined sign colour values

//blue sign values
#define BLUE_H_LO 100
#define BLUE_H_HI 161
#define BLUE_S_LO 80
#define BLUE_S_HI 255
#define BLUE_V_LO 35
#define BLUE_V_HI 255

//yellow sign values
#define YEL_H_LO 15
#define YEL_H_HI 40
#define YEL_S_LO 200
#define YEL_S_HI 255
#define YEL_V_LO 140
#define YEL_V_HI 255

//red sign values lower bound
#define RED1_H_LO 0
#define RED1_H_HI 10
#define RED1_S_LO 30
#define RED1_S_HI 255
#define RED1_V_LO 30
#define RED1_V_HI 255

//red sign values upper bound
#define RED2_H_LO 160
#define RED2_H_HI 255
#define RED2_S_LO 30
#define RED2_S_HI 255
#define RED2_V_LO 30
#define RED2_V_HI 255

//resolution
#define Y_PIX 480
#define X_PIX 640

////////////////////////////////////
/*Block #3: Debug Window Settings*/
//////////////////////////////////

//debug visual feedback
//#define CLAHESHOW //turn on to get visual feedback on the CLAHE via x forwarding
//#define CLAHEDARKSHOW // turn on to get the dark image visual feedback from either CLAHE when turned on or no CLAHE 
#define BLUEIMSHOW //turn on to get visual feedback on the blue filter via x forwarding
//#define REDIMSHOW //turn on to get visual feedback on the red filter via x forwarding
//#define YELIMSHOW //turn on to get visual feedback on the yellow filter via x forwarding
#define FINALSHOW //turn on to get visual feedback on the final image with all decisions made via x forwarding
//#define LINEIMSHOW // turn on to get visual feedback on the line filtering via x forwarding
//#define DILEROSHOW // turn on to get visual feedback on the dilate and erode processing via x forwarding
//
#define SAVEFRAMES // turn on to save frames to a folder with day_month_hour, will save both clean and processed images

/////////////////////////////
/*Block #4: CLAHE settings*/
///////////////////////////

//CLAHE (contrast limited adaptive histogram equalization) settings
#define CLAHEPROC // use CLAHE to improve the contrast of the image
#define CLAHEGRADE 4
#define CLAHETILEW 64 //64 werkt best oke
#define CLAHETILEH 64


////////////////////////////////
/*Block #5: class definitions*/
//////////////////////////////

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
