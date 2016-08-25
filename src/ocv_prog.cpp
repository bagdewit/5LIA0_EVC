#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
 
using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    if (argc != 2) {
        cout <<" Usage: ./thisfile someimage.jpg" << endl;
        return -1;
    }

    Mat src = imread(argv[1], CV_LOAD_IMAGE_COLOR);
 
    Mat hsv_image;
    cvtColor(src, hsv_image, cv::COLOR_BGR2HSV);

    Mat dark_image;
    inRange(hsv_image, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 180), dark_image);
    
    Rect croprect(0,120,432,120);
    Mat imcrop;
    dark_image(croprect).copyTo(imcrop);

    erode(imcrop, imcrop, getStructuringElement(MORPH_RECT, Size(2,2)), Point(-1,-1),1);
    //dilate(imcrop, imcrop, getStructuringElement(MORPH_RECT, Size(2,2)), Point(-1,-1),1);



    Mat dst, cdst;
    Canny(imcrop, dst, 100, 200, 3);
    cvtColor(dst, cdst, CV_GRAY2BGR);
    vector<Vec4i> lines;
    HoughLinesP(dst, lines, 1, CV_PI/720, 25, 30, 10 );
    for( size_t i = 0; i < lines.size(); i++ )
    {
        Vec4i l = lines[i];
        line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
        cout << l[0] << ", " << l[1] << ", " << l[2] << ", " << l[3] << ", " << endl;
    }
    imshow("detected lines", cdst);


    vector<Vec4i> lb_candidates;
    vector<Vec4i> rb_candidates;
    vector<Vec4i> f_candidates;

    for( size_t i = 0; i < lines.size(); i++ )
    {
        Vec4i l = lines[i];
        if(l[0] < 5 && l[3] < l[1]) lb_candidates.push_back(l);
        else if(l[2] > 428 && l[3] > l[1]) rb_candidates.push_back(l);
        else f_candidates.push_back(l);
    }

    int additions=1;
    while(additions) {
        additions=0;
        for(size_t j=0; j < f_candidates.size(); j++) {
            Vec4i cand = f_candidates[j];
            for(size_t i=0; i < rb_candidates.size(); i++) {
                Vec4i l = rb_candidates[i];
                if(cand[2]+5 > l[0] && cand[2]-5 < l[0] && cand[3]+5 > l[1] && cand[3]-5 < l[1]) {
                    rb_candidates.push_back(cand);
                    f_candidates.erase(f_candidates.begin()+j-1);
                    additions++;
                }
            }
            for(size_t i=0; i < lb_candidates.size() && find(f_candidates.begin(), f_candidates.end(), cand) != f_candidates.end(); i++) {
                Vec4i l = lb_candidates[i];
                Vec4i cand = f_candidates[j];
                if(cand[0]+5 > l[2] && cand[0]-5 < l[2] && cand[1]+5 > l[3] && cand[1]-5 < l[3]) {
                    lb_candidates.push_back(cand);
                    f_candidates.erase(f_candidates.begin()+j-1);
                    additions++;
                }
            }
        }
    }
    if (lb_candidates.back()[2]+5 > rb_candidates.back()[0] && lb_candidates.back()[2]-5 < rb_candidates.back()[0] &&
        lb_candidates.back()[3]+5 > rb_candidates.back()[1] && lb_candidates.back()[3]-5 < rb_candidates.back()[1]) {
        cout << "Dead end or corner" << endl;
    }


    imshow("dark_image", imcrop);
 
    while((cv::waitKey() & 0xEFFFFF) != 27); //27 is the keycode for ESC
    return 0;
}
