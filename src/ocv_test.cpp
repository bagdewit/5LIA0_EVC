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
 
    Mat dst, cdst;
    Canny(src, dst, 50, 200, 3); 
    cvtColor(dst, cdst, CV_GRAY2BGR); 
 
    vector<Vec2f> lines;
    // detect lines
    HoughLines(dst, lines, 1, CV_PI/180, 150, 0, 0 );
    cout << lines.size() << endl;
 
    // Reduce the noise so we avoid false circle detection
    GaussianBlur(src, src, Size(9, 9), 2, 2 );
    
    // Circles
    vector<Vec3f> circles;
    HoughCircles(src, circles, CV_HOUGH_GRADIENT, 1, dst.rows/8, 200, 45, 0, 0 );
    cout << circles.size() << endl;

    // draw lines
    for( size_t i = 0; i < lines.size(); i++ )
    {
        float rho = lines[i][0], theta = lines[i][1];
        Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        line( cdst, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
    }
    for( size_t i = 0; i < circles.size(); i++ )
    {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        // circle center
        circle(cdst, center, 3, Scalar(0,255,0), -1, 8, 0 );
        // circle outline
        circle(cdst, center, radius, Scalar(255,255,0), 3, 8, 0 );
    }
 
    imshow("source", src);
    imshow("detected lines", cdst);
 
    while((cv::waitKey() & 0xEFFFFF) != 27); //27 is the keycode for ESC
    return 0;
}
