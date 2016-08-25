#include "ocv_routines.h"

using namespace cv;
using namespace std;

int circle_and_line_detected()
{
    char filename[20] = "test.jpg";
    double t_start, t_stop, t_tot;

    t_start = t_tot = omp_get_wtime();
    Mat src = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
    t_stop = omp_get_wtime();
    std::cout << " t_imread: " << t_stop-t_start << "\n";
 
    Mat dst, cdst;
    t_start = omp_get_wtime();
    Canny(src, dst, 50, 200, 3); 
    t_stop = omp_get_wtime();
    std::cout << " t_canny: " << t_stop-t_start << "\n";

    t_start = omp_get_wtime();
    cvtColor(dst, cdst, CV_GRAY2BGR); 
    t_stop = omp_get_wtime();
    std::cout << " t_cvtColor: " << t_stop-t_start << "\n";
 
    vector<Vec2f> lines;
    // detect lines
    t_start = omp_get_wtime();
    HoughLines(dst, lines, 1, CV_PI/180, 150, 0, 0 );
    t_stop = omp_get_wtime();
    std::cout << " t_houghline: " << t_stop-t_start << "\n";
 
    // Reduce the noise so we avoid false circle detection
    t_start = omp_get_wtime();
    GaussianBlur(src, src, Size(9, 9), 2, 2 );
    t_stop = omp_get_wtime();
    std::cout << " t_blur: " << t_stop-t_start << "\n";
    
    // Circles
    vector<Vec3f> circles;
    t_start = omp_get_wtime();
    HoughCircles(src, circles, CV_HOUGH_GRADIENT, 1, dst.rows/8, 200, 45, 0, 0 );
    t_stop = omp_get_wtime();
    std::cout << " t_houghcircle: " << t_stop-t_start << "\n";
    
    std::cout << "        TOTAL: " << t_stop-t_tot << "\n";

    return (lines.size()>0)+2*(circles.size()>0);
}
