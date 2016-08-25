#include <omp.h>
#include <iomanip>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include "types.h"
 
using namespace cv;
using namespace std;

Mat _g_final_destination;

Sign blob_detect_blue_sign(Mat &blue_hue_image) {
    Sign result;
    // Get connected components and stats
    const int connectivity_8 = 8;
    Mat labels, stats, centroids;

    int nLabels ;
    nLabels = connectedComponentsWithStats(blue_hue_image, labels, stats, centroids, connectivity_8, CV_32S);
    //cout << "Number of connected components = " << nLabels << endl;

    if(nLabels<2) {
        result.size = 0;
        return result;
    }

    vector<pair<int, int> > sizes(nLabels);
    for (int i=0; i<nLabels; i++) {
        sizes[i] = make_pair(stats.at<int>(i,CC_STAT_AREA),i);
    }
    sort(sizes.begin(), sizes.end());
    for (int i = min(5, nLabels-2); i>1; i--) {
        int blobnum = (sizes.end()-i)->second;
        if(stats.at<int>(blobnum,CC_STAT_AREA) > 400 && !blue_hue_image.at<uchar>(stats.at<int>(blobnum,CC_STAT_TOP),stats.at<int>(blobnum,CC_STAT_LEFT))) {
            int sample_height = stats.at<int>(blobnum,CC_STAT_TOP) + 0.6*stats.at<int>(blobnum,CC_STAT_HEIGHT);
            int lwidth = stats.at<int>(blobnum,CC_STAT_LEFT) + 0.33*stats.at<int>(blobnum,CC_STAT_WIDTH);
            int mwidth = stats.at<int>(blobnum,CC_STAT_LEFT) + 0.5*stats.at<int>(blobnum,CC_STAT_WIDTH);
            int rwidth = stats.at<int>(blobnum,CC_STAT_LEFT) + 0.66*stats.at<int>(blobnum,CC_STAT_WIDTH);
            if(!blue_hue_image.at<uchar>(sample_height,lwidth)) {
                result.size = stats.at<int>(blobnum,CC_STAT_AREA);
                cout << "---TURN RIGHT! (" << result.size << ")\n";
                result.type = rturn;
                result.tl_x = stats.at<int>(blobnum,CC_STAT_LEFT);
                result.tl_y = stats.at<int>(blobnum,CC_STAT_TOP);
                result.w = stats.at<int>(blobnum,CC_STAT_WIDTH);
                result.h = stats.at<int>(blobnum,CC_STAT_HEIGHT);
            } else if (!blue_hue_image.at<uchar>(sample_height,mwidth)) {
                result.size = stats.at<int>(blobnum,CC_STAT_AREA);
                cout << "---STRAIGHT AHEAD! (" << result.size << ")\n";
                result.type = fturn;
                result.tl_x = stats.at<int>(blobnum,CC_STAT_LEFT);
                result.tl_y = stats.at<int>(blobnum,CC_STAT_TOP);
                result.w = stats.at<int>(blobnum,CC_STAT_WIDTH);
                result.h = stats.at<int>(blobnum,CC_STAT_HEIGHT);
            } else if (!blue_hue_image.at<uchar>(sample_height,rwidth)) {
                result.size = stats.at<int>(blobnum,CC_STAT_AREA);
                cout << "---TURN LEFT (" << result.size << ")\n";
                result.type = lturn;
                result.tl_x = stats.at<int>(blobnum,CC_STAT_LEFT);
                result.tl_y = stats.at<int>(blobnum,CC_STAT_TOP);
                result.w = stats.at<int>(blobnum,CC_STAT_WIDTH);
                result.h = stats.at<int>(blobnum,CC_STAT_HEIGHT);
            } else {
                cout << "---ERROR: BLUE SIGN BUT CAN'T FIND WHICH ONE!\n";
            }
            //circle( _g_final_destination, center[largestComp], (int)radius[largestComp], color, 2, 8, 0 );
        }
    }
    return result;
}

Sign edge_detect_blue_sign(Mat &blue_hue_image) {
    Sign result;
    result.type = none;
    result.size = 0; 

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    Mat temp = blue_hue_image.clone();

    erode(blue_hue_image, blue_hue_image, getStructuringElement(MORPH_RECT, Size(2,2)), Point(-1,-1),2);
    dilate(temp, temp, getStructuringElement(MORPH_RECT, Size(3,3)), Point(-1,-1),2);

    findContours( temp, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE );

    cvtColor(temp,temp, CV_GRAY2BGR);

    vector<vector<Point> > contours_poly( contours.size() );
    vector<Rect> boundRect( contours.size() );
    vector<Point2f>center( contours.size() );
    vector<float>radius( contours.size() );
    vector<vector<Point> > hullsP(contours.size());

    if( contours.size() == 0 ) {
        cout << "---no blue contours found...\n";
        result.size = 0;
        return result;
    }

    int idx = 0, largestComp = -1;
    int maxWeight = 0;

    for( ; idx >= 0; idx = hierarchy[idx][0] )
    {
        drawContours( temp, contours, idx, Scalar(0,0,255), 1, 8, vector<Vec4i>(), 0, Point() );
        approxPolyDP( Mat(contours[idx]), contours_poly[idx], 3, true );
        if(contours_poly[idx].size() > 7 && isContourConvex(contours_poly[idx])) {
            boundRect[idx] = boundingRect( Mat(contours_poly[idx]) );
            minEnclosingCircle( (Mat)contours_poly[idx], center[idx], radius[idx] );
            int weight = 5*boundRect[idx].height+boundRect[idx].width;
            if( weight > maxWeight )
            {
                maxWeight = weight;
                largestComp = idx;
            }
        }
    }
    if(maxWeight == 0) {
        cout << "---No blue ovals found\n";
    } else if (maxWeight < 100) {
        cout <<"---Tiny blue oval found (" << maxWeight<< ")\n";
    } else {
        result.size = maxWeight; 

        int sample_height = boundRect[largestComp].y + 0.6*boundRect[largestComp].height;

        int lwidth = boundRect[largestComp].x + 0.33*boundRect[largestComp].width;
        int mwidth = boundRect[largestComp].x + 0.5*boundRect[largestComp].width;
        int rwidth = boundRect[largestComp].x + 0.66*boundRect[largestComp].width;

        if(!blue_hue_image.at<uchar>(sample_height,lwidth) ||
                !blue_hue_image.at<uchar>(sample_height,lwidth-1) ||
                !blue_hue_image.at<uchar>(sample_height,lwidth+1)) {
            result.size = maxWeight;
            cout << "---TURN RIGHT! (" << result.size << ")\n";
            result.type = rturn;
            result.tl_x = boundRect[largestComp].x;
            result.tl_y = boundRect[largestComp].y;
            result.w = boundRect[largestComp].width;
            result.h = boundRect[largestComp].height;
        } else if(!blue_hue_image.at<uchar>(sample_height,mwidth) ||
                !blue_hue_image.at<uchar>(sample_height,mwidth-1) ||
                !blue_hue_image.at<uchar>(sample_height,mwidth+1)) {
            result.size = maxWeight;
            cout << "---STRAIGHT AHEAD! (" << result.size << ")\n";
            result.type = fturn;
            result.tl_x = boundRect[largestComp].x;
            result.tl_y = boundRect[largestComp].y;
            result.w = boundRect[largestComp].width;
            result.h = boundRect[largestComp].height;
        } else if(!blue_hue_image.at<uchar>(sample_height,rwidth) ||
                !blue_hue_image.at<uchar>(sample_height,rwidth-1) ||
                !blue_hue_image.at<uchar>(sample_height,rwidth+1)) {
            result.size = maxWeight;
            cout << "---TURN LEFT (" << result.size << ")\n";
            result.type = lturn;
            result.tl_x = boundRect[largestComp].x;
            result.tl_y = boundRect[largestComp].y;
            result.w = boundRect[largestComp].width;
            result.h = boundRect[largestComp].height;
        } else {
            result.type = none;
            result.size = maxWeight;
            result.tl_x = boundRect[largestComp].x;
            result.tl_y = boundRect[largestComp].y;
            result.w = boundRect[largestComp].width;
            result.h = boundRect[largestComp].height;
        }

        /// Draw polygonal contour + bonding rects + circles (debug)
        Scalar color = Scalar(0,0,255);
        drawContours( _g_final_destination, contours_poly, largestComp, color, 1, 8, vector<Vec4i>(), 0, Point() );
        rectangle( _g_final_destination, boundRect[largestComp].tl(), boundRect[largestComp].br(), color, 2, 8, 0 );
        circle( _g_final_destination, center[largestComp], (int)radius[largestComp], color, 2, 8, 0 );
    }

    //imshow("blue_temp", temp);
    //imshow("blue_all", blue_hue_image);

    return result;
}

Sign edge_detect_red_sign(Mat &red_hue_image) {
    Sign result;
    result.type = stopsign;
    result.size = 0; 

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    Mat temp = red_hue_image.clone();

    erode(temp, temp, getStructuringElement(MORPH_RECT, Size(2,2)), Point(-1,-1),1);
    dilate(temp, temp, getStructuringElement(MORPH_RECT, Size(3,3)), Point(-1,-1),4);

    Mat tmp = temp.clone();
    imshow("red_temp", tmp);
    findContours( temp, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE );

    cvtColor(temp,temp, CV_GRAY2BGR);

    vector<vector<Point> > contours_poly( contours.size() );
    vector<Rect> boundRect( contours.size() );
    vector<Point2f>center( contours.size() );
    vector<float>radius( contours.size() );
    vector<vector<Point> > hullsP(contours.size());

    if( contours.size() == 0 ) {
        cout << "---no red contours found...\n";
        result.size = 0;
        return result;
    }

    int idx = 0, largestComp = -1;
    int maxWeight = 0;

    for( ; idx >= 0; idx = hierarchy[idx][0] )
    {
        drawContours( temp, contours, idx, Scalar(0,0,255), 1, 8, vector<Vec4i>(), 0, Point() );
        approxPolyDP( Mat(contours[idx]), contours_poly[idx], 3, true );
        if(contours_poly[idx].size() == 8 && isContourConvex(contours_poly[idx])) {
            boundRect[idx] = boundingRect( Mat(contours_poly[idx]) );
            minEnclosingCircle( (Mat)contours_poly[idx], center[idx], radius[idx] );
            int weight = 5*boundRect[idx].height+boundRect[idx].width;
            if( weight > maxWeight )
            {
                maxWeight = weight;
                largestComp = idx;
            }
        }
    }
    if(maxWeight == 0) {
        cout << "---No red Octagons found\n";
    } else if (maxWeight < 200) {
        cout <<"---Maybe stop? Tiny red octagon found (" << maxWeight<< ")\n";
    } else {
        cout <<"---STOP (" << maxWeight<< ")\n";
        result.size = maxWeight; 

        /// Draw polygonal contour + bonding rects + circles (debug)
        Scalar color = Scalar(0,0,255);
        drawContours( _g_final_destination, contours_poly, largestComp, color, 1, 8, vector<Vec4i>(), 0, Point() );
        rectangle( _g_final_destination, boundRect[largestComp].tl(), boundRect[largestComp].br(), color, 2, 8, 0 );
        circle( _g_final_destination, center[largestComp], (int)radius[largestComp], color, 2, 8, 0 );
    }
    //imshow("red_temp", temp);
    return result;
}

Sign edge_detect_yellow_sign(Mat &yellow_hue_image) {
    Sign result;
    result.type = uturn;
    result.size = 0; 

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    Mat temp;

    erode(yellow_hue_image, temp, getStructuringElement(MORPH_RECT, Size(2,2)), Point(-1,-1),4);
    //dilate(temp, temp, getStructuringElement(MORPH_RECT, Size(2,2)), Point(-1,-1),1);

    Mat tmp = temp.clone();
    //imshow("yellow_eroded",tmp);
    findContours( temp, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE );

    vector<vector<Point> > contours_poly( contours.size() );
    vector<Rect> boundRect( contours.size() );
    vector<Point2f>center( contours.size() );
    vector<float>radius( contours.size() );


    if( contours.size() == 0 ) {
        cout << "---no yellow contours found...\n";
        result.size = 0;
        return result;
    }

    int idx = 0, largestComp = -1;
    int maxWeight = 0;
    RotatedRect temp_rect, big_rect; 
    for( ; idx >= 0; idx = hierarchy[idx][0] )
    {
        temp_rect = minAreaRect(contours[idx]);
        approxPolyDP( Mat(contours[idx]), contours_poly[idx], 3, true );
        if(temp_rect.angle > -50 && temp_rect.angle < -40 
                && temp_rect.size.width > temp_rect.size.height*0.75
                && temp_rect.size.width < temp_rect.size.height*1.33) { //in case of bubbly edge
            cout << "w: " <<temp_rect.size.width<< ", h: " << temp_rect.size.height << endl;
            boundRect[idx] = boundingRect( Mat(contours_poly[idx]) );
            minEnclosingCircle( (Mat)contours_poly[idx], center[idx], radius[idx] );
            int weight = 5*temp_rect.size.height+temp_rect.size.width;
            if( weight > maxWeight )
            {
                maxWeight = weight;
                largestComp = idx;
                big_rect = temp_rect;
            }
        }
    }
    if(maxWeight == 0) {
        cout << "---No yellow squares found\n";
    } else if (maxWeight < 200) {
        cout <<"---Maybe uturn? Tiny yellow square found (" << maxWeight<< ")\n";
    } else {
        cout <<"---UTURN (" << maxWeight<< ")\n";
        result.size = maxWeight; 

        /// Draw polygonal contour + bonding rects + circles (debug)
        Scalar color = Scalar(0,255,255);
        drawContours( _g_final_destination, contours_poly, largestComp, color, 1, 8, vector<Vec4i>(), 0, Point() );
        Point2f rect_points[4]; big_rect.points( rect_points );
        for( int j = 0; j < 4; j++ ) line( _g_final_destination, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
        circle( _g_final_destination, center[largestComp], (int)radius[largestComp], color, 2, 8, 0 );
    }

    //imshow("yellow_temp", temp);

    return result;
}

int main(int argc, char** argv)
{
    double main_tic = omp_get_wtime();
    double main_toc;
    Mat src_col,src;
    VideoCapture cap;
    if (argc == 1) {
        cout <<" Usage: ./thisfile someimage.jpg" << endl;
        cout <<" using camera...\n";
        if(!cap.open(0))
            return 0;
        cap.set(CAP_PROP_FRAME_WIDTH,640);
        cap.set(CAP_PROP_FRAME_HEIGHT,480);
        for (int i=0; i<5; i++) //warmup frames
            cap.read(src_col);
    } else if (argc == 3) {
        cout <<" using stream...\n";
        if(!cap.open(argv[2]))
            return 0;
    } else {
        src_col = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    }
filthy_goto:
    if(argc != 2) {
        main_tic = omp_get_wtime();
        cap.read(src_col);
        cv::waitKey(3);
        cap.read(src_col);
        cv::waitKey(3);
        cap.read(src_col);
        cv::waitKey(3);
        cap.read(src_col);
        cv::waitKey(3);
        cap.read(src_col);
        cv::waitKey(3);
        cap.read(src_col);
        cv::waitKey(3);
        cap.read(src_col);
        cv::waitKey(3);
        cap.read(src_col);
        cv::waitKey(3);
        cap.read(src_col);
        cv::waitKey(3);
        cap.read(src_col);
    }

    main_toc = omp_get_wtime();
    cout << "(main cuml) load rgb image: " << setprecision(7) << main_toc-main_tic << endl;

    cout << "\nWxH: " << src_col.cols << ", " << src_col.rows << endl;
    
    Rect topcroprect(0,0,432,160);
    Rect botcroprect(0,120,432,120);
    
    Mat topcrop, botcrop;
    src_col(topcroprect).copyTo(topcrop);
    src_col(botcroprect).copyTo(botcrop);

    _g_final_destination = src_col;
    Mat hsv_image;
    cvtColor(topcrop, hsv_image, cv::COLOR_BGR2HSV);

    main_toc = omp_get_wtime();
    cout << "(main cuml) hsv converted: " << setprecision(7) << main_toc-main_tic << endl;
    cout << main_toc << endl;

    Mat lower_red_hue_range;
    Mat upper_red_hue_range;
    Mat red_hue_image;
    Mat blue_hue_image;
    Mat yellow_hue_image;
 
    inRange(hsv_image, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red_hue_range);
    inRange(hsv_image, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), upper_red_hue_range);
    addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);
    inRange(hsv_image, cv::Scalar(90, 50, 30), cv::Scalar(135, 255, 255), blue_hue_image);
    inRange(hsv_image, cv::Scalar(15, 100, 100), cv::Scalar(27, 255, 255), yellow_hue_image);

    main_toc = omp_get_wtime();
    cout << "(main cuml) split images to hsv ranges: " << setprecision(7) << main_toc-main_tic << endl;

    edge_detect_blue_sign(blue_hue_image);
    edge_detect_red_sign(red_hue_image);
    edge_detect_yellow_sign(yellow_hue_image);

    main_toc = omp_get_wtime();
    cout << "(main cuml) full_program_timing: " << setprecision(7) << main_toc-main_tic << endl;

    //imshow("blue", blue_hue_image);
    //imshow("red", red_hue_image);
    //imshow("yellow", yellow_hue_image); 
    //imshow("final destination", _g_final_destination); 
    if(argc == 2) {
        while((cv::waitKey() & 0xEFFFFF) != 27); //27 is the keycode for ESC
    } else {
        cv::waitKey(30);
    }
    if(argc != 2) 
        goto filthy_goto;
    return 0;
}
