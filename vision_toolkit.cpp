#include "vision_toolkit.h"
 
using namespace cv;
using namespace std;

Sign VisionToolkit::blob_detect_blue_sign(Mat &blue_hue_image) {
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

Sign VisionToolkit::edge_detect_blue_sign(Mat &blue_hue_image) {
    Sign result;
    result.type = none;
    result.size = 0; 

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    Mat temp = blue_hue_image.clone();
    Mat print1 = blue_hue_image.clone();

    erode(blue_hue_image, blue_hue_image, getStructuringElement(MORPH_RECT, Size(1,4)), Point(-1,-1),2);
    erode(temp, temp, getStructuringElement(MORPH_RECT, Size(2,2)), Point(-1,-1),1);
    dilate(temp, temp, getStructuringElement(MORPH_RECT, Size(3,3)), Point(-1,-1),3);
    
    Mat print2 = blue_hue_image.clone();
    Mat print3 = temp.clone();

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
        //drawContours( _g_final_destination, contours, idx, Scalar(0,0,0), 1, 8, vector<Vec4i>(), 0, Point() );
        approxPolyDP( Mat(contours[idx]), contours_poly[idx], 3, true );
        boundRect[idx] = boundingRect( Mat(contours_poly[idx]) );
        if(contours_poly[idx].size() > 7 && 
                boundRect[idx].height + boundRect[idx].tl().y < Y_BOTRECT-15 &&
                boundRect[idx].height*0.7 < boundRect[idx].width && 
                boundRect[idx].width*0.9 < boundRect[idx].height){// && isContourConvex(contours_poly[idx])) {
            minEnclosingCircle( (Mat)contours_poly[idx], center[idx], radius[idx] );
            int weight = contourArea(contours[idx]);
            if( weight > maxWeight && 
                    center[idx].x - radius[idx] + radius[idx]/4 > boundRect[idx].tl().x &&
                    center[idx].y - radius[idx] + radius[idx]/4 > boundRect[idx].tl().y)
            {
                maxWeight = weight;
                largestComp = idx;
            }
            cout << "oval with bot y: " << boundRect[idx].height + boundRect[idx].tl().y << ", " << weight << endl;
            cout << "circ: " << center[idx].x - radius[idx] << ", " << center[idx].y - radius[idx] << ", " << radius[idx] <<endl;
            cout << "rect: " << boundRect[idx].tl().x << ", " << boundRect[idx].tl().y << endl;
            Scalar color = Scalar(0,0,0);
            rectangle( _g_final_destination, boundRect[idx].tl(), boundRect[idx].br(), color, 2, 8, 0 );
            circle( _g_final_destination, center[idx], (int)radius[idx], color, 2, 8, 0 );
        }
    }
    if(maxWeight == 0) {
        result.type = none;
        frames_since_no_left = 0;
        frames_since_no_forward = 0;
        frames_since_no_right = 0;
        cout << "---No blue ovals found\n";
    } else if (maxWeight < 1200) {
        result.type = none;
        frames_since_no_left = 0;
        frames_since_no_forward = 0;
        frames_since_no_right = 0;
        cout <<"---Tiny blue oval found (" << maxWeight<< ")\n";
    } else {
        result.size = maxWeight; 

        int sample_height = boundRect[largestComp].y + 0.66*boundRect[largestComp].height;

        int lwidth = boundRect[largestComp].x + 0.33*boundRect[largestComp].width;
        int mwidth = boundRect[largestComp].x + 0.5*boundRect[largestComp].width;
        int rwidth = boundRect[largestComp].x + 0.66*boundRect[largestComp].width;

        if(!blue_hue_image.at<uchar>(sample_height,lwidth) ||
                !blue_hue_image.at<uchar>(sample_height,lwidth-1) ||
                !blue_hue_image.at<uchar>(sample_height,lwidth+1)) {
            frames_since_no_left = 0;
            frames_since_no_forward = 0;
            if(frames_since_no_right < 1) {
                cout <<"---First blue found (" << maxWeight<< ")\n";
                frames_since_no_right++;
            }else{
                frames_since_no_right++;
                result.size = maxWeight;
                cout << "---TURN RIGHT! (" << result.size << ")\n";
                result.type = rturn;
                result.tl_x = boundRect[largestComp].x;
                result.tl_y = boundRect[largestComp].y;
                result.w = boundRect[largestComp].width;
                result.h = boundRect[largestComp].height;
            }
        } else if(!blue_hue_image.at<uchar>(sample_height,mwidth) ||
                !blue_hue_image.at<uchar>(sample_height,mwidth-1) ||
                !blue_hue_image.at<uchar>(sample_height,mwidth+1)) {
            frames_since_no_left = 0;
            frames_since_no_right = 0;
            if(frames_since_no_forward < 1) {
                cout <<"---First blue found (" << maxWeight<< ")\n";
                frames_since_no_forward++;
            }else{
                frames_since_no_forward++;
                result.size = maxWeight;
                cout << "---STRAIGHT AHEAD! (" << result.size << ")\n";
                result.type = fturn;
                result.tl_x = boundRect[largestComp].x;
                result.tl_y = boundRect[largestComp].y;
                result.w = boundRect[largestComp].width;
                result.h = boundRect[largestComp].height;
            }
        } else if(!blue_hue_image.at<uchar>(sample_height,rwidth) ||
                !blue_hue_image.at<uchar>(sample_height,rwidth-1) ||
                !blue_hue_image.at<uchar>(sample_height,rwidth+1)) {
            frames_since_no_forward = 0;
            frames_since_no_right = 0;
            if(frames_since_no_left < 1) {
                frames_since_no_left++;
                cout <<"---First blue found (" << maxWeight<< ")\n";
            }else{
                frames_since_no_left++;
                result.size = maxWeight;
                cout << "---TURN LEFT (" << result.size << ")\n";
                result.type = lturn;
                result.tl_x = boundRect[largestComp].x;
                result.tl_y = boundRect[largestComp].y;
                result.w = boundRect[largestComp].width;
                result.h = boundRect[largestComp].height;
            }
        } else {
            cout <<"---Weird blue found (" << maxWeight<< ")\n";
            result.type = none;
            result.size = maxWeight;
            result.tl_x = boundRect[largestComp].x;
            result.tl_y = boundRect[largestComp].y;
            result.w = boundRect[largestComp].width;
            result.h = boundRect[largestComp].height;
        }

        /// Draw polygonal contour + bonding rects + circles (debug)
        Scalar color = Scalar(255,200*(result.type==lturn)+255*(result.type==none),200*(result.type==rturn)+255*(result.type==none));
        drawContours( _g_final_destination, contours_poly, largestComp, color, 1, 8, vector<Vec4i>(), 0, Point() );
        rectangle( _g_final_destination, boundRect[largestComp].tl(), boundRect[largestComp].br(), color, 2, 8, 0 );
        circle( _g_final_destination, center[largestComp], (int)radius[largestComp], color, 2, 8, 0 );
    }

    //imshow("blue_temp", temp);
    //imshow("blue_all", blue_hue_image);
    imshow("orig", print1);
    imshow("erod", print2);
    imshow("dila", print3);

    return result;
}


Sign VisionToolkit::edge_detect_red_sign(Mat &red_hue_image) {
    Sign result;
    result.type = stopsign;
    result.size = 0; 

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    Mat temp = red_hue_image.clone();

    erode(temp, temp, getStructuringElement(MORPH_RECT, Size(2,2)), Point(-1,-1),1);
    Mat print1 = temp.clone();
    dilate(temp, temp, getStructuringElement(MORPH_RECT, Size(3,3)), Point(-1,-1),4);
    Mat print2 = temp.clone();

    findContours( temp, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE );

    cvtColor(temp,temp, CV_GRAY2BGR);

    vector<vector<Point> > contours_poly( contours.size() );
    vector<Rect> boundRect( contours.size() );
    vector<Point2f>center( contours.size() );
    vector<float>radius( contours.size() );
    vector<vector<Point> > hullsP(contours.size());

    if( contours.size() == 0 ) {
        cout << "---no red contours found...\n";
        result.type = none;
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
        result.type = none;
        frames_since_no_red = 0;
        cout << "---No red Octagons found\n";
    } else if (maxWeight < 200) {
        result.type = none;
        frames_since_no_red = 0;
        cout << "---Maybe stop? Tiny red octagon found (" << maxWeight<< ")\n";
    } else if (frames_since_no_red < 0) { //disable needing two frames: the stop sign (never?) gives false positives
        frames_since_no_red++;
        cout << "---First red octagon (" << maxWeight<< ")\n";
    } else {
        result.type = stopsign;
        frames_since_no_red++;
        cout << "---STOP (" << maxWeight<< ")\n";
        result.size = maxWeight; 

        /// Draw polygonal contour + bonding rects + circles (debug)
        Scalar color = Scalar(0,0,255);
        drawContours( _g_final_destination, contours_poly, largestComp, color, 1, 8, vector<Vec4i>(), 0, Point() );
        rectangle( _g_final_destination, boundRect[largestComp].tl(), boundRect[largestComp].br(), color, 2, 8, 0 );
        circle( _g_final_destination, center[largestComp], (int)radius[largestComp], color, 2, 8, 0 );
    }
    //imshow("red_temp", temp);
    //imshow("rpri1", print1);
    //imshow("rpri2", print2);
    return result;
}

Sign VisionToolkit::edge_detect_yellow_sign(Mat &yellow_hue_image) {
    Sign result;
    result.type = uturn;
    result.size = 0; 

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    Mat temp;

    erode(yellow_hue_image, temp, getStructuringElement(MORPH_RECT, Size(3,3)), Point(-1,-1),1); //TODO check wether (2,2) ,.., 4 is better
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
        result.type = none;
        return result;
    }

    int idx = 0, largestComp = -1;
    int maxWeight = 0;
    RotatedRect temp_rect, big_rect; 
    for( ; idx >= 0; idx = hierarchy[idx][0] )
    {
        temp_rect = minAreaRect(contours[idx]);
        approxPolyDP( Mat(contours[idx]), contours_poly[idx], 3, true );
        if(temp_rect.angle > -55 && temp_rect.angle < -35 
                && temp_rect.size.width > temp_rect.size.height*0.75
                && temp_rect.size.width < temp_rect.size.height*1.33) { //in case of bubbly edge
            //cout << "w: " <<temp_rect.size.width<< ", h: " << temp_rect.size.height << endl;
            boundRect[idx] = boundingRect( Mat(contours_poly[idx]) );
            minEnclosingCircle( (Mat)contours_poly[idx], center[idx], radius[idx] );
            int weight = 3*temp_rect.size.height+3*temp_rect.size.width;
            if( weight > maxWeight )
            {
                maxWeight = weight;
                largestComp = idx;
                big_rect = temp_rect;
            }
        }
    }
    if(maxWeight == 0) {
        result.type = none;
        frames_since_no_yellow = 0;
        cout << "---No yellow squares found\n";
    } else if (maxWeight < 185) {
        result.type = none;
        frames_since_no_yellow = 0;
        cout <<"---Maybe uturn? Tiny yellow square found (" << maxWeight<< ")\n";
    } else if (frames_since_no_yellow < 2) {
        result.type = none;
        frames_since_no_yellow++;
        cout <<"---Maybe uturn? First yellow square found (" << maxWeight<< ")\n";
    } else {
        result.type = uturn;
        frames_since_no_yellow++;
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

Lane VisionToolkit::lane_detect(Mat &dark_image) {
    double lane_tic = omp_get_wtime();
    double lane_toc;
    Lane result;
    //defvals
    result.tl_y = result.w = result.h = 0;
    result.tl_x = 50;
    result.type = lrf;

    dilate(dark_image, dark_image, getStructuringElement(MORPH_RECT, Size(2,2)), Point(-1,-1),2);
    erode(dark_image, dark_image, getStructuringElement(MORPH_RECT, Size(2,2)), Point(-1,-1),3);
    dilate(dark_image, dark_image, getStructuringElement(MORPH_RECT, Size(2,2)), Point(-1,-1),1);

    lane_tic = lane_tic + 0; //silence lint warnings whenever printing is disabled
    lane_toc = omp_get_wtime();
    //cout << "lane 1: " << setprecision(7) << lane_toc-lane_tic << endl;

    Mat dark_processed = dark_image.clone();
    Mat dark_col;
    cvtColor(dark_processed, dark_col, CV_GRAY2BGR);

    lane_toc = omp_get_wtime();
    //cout << "lane 2: " << setprecision(7) << lane_toc-lane_tic << endl;

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    findContours(dark_processed, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE );

    lane_toc = omp_get_wtime();
    //cout << "lane 4: " << setprecision(7) << lane_toc-lane_tic << endl;

    vector<vector<Point> > contours_poly( contours.size() );
    vector<Rect> boundRect( contours.size() );
    vector<RotatedRect> rotRect( contours.size() );
    vector<Point2f>center( contours.size() );
    vector<float>radius( contours.size() );
    vector<int> possible_left;
    vector<int> possible_right;


    if( contours.size() == 0 ) {
        cout << "---no dark spots found???\n";
        result.size = 0;
        return result;
    }

    
    RotatedRect temp_rect, big_rect; 
    for(int idx = 0; idx >= 0; idx = hierarchy[idx][0] )
    {
        rotRect[idx] = minAreaRect(contours[idx]);
        approxPolyDP( Mat(contours[idx]), contours_poly[idx], 3, true );
        boundRect[idx] = boundingRect( Mat(contours_poly[idx]) );
        int weight = rotRect[idx].size.height+rotRect[idx].size.width;
        if(weight > 81 && rotRect[idx].size.width > 9 && rotRect[idx].size.height > 9) {
            cout << "w,h: " << rotRect[idx].size.width << ", " << rotRect[idx].size.height << endl;
            if( boundRect[idx].x < 2 || (boundRect[idx].y + boundRect[idx].height > 238 && boundRect[idx].x < 144)){
                possible_left.push_back(idx);
            }
            if( boundRect[idx].x+boundRect[idx].width > 430 || (boundRect[idx].y + boundRect[idx].height > 238 && boundRect[idx].x+boundRect[idx].width > 288)){
                possible_right.push_back(idx);
            }
            Scalar color = Scalar(rand()%155+100,rand()%155+100,rand()%155+100);
            drawContours( dark_col, contours_poly, idx, color, 1, 8, vector<Vec4i>(), 0, Point() );
            drawContours( _g_final_destination, contours_poly, idx, color, 1, 8, vector<Vec4i>(), 0, Point(0, Y_BOTRECT-3) );
            ostringstream num;
            num << idx;
            putText(dark_col, num.str(), Point(boundRect[idx].x, boundRect[idx].y+30), FONT_HERSHEY_SIMPLEX, 0.3, color);
        }
    }

    lane_toc = omp_get_wtime();
    //cout << "lane 5: " << setprecision(7) << lane_toc-lane_tic << endl;

    // select the most important left/right contour
    int maxH=10, lId=-1, rId=-1;
    for(int i=0; i < (int) possible_left.size(); i++){
        if(boundRect[possible_left[i]].y+boundRect[possible_left[i]].height > maxH) {
            maxH = boundRect[possible_left[i]].y+boundRect[possible_left[i]].height;
            lId = possible_left[i];
        }
    }
    maxH=0;
    for(int i=0; i < (int) possible_right.size(); i++){
        if(boundRect[possible_right[i]].y+boundRect[possible_right[i]].height > maxH) {
            maxH = boundRect[possible_right[i]].y+boundRect[possible_right[i]].height;
            rId = possible_right[i];
        }
    }

    lane_toc = omp_get_wtime();
    //cout << "lane 6: " << setprecision(7) << lane_toc-lane_tic << endl;
 
    vector<vector<Point> > lhull(1);
    vector<vector<Point> > rhull(1);
    Moments moml, momr;
    bool l_in_way = false, r_in_way = false, l_in_long_way = false, r_in_long_way = false;
    if(lId!=-1) {
        convexHull( Mat(contours[lId]), lhull[0], false );
        //moml = moments(lhull[0]);
        moml = moments(contours[lId]);
        l_in_way = (pointPolygonTest(contours[lId], Point(215,BOT_HEIGHT-20), false) > 0 ||
                    pointPolygonTest(contours[lId], Point(215,BOT_HEIGHT-5), false) > 0 ||
                    pointPolygonTest(contours[lId], Point(180,BOT_HEIGHT-15), false) > 0 ||
                    pointPolygonTest(contours[lId], Point(180,BOT_HEIGHT-5), false) > 0 ||
                    pointPolygonTest(contours[lId], Point(250,BOT_HEIGHT-15), false) > 0 ||
                    pointPolygonTest(contours[lId], Point(250,BOT_HEIGHT-5), false) > 0 );
        l_in_long_way = (pointPolygonTest(contours[lId], Point(215,BOT_HEIGHT-45), false) > 0 ||
                    pointPolygonTest(contours[lId], Point(144,BOT_HEIGHT-35), false) > 0 ||
                    pointPolygonTest(contours[lId], Point(288,BOT_HEIGHT-35), false) > 0 );
        Scalar color = Scalar(0,255,0);
        drawContours( _g_final_destination, lhull, 0, color, 1, 8, vector<Vec4i>(), 0, Point(0, Y_BOTRECT-3) );
        drawContours( _g_final_destination, contours_poly, lId, color, 2, 8, vector<Vec4i>(), 0, Point(0, Y_BOTRECT-3) );
        drawContours( dark_col, contours_poly, lId, color, 3, 8, vector<Vec4i>(), 0, Point() );
        //draw _in_way markers
        //line( _g_final_destination, Point(215,Y_BOTRECT + BOT_HEIGHT-20),Point(215,Y_BOTRECT + BOT_HEIGHT-5), Scalar(0,155,0), 3, CV_AA);
        //line( _g_final_destination, Point(215,Y_BOTRECT + BOT_HEIGHT-20),Point(180,Y_BOTRECT + BOT_HEIGHT-15), Scalar(0,155,0), 3, CV_AA);
        //line( _g_final_destination, Point(180,Y_BOTRECT + BOT_HEIGHT-5),Point(180,Y_BOTRECT + BOT_HEIGHT-15), Scalar(0,155,0), 3, CV_AA);
        //line( _g_final_destination, Point(215,Y_BOTRECT + BOT_HEIGHT-20),Point(250,Y_BOTRECT + BOT_HEIGHT-15), Scalar(0,155,0), 3, CV_AA);
        //line( _g_final_destination, Point(250,Y_BOTRECT + BOT_HEIGHT-5),Point(250,Y_BOTRECT + BOT_HEIGHT-15), Scalar(0,155,0), 3, CV_AA);
        //line( _g_final_destination, Point(215,Y_BOTRECT + BOT_HEIGHT-45),Point(288,Y_BOTRECT + BOT_HEIGHT-35), Scalar(0,155,0), 3, CV_AA);
        //line( _g_final_destination, Point(215,Y_BOTRECT + BOT_HEIGHT-45),Point(144,Y_BOTRECT + BOT_HEIGHT-35), Scalar(0,255,0), 3, CV_AA);
        //line( _g_final_destination, Point(215,Y_BOTRECT + BOT_HEIGHT-45),Point(288,Y_BOTRECT + BOT_HEIGHT-35), Scalar(0,255,0), 3, CV_AA);
    }
    if(rId!=-1) {
        //convexHull( Mat(contours[rId]), rhull[0], false );
        //momr = moments(rhull[0]);
        momr = moments(contours[rId]);
        r_in_way = (pointPolygonTest(contours[rId], Point(215,BOT_HEIGHT-20), false) > 0 ||
                    pointPolygonTest(contours[rId], Point(215,BOT_HEIGHT-5), false) > 0 ||
                    pointPolygonTest(contours[rId], Point(180,BOT_HEIGHT-15), false) > 0 ||
                    pointPolygonTest(contours[rId], Point(180,BOT_HEIGHT-5), false) > 0 ||
                    pointPolygonTest(contours[rId], Point(250,BOT_HEIGHT-15), false) > 0 ||
                    pointPolygonTest(contours[rId], Point(250,BOT_HEIGHT-5), false) > 0 );
        r_in_long_way = (pointPolygonTest(contours[rId], Point(215,BOT_HEIGHT-45), false) > 0 ||
                    pointPolygonTest(contours[rId], Point(144,BOT_HEIGHT-35), false) > 0 ||
                    pointPolygonTest(contours[rId], Point(288,BOT_HEIGHT-35), false) > 0 );
        Scalar color = Scalar(0,0,255);
        drawContours( _g_final_destination, rhull, 0, color, 1, 8, vector<Vec4i>(), 0, Point(0, Y_BOTRECT-3) );
        drawContours( _g_final_destination, contours_poly, rId, color, 2, 8, vector<Vec4i>(), 0, Point(0, Y_BOTRECT-3) );
        drawContours( dark_col, contours_poly, rId, color, 3, 8, vector<Vec4i>(), 0, Point() );
        //draw _in_way markers
        //line( _g_final_destination, Point(215,Y_BOTRECT + BOT_HEIGHT-20),Point(215,Y_BOTRECT + BOT_HEIGHT-5), Scalar(100,255,100), 3, CV_AA);
        //line( _g_final_destination, Point(215,Y_BOTRECT + BOT_HEIGHT-20),Point(180,Y_BOTRECT + BOT_HEIGHT-15), Scalar(100,255,100), 3, CV_AA);
        //line( _g_final_destination, Point(180,Y_BOTRECT + BOT_HEIGHT-5),Point(180,Y_BOTRECT + BOT_HEIGHT-15), Scalar(100,255,100), 3, CV_AA);
        //line( _g_final_destination, Point(215,Y_BOTRECT + BOT_HEIGHT-20),Point(250,Y_BOTRECT + BOT_HEIGHT-15), Scalar(100,255,100), 3, CV_AA);
        //line( _g_final_destination, Point(250,Y_BOTRECT + BOT_HEIGHT-5),Point(250,Y_BOTRECT + BOT_HEIGHT-15), Scalar(100,255,100), 3, CV_AA);
        //line( _g_final_destination, Point(215,Y_BOTRECT + BOT_HEIGHT-45),Point(288,Y_BOTRECT + BOT_HEIGHT-35), Scalar(100,255,100), 3, CV_AA);
        //line( _g_final_destination, Point(215,Y_BOTRECT + BOT_HEIGHT-45),Point(144,Y_BOTRECT + BOT_HEIGHT-35), Scalar(0,255,0), 3, CV_AA);
        //line( _g_final_destination, Point(215,Y_BOTRECT + BOT_HEIGHT-45),Point(288,Y_BOTRECT + BOT_HEIGHT-35), Scalar(0,255,0), 3, CV_AA);
    }

    lane_toc = omp_get_wtime();
    //cout << "lane 7: " << setprecision(7) << lane_toc-lane_tic << endl;

    if(lId == rId) {
        if(lId == -1) {
            result.type = lrf;
            line( _g_final_destination, Point(X_PIX/2, 1),Point(X_PIX/2, X_PIX-1), Scalar(200,200,255), 3, CV_AA);
        } else if(l_in_way) {
            result.type = dead;
        } else {
            int lane_x_coord = X_PIX-1 - momr.m10/momr.m00;
            int lane_y_coord = momr.m01/momr.m00;
            int lane_x_normalized = (lane_x_coord*100)/X_PIX; 
            result.tl_y = ((BOT_HEIGHT-lane_y_coord)*100)/BOT_HEIGHT;
            line( _g_final_destination, Point(lane_x_coord, 1),Point(lane_x_coord, X_PIX-1), Scalar(200,200,255), 3, CV_AA);
            line( _g_final_destination, Point(1, lane_y_coord+Y_BOTRECT),Point(X_PIX-1, lane_y_coord+Y_BOTRECT), Scalar(100,100,155), 2, CV_AA);
            if (lane_x_coord < 170) {
                if (r_in_long_way) {
                    result.type = l;
                    result.tl_x = lane_x_normalized + 5;
                } else {
                    result.type = lf;
                    result.tl_x = lane_x_normalized;
                }
            } else if (lane_x_coord > 250) {
                if (r_in_long_way) {
                    result.type = r;
                    result.tl_x = lane_x_normalized - 5;
                } else {
                    result.type = rf;
                    result.tl_x = lane_x_normalized;
                }
            } else {
                result.tl_x = lane_x_normalized;
                if (r_in_long_way) { 
                    result.type = dead;
                } else {
                    result.type = f;
                }
            }
        }
    } else if (lId == -1) {
        int lane_x_coord = momr.m10/momr.m00 / 2;
        int lane_y_coord = momr.m01/momr.m00;
        int lane_x_normalized = (lane_x_coord*100)/X_PIX;
        result.tl_y = ((BOT_HEIGHT-lane_y_coord)*100)/BOT_HEIGHT;
        line( _g_final_destination, Point(lane_x_coord, 1),Point(lane_x_coord, X_PIX-1), Scalar(200,200,255), 3, CV_AA);
        line( _g_final_destination, Point(1, lane_y_coord+Y_BOTRECT),Point(X_PIX-1, lane_y_coord+Y_BOTRECT), Scalar(100,100,155), 2, CV_AA);
        if (!r_in_long_way) {
            result.type = lf;
            result.tl_x = lane_x_normalized;
        } else {
            result.type = l;
            result.tl_x = lane_x_normalized + 5;
        }
        if(r_in_way) {
            result.type = dead;
        }

    } else if (rId == -1) {
        int lane_x_coord = (moml.m10/moml.m00 + X_PIX-1) / 2;
        int lane_y_coord = moml.m01/moml.m00;
        int lane_x_normalized = (lane_x_coord*100)/X_PIX;
        result.tl_y = ((BOT_HEIGHT-lane_y_coord)*100)/BOT_HEIGHT;
        line( _g_final_destination, Point(lane_x_coord, 1),Point(lane_x_coord, X_PIX-1), Scalar(200,200,255), 3, CV_AA);
        line( _g_final_destination, Point(1, lane_y_coord+Y_BOTRECT),Point(X_PIX-1, lane_y_coord+Y_BOTRECT), Scalar(100,100,155), 2, CV_AA);
        if (!r_in_long_way) {
            result.type = rf;
            result.tl_x = lane_x_normalized;
        } else {
            result.type = r;
            result.tl_x = lane_x_normalized - 5;
        }
        if(l_in_way) {
            result.type = dead;
        }

    } else {
        int lane_y_coord = (momr.m01/momr.m00 + moml.m01/moml.m00) / 2;
        float l_dominance_factor = (moml.m01/moml.m00 - momr.m01/momr.m00) / BOT_HEIGHT; 
        cout << "left dominance: " << l_dominance_factor << endl;
        int lane_x_coord = (momr.m10/momr.m00 + moml.m10/moml.m00) / 2;
        int lane_x_normalized = ((1+l_dominance_factor)*momr.m10/momr.m00 + (1-l_dominance_factor)*moml.m10/moml.m00)/2 *100 / X_PIX;
        if (boundRect[lId].width > boundRect[rId].x) {
            cout << "lx_norm: " << lane_x_normalized;
            if (momr.m01/momr.m00 > moml.m01/moml.m00) {
                lane_x_normalized -= (boundRect[lId].width - boundRect[rId].x)*100/X_PIX;
            } else {
                lane_x_normalized += (boundRect[lId].width - boundRect[rId].x)*100/X_PIX;
            }
            cout << ", lx_norm_n: " << lane_x_normalized << endl;
            cout << "l_w-r_x: " << boundRect[lId].width << ", " << boundRect[rId].y << endl;
        }
        result.tl_y = ((BOT_HEIGHT-lane_y_coord)*100)/BOT_HEIGHT;
        cout << "lx_norm: " << lane_x_normalized << endl;
        line( _g_final_destination, Point(lane_x_normalized*4.32, 1),Point(lane_x_normalized*4.32, X_PIX-1), Scalar(200,200,255), 3, CV_AA);
        line( _g_final_destination, Point(1, lane_y_coord+Y_BOTRECT),Point(X_PIX-1, lane_y_coord+Y_BOTRECT), Scalar(100,100,155), 2, CV_AA);
        if (lane_x_coord < 170) {
            if (r_in_long_way || l_in_long_way) {
                result.type = l;
                result.tl_x = lane_x_normalized + 5;
            } else {
                result.type = lf;
            result.tl_x = lane_x_normalized;
            }
        } else if (lane_x_coord > 250) {
            if (r_in_long_way || l_in_long_way) {
                result.type = r;
                result.tl_x = lane_x_normalized - 5;
            } else {
                result.type = rf;
                result.tl_x = lane_x_normalized;
            }
        } else {
            if (r_in_long_way || l_in_long_way) { 
                result.type = dead;
                result.tl_x = lane_x_normalized;
                cout << "\nseemingly blocked, shadow?\n";
            } else {
                result.type = f;
                result.tl_x = lane_x_normalized;
            }
        }
        if(l_in_way || r_in_way) {
            result.type = dead;
        }
    }
    switch(result.type) {
        case l:
            putText(_g_final_destination, "L", Point(30, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(255,255,255), 6);
            putText(_g_final_destination, "L", Point(30, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,20), 2);
            break;
        case r:
            putText(_g_final_destination, "R", Point(30, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(255,255,255), 6);
            putText(_g_final_destination, "R", Point(30, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,20), 2);
            break;
        case lr:
            putText(_g_final_destination, "LR", Point(30, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(255,255,255), 6);
            putText(_g_final_destination, "LR", Point(30, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,20), 2);
            break;
        case lrf:
            putText(_g_final_destination, "LRF", Point(30, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(255,255,255), 6);
            putText(_g_final_destination, "LRF", Point(30, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,20), 2);
            break;
        case rf:
            putText(_g_final_destination, "RF", Point(30, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(255,255,255), 6);
            putText(_g_final_destination, "RF", Point(30, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,20), 2);
            break;
        case lf:
            putText(_g_final_destination, "LF", Point(30, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(255,255,255), 6);
            putText(_g_final_destination, "LF", Point(30, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,20), 2);
            break;
        case dead:
            putText(_g_final_destination, "SONAR (turn or f)", Point(30, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(255,255,255), 6);
            putText(_g_final_destination, "SONAR (turn or f)", Point(30, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,20), 2);
            break;
        case f:
            putText(_g_final_destination, "F", Point(30, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(255,255,255), 6);
            putText(_g_final_destination, "F", Point(30, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,20), 2);
            break;
        case in_corner:
            putText(_g_final_destination, "SONAR (turn or f)", Point(30, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(255,255,255), 6);
            putText(_g_final_destination, "SONAR (turn or f)", Point(30, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,20), 2);
            break;
        default:
            putText(_g_final_destination, "IETS FAALT", Point(30, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(255,255,255), 6);
            putText(_g_final_destination, "IETS FAALT", Point(30, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,20), 2);
            cout << "er faalt iets\n";
    }

    lane_toc = omp_get_wtime();
    //cout << "lane 8: " << setprecision(7) << lane_toc-lane_tic << endl;

    //imshow("dark_col", dark_col);

    cout << "tl_x: " << result.tl_x << endl;
    return result;
}

void VisionToolkit::initSignMems() {
    frames_since_no_yellow=0;
    frames_since_no_red=0;
    frames_since_no_left=0;
    frames_since_no_right=0;
    frames_since_no_forward=0;
}

void VisionToolkit::calibGrey() {
    Mat hsv_image;
    cvtColor(src_col, hsv_image, cv::COLOR_BGR2HSV);
    Rect botcroprect(0,Y_BOTRECT,X_PIX,BOT_HEIGHT);
    Mat botcrop, dark_image;
    hsv_image(botcroprect).copyTo(botcrop);
    maxgreyval = DEFAULT_GREYVAL;
    inRange(botcrop, cv::Scalar(0, 0, 0), cv::Scalar(255, 180, maxgreyval), dark_image);
    while(countNonZero(dark_image) < X_PIX*BOT_HEIGHT/40) {
        maxgreyval += 5;
        inRange(botcrop, cv::Scalar(0, 0, 0), cv::Scalar(255, 180, maxgreyval), dark_image);
    }
    while(countNonZero(dark_image) > X_PIX*BOT_HEIGHT/8) {
        maxgreyval -= 5;
        inRange(botcrop, cv::Scalar(0, 0, 0), cv::Scalar(255, 180, maxgreyval), dark_image);
    }
}


VisionToolkit::VisionToolkit(char *file) {
    initSignMems();
    if(!_cap2.open(file))
        cout << "opening file failed!\n";
    for (int i=0; i<50; i++) //skip first few frames
        _cap2.read(src_col);
    //calibGrey();
    maxgreyval = DEFAULT_GREYVAL;
    save_frame = false;
}

VisionToolkit::VisionToolkit() {
    initSignMems();
    cout << "opening webcam";
	
	
    _cap.set( CV_CAP_PROP_FORMAT, CV_8UC3 );

    _cap.set( CV_CAP_PROP_FRAME_WIDTH, X_PIX);
    _cap.set( CV_CAP_PROP_FRAME_HEIGHT, Y_PIX);
    _cap.set( CV_CAP_PROP_EXPOSURE, 20);
    _cap.set( CV_CAP_PROP_GAIN, 100);

    cout<<"Opening Camera..."<<endl;
    if (!_cap.open()) {cerr<<"Error opening the camera"<<endl;}

	
	/*
opnieuw:
    if(!_cap.open()) {
        if(!_cap.open()) {
            cout << '.';
            goto opnieuw;
        } else {    
			_cap.set( CV_CAP_PROP_FORMAT, CV_8UC3 );

			_cap.set( CV_CAP_PROP_FRAME_WIDTH, X_PIX);
			_cap.set( CV_CAP_PROP_FRAME_HEIGHT, Y_PIX);
			_cap.set( CV_CAP_PROP_EXPOSURE, 20);
			_cap.set( CV_CAP_PROP_GAIN, 100);

            cout << endl;
        }
    } else {    
			_cap.set( CV_CAP_PROP_FORMAT, CV_8UC3 );

			_cap.set( CV_CAP_PROP_FRAME_WIDTH, X_PIX);
			_cap.set( CV_CAP_PROP_FRAME_HEIGHT, Y_PIX);
			_cap.set( CV_CAP_PROP_EXPOSURE, 20);
			_cap.set( CV_CAP_PROP_GAIN, 100);
        cout << endl;
    }*/
    for (int i=0; i<50; i++){ //warmup frames
		_cap.grab();
        _cap.retrieve (src_col);
	}
    //calibGrey();
    maxgreyval = DEFAULT_GREYVAL;
    save_frame = true;
}

int VisionToolkit::process_frame()
{
    double main_tic = omp_get_wtime();
    double main_toc;
    
    for(int i=0;i<9;i++) {
        main_tic = omp_get_wtime();
        _cap.grab();
        _cap.retrieve (src_col);
        waitKey(3);
    }
    ostringstream filename;
    filename << "caps/" << std::time(0) << "_" << omp_get_wtime() << ".jpg";
    if(save_frame) imwrite(filename.str(), src_col);

    main_toc = omp_get_wtime();
    cout << "(main cuml) load rgb image: " << setprecision(7) << main_toc-main_tic << endl;
    cout << "\nWxH: " << src_col.cols << ", " << src_col.rows << endl;
    _g_final_destination = src_col;
    Mat hsv_image;
    cvtColor(src_col, hsv_image, cv::COLOR_BGR2HSV);

    main_toc = omp_get_wtime();
    cout << "(main cuml) hsv converted: " << setprecision(7) << main_toc-main_tic << endl;

    Mat lower_red_hue_range;
    Mat upper_red_hue_range;
    Mat red_hue_image;
    Mat blue_hue_image;
    Mat yellow_hue_image;
    Mat dark_image;
 
    Rect topcroprect(0,0,X_PIX,Y_BOTRECT);
    Rect botcroprect(0,Y_BOTRECT,X_PIX,BOT_HEIGHT);
    line( _g_final_destination, Point(1, Y_BOTRECT),Point(X_PIX, Y_BOTRECT), Scalar(0,0,0), 2, CV_AA);
    
    Mat topcrop, botcrop;
    hsv_image(topcroprect).copyTo(topcrop);
    hsv_image(botcroprect).copyTo(botcrop);

    inRange(topcrop, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red_hue_range);
    inRange(topcrop, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), upper_red_hue_range);
    addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);
    inRange(topcrop, cv::Scalar(102, 110, 65), cv::Scalar(120, 255, 255), blue_hue_image);
    inRange(topcrop, cv::Scalar(15, 100, 100), cv::Scalar(27, 255, 255), yellow_hue_image);

#if 0 //disable HISTOEQ, bigger problems than it fixes
    //histeq stuff
    cvtColor( botcrop, botcrop, COLOR_HSV2BGR);
    cvtColor( botcrop, botcrop, COLOR_BGR2GRAY );
    equalizeHist( botcrop, botcrop);
    cvtColor( botcrop, botcrop, CV_GRAY2BGR);
    cvtColor(src_col, hsv_image, cv::COLOR_BGR2HSV);
#endif
    inRange(botcrop, cv::Scalar(0, 0, 0), cv::Scalar(255, 180, maxgreyval), dark_image);
    
    cout << "\n\nNum nonzero: " << countNonZero(dark_image) << endl;

    main_toc = omp_get_wtime();
    cout << "(main cuml) split images to hsv ranges: " << setprecision(7) << main_toc-main_tic << endl;

    blue_sign = edge_detect_blue_sign(blue_hue_image);
    red_sign = edge_detect_red_sign(red_hue_image);
    yellow_sign = edge_detect_yellow_sign(yellow_hue_image);

    main_toc = omp_get_wtime();
    cout << "(main cuml) detected signs: " << setprecision(7) << main_toc-main_tic << endl;
    
    current_lane = lane_detect(dark_image);

    main_toc = omp_get_wtime();
    cout << "(main cuml) full_program_timing: " << setprecision(7) << main_toc-main_tic << endl;

    //imshow("blue", blue_hue_image);
    //imshow("red", red_hue_image);
    //imshow("yellow", yellow_hue_image); 
    imshow("Final destination", _g_final_destination); 
    ostringstream filename_fd;
    filename_fd << "caps/" << std::time(0) << "_" << omp_get_wtime() << "_fd.jpg";
    if(save_frame) imwrite(filename_fd.str(), src_col);
    waitKey(3);
    if ((waitKey(20) & 0xEFFFFF) == 27) {
        for(int i=0;i<180;i++){
            _cap.grab();
			_cap.retrieve (src_col);
            waitKey(1);
        }
    }
    return 0;
}
