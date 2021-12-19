
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        // pixel coordinates
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0); 
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0); 

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}

/* 
* The show3DObjects() function below can handle different output image sizes, but the text output has been manually tuned to fit the 2000x2000 size. 
* However, you can make this function work for other sizes too.
* For instance, to use a 1000x1000 size, adjusting the text positions by dividing them by 2.
*/
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::resize(topviewImg, topviewImg, cv::Size(1000, 1000));
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    // In this implementation, keypoints and keypoint matches are already appended to the vector in the function `matchBoundingBoxes`
    // Therefore, this function only conducts an outlier filtering
    std::vector<double> distances;

    for (auto it = boundingBox.kptMatches.begin(); it != boundingBox.kptMatches.end(); ++it)
    {
        cv::KeyPoint kptCurr = kptsCurr.at(it->trainIdx);
        cv::KeyPoint kptPrev = kptsPrev.at(it->queryIdx);

        double dist = cv::norm(kptCurr.pt - kptPrev.pt);
        distances.push_back(dist);
    }

    std::sort(distances.begin(), distances.end());
    long medIndex = floor(distances.size() / 2.0);
    double medDist = distances.size() % 2 == 0 ? (distances[medIndex - 1] + distances[medIndex]) / 2.0 : distances[medIndex];
    double tolRate = 3.0;

    for (auto it = boundingBox.kptMatches.begin(); it != boundingBox.kptMatches.end(); ++it)
    {
        cv::KeyPoint kptCurr = kptsCurr.at(it->trainIdx);
        cv::KeyPoint kptPrev = kptsPrev.at(it->queryIdx);

        double dist = cv::norm(kptCurr.pt - kptPrev.pt);
        if (abs(dist - medDist) > tolRate * medDist)
        {
            cout << "Median: " << medDist << ", distance: " << dist << ", filtered" << endl;
            boundingBox.kptMatches.erase(it--);
        }
        else
        {
            boundingBox.keypoints.push_back(kptCurr);
        }
    }
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // ...
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // auxiliary variables
    double dT = 1.0 / frameRate;        // time between two measurements in seconds
    double laneWidth = 4.0;             // assumed width of the ego lane

    double num_sigma = 0.3;
    double minXPrev = 1e9, minXCurr = 1e9;

    // previous frame
    std::vector<double> xPrev;
    for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
    {
        xPrev.push_back(it->x);
    }

    double meanPrev = accumulate(xPrev.begin(), xPrev.end(), 0.0) / xPrev.size();
    auto add_square_prev = [meanPrev](double sum, int i)
    {
        auto d = i - meanPrev;
        return sum + d*d;
    };
    double totalPrev = accumulate(xPrev.begin(), xPrev.end(), 0.0, add_square_prev);
    double variancePrev = sqrt(totalPrev / xPrev.size());

    // cout << "mean: " << meanPrev << ", variance: " << variancePrev << endl;

    for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
    {
        
        if ((abs(it->y) <= laneWidth / 2.0) && (abs(it->x - meanPrev) <= num_sigma * variancePrev))
        { // 3D point within ego lane? within 3 sigma?
            minXPrev = minXPrev > it->x ? it->x : minXPrev;
        }
    }

    // cout << "minXPrev: " << minXPrev << endl;

    // // current frame
    std::vector<double> xCurr;
    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {
        xCurr.push_back(it->x);
    }

    double meanCurr = accumulate(xCurr.begin(), xCurr.end(), 0.0) / xCurr.size();
    auto add_square_curr = [meanCurr](double sum, int i)
    {
        auto d = i - meanCurr;
        return sum + d*d;
    };
    double totalCurr = accumulate(xCurr.begin(), xCurr.end(), 0.0, add_square_curr);
    double varianceCurr = sqrt(totalCurr / xCurr.size());

    // cout << "mean: " << meanCurr << ", variance: " << varianceCurr << endl;

    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {
        
        if ((abs(it->y) <= laneWidth / 2.0) && (abs(it->x - meanCurr) <= num_sigma * varianceCurr))
        { // 3D point within ego lane? within 3 sigma?
            minXCurr = minXCurr > it->x ? it->x : minXCurr;
        }
    }

    // cout << "minXCurr: " << minXCurr << endl;

    // compute TTC from both measurements
    TTC = minXCurr * dT / (minXPrev - minXCurr);

}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{

    for (auto it_curr = currFrame.boundingBoxes.begin(); it_curr != currFrame.boundingBoxes.end(); ++it_curr)
    {

        for (auto it_match = matches.begin(); it_match != matches.end(); ++it_match)
        {
            // queryIdx for source (previous frame), trainIdx for reference (current frame)
            cv::KeyPoint kpt_curr = currFrame.keypoints.at(it_match->trainIdx);

            if (it_curr->roi.contains(kpt_curr.pt))
            {
                // it_curr->keypoints.push_back(kpt_curr);
                it_curr->kptMatches.push_back(*it_match);
            }
        }

        int max_matched_num = 0;
        int matched_prev_id = 1e8;

        // Loop over all *previous* bounding boxes
        for (auto it_prev = prevFrame.boundingBoxes.begin(); it_prev != prevFrame.boundingBoxes.end(); ++it_prev)
        {
            int matched_num = 0;
            // Loop over all matches inside *current* bounding box
            for (auto it_match = it_curr->kptMatches.begin(); it_match != it_curr->kptMatches.end(); ++it_match)
            {
                cv::KeyPoint kpt_prev = prevFrame.keypoints.at(it_match->queryIdx);
                if (it_prev->roi.contains(kpt_prev.pt))
                {
                    matched_num++;
                }
            }

            if (matched_num > max_matched_num)
            {
                max_matched_num = matched_num;
                matched_prev_id = it_prev->boxID;
            }
        }

        // Matched successfully
        if (matched_prev_id < 1e8)
        {
            // Todo: first current, second previous?
            bbBestMatches.insert(std::pair<int, int>(matched_prev_id, it_curr->boxID));
        }
    }

}
