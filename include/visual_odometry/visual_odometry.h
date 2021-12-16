#ifndef VISUAL_ODOMETRY_H
#define VISUAL_ODOMETRY_H

#include <vector>
#include <math.h>
#include <Eigen/Dense>

#include <iostream>
#include <fstream>
#include <opencv2/core.hpp>
//#ifdef HAVE_OPENCV_XFEATURES2D
//#include <opencv2/opencv.hpp>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/xfeatures2d.hpp>
//#include "home/name/opencv_contrib-master/xfeatures2d/include/opencv2/xfeatures2d.hpp"

using std::vector;

namespace visual_odometry
{

class StereoVO
{
public:
    StereoVO();
	~StereoVO();
    void camParam();//cv::Mat&,cv::Mat&,cv::Mat&);
    void process(cv::Mat&, const cv::Mat &img_disp);
    //void getDeltaPose();//pose diference from last iteration
    //void getPose();//pose since start


private:

    cv::Ptr<cv::FastFeatureDetector> detector_fast;
    cv::Ptr<cv::xfeatures2d::SURF> detector_surf;
    cv::Ptr<cv::SIFT> detector_sift;
    cv::Ptr<cv::ORB> detector_orb;
    std::vector<cv::KeyPoint> keypoints_curr, keypoints_prev;
    cv::Mat descriptors_curr, descriptors_prev;
    cv::Mat disp_prev;//disparity previous iteration

    int min_hessian = 400;

    cv::Ptr<cv::DescriptorMatcher> matcher;
    std::vector<cv::DMatch> matches;

    cv::Mat Q;
    cv::Mat M1,D1;
    cv::Mat R,T; //calculated Rotation and translation


    void calcDisp(cv::Mat&, cv::Mat&, cv::Mat&);
    //Eigen::Matrix<double,9,1> vo_state;
    //Eigen::Matrix<double,6,1> vo_cov;

    //void calcPose();
    //void calcDeltaPose();



};//class StereoVO

}//namespace
#endif
