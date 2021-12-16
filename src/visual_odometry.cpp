#include <visual_odometry/visual_odometry.h>

using std::vector;
using std::cout;
using std::cerr;
using std::endl;

namespace visual_odometry
{

StereoVO::StereoVO()
{
    detector_fast = cv::FastFeatureDetector::create();
    detector_surf = cv::xfeatures2d::SURF::create(min_hessian);
    detector_sift = cv::SIFT::create(20);
    detector_orb = cv::ORB::create(100);
    matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE);
    camParam();

}

StereoVO::~StereoVO(){}

void StereoVO::camParam()
{
    cv::FileStorage fs_l, fs_r;
    fs_l.open("/home/name/catkin_ws/src/dataset_pub/config/cam_calib_l.yml", cv::FileStorage::READ);
    fs_r.open("/home/name/catkin_ws/src/dataset_pub/config/cam_calib_r.yml", cv::FileStorage::READ);

    cv::Mat P1,P2,D1;

    fs_l["projection_matrix"] >> P1;
    fs_l["distortion_coefficients"] >> D1;
    fs_r["projection_matrix"] >> P2;

    cv::Mat R,T;
    cv::decomposeProjectionMatrix(P1,M1,R,T,cv::noArray(),cv::noArray(),cv::noArray(),cv::noArray());
    M1 = M1.clone();
    D1 = D1.clone();

    //cv::stereoRectify(M1,D1,M2,D2,image_size,R,T,R1,R2,P1,P2,Q )
    double cx = P1.at<double>(0,2);
    double cy = P1.at<double>(1,2);
    double f = P1.at<double>(0,0);
    double tx = P2.at<double>(0,3);
    double cx_r = P2.at<double>(0,2);

    double q[] =
    {
        1, 0, 0, -cx, //-cc_new[0].x,
        0, 1, 0, -cy, //-cc_new[0].y,
        0, 0, 0, f, //fc_new,
        0, 0, -1/tx, //-1./_t[idx],
        (cx - cx_r)/tx //(idx == 0 ? cc_new[0].x - cc_new[1].x : cc_new[0].y - cc_new[1].y)/_t[idx]
    };
    Q = cv::Mat(4, 4, CV_64F, q);//CHECK
    Q = Q.clone();
}

void StereoVO::process(cv::Mat& img1,const cv::Mat& img_disp)
{
    std::cout << "entrou" << std::endl;
    if (img1.rows > 60 && img1.cols > 60)
        cv::circle(img1, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    std::cout << "\nQ\n" << Q <<std::endl;

    //get keypoints on left camera
    ////std::vector<cv::KeyPoint> keypoints_curr;
    ////cv::Mat descriptors_curr;
    //detector_surf->detectAndCompute(img1,cv::noArray(),keypoints_curr,descriptors_curr);
/*    detector_surf->detect(img1, keypoints_curr);
    cv::KeyPointsFilter::retainBest(keypoints_curr,20);
    cv::DescriptorExtractor::
    detector_surf->compute(img1,keypoints_curr,descriptors_curr);
*/
    detector_orb->detectAndCompute(img1,cv::noArray(),keypoints_curr,descriptors_curr);
    //cv::Mat img_out;
    cv::drawKeypoints(img1,keypoints_curr,img1);
//    img_out.copyTo(img1);
/*    cv::KeyPointsFilter::retainBest(keypoints_curr,20);
    detector_fast->detect(img1,keypoints_curr,cv::noArray());
    detector_fast->compute(img1,keypoints_curr,descriptors_curr);
*/
    if ( keypoints_prev.size() != 0 )//if not first iteration
    {
        //matcher, maybe change for tracker
        matcher->match( descriptors_curr, descriptors_prev, matches);
        std::vector<cv::KeyPoint> matched_keypoints_curr, matched_keypoints_prev;
        for(const auto & elem : matches)
        {
            matched_keypoints_curr.push_back( keypoints_curr[ elem.queryIdx ] );
            matched_keypoints_prev.push_back( keypoints_prev[ elem.trainIdx ] );
        }

        //calculate 3D for prev_points, TODO: pre-calculate
        std::vector<cv::Point2f> img_pts_prev;//, img_pts_prev2;
        std::vector<cv::Point3f> img_pts_disp_prev;
        cv::KeyPoint::convert(matched_keypoints_prev,img_pts_prev);

        //add disparity to img_pts
        for(const auto & elem : img_pts_prev)
        {
            float disp = disp_prev.at<float>(elem.y,elem.x);
            if( disp != -1 )
            {
                img_pts_disp_prev.push_back( cv::Point3f ( elem.x, elem.y,  disp) );
                //img_pts_prev2.push_back(elem);//"rebuild vector without invalid points"
            }
        }

        //cv::Mat homog; cv::convertPointsToHomogeneous(img_pts_disp_prev, homog);
        //cv::Mat homog; cv::convertPointsToHomogeneous(img_pts_disp_prev, homog);

        //std::cout << "\nhomog \n" << homog << std::endl;
        //std::cout << "\nQ\n" << Q << std::endl;
        //std::cout << "saida ponto object" << Q*homog << std::endl;
        //std::vector<cv::Point3f> obj_pts_prev;
        std::vector<cv::Point3f> obj_pts_prev;

        try
        {
            cv::perspectiveTransform(img_pts_disp_prev, obj_pts_prev, Q); //transform 2D to 3D
        }
        catch(cv::Exception& e)
        {
            const char* err_msg = e.what();
            std::cout << "exception caught: " << err_msg << std::endl;
        }

        std::cout << "img_pts_disp_prev" << img_pts_disp_prev << std::endl;
        std::cout << "obj_points_prev" << obj_pts_prev << std::endl;

        //get image points for current frame
        const std::vector<cv::Point2f> img_pts_curr;
        cv::KeyPoint::convert(matched_keypoints_curr,img_pts_curr);

        //TODO: change to cv::solvePnPRansac() method
        std::cout << "img_pts_curr" << img_pts_curr << std::endl;
        try{
            cv::solvePnP(obj_pts_prev,img_pts_curr,M1,D1,R,T,false,cv::SOLVEPNP_ITERATIVE);
        }
        catch(cv::Exception& e)
        {
            const char* err_msg = e.what();
            std::cout << "exception caught: " << err_msg << std::endl;
        }


        std::cout << "\nRotação\n" << R << std::endl;
        std::cout << "\nTranslação\n" << T << std::endl;

    }

    keypoints_prev = keypoints_curr;
    descriptors_prev = descriptors_curr.clone();
    disp_prev = img_disp.clone();

    //build matrix of
    //calculate deltaPose between current and prev step and speeds/accelerations


    //calculate trajectory performed



    //cv::Mat img_out;
    //cv::drawKeypoints(img1, keypoints1, img1);//_out);//,cv::Scalar::all(-1),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    //img_out.copyTo(img1);


}



void StereoVO::calcDisp(cv::Mat & img_l , cv::Mat &img_r, cv::Mat & img_disp)
{
    //cv::StereoBM::create(0,21);
}

}//namespace
