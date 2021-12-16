#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <visual_odometry/visual_odometry.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/CameraInfo.h>


static const std::string OPENCV_WINDOW = "Image window";
static const std::string OPENCV_WINDOW2 = "Image window2";

class ImageConverter
{
    ros::NodeHandle nh_;
    message_filters::Subscriber<sensor_msgs::Image> cam_l;
    //message_filters::Subscriber<sensor_msgs::Image> cam_disp;//cam_r;
    message_filters::Subscriber<stereo_msgs::DisparityImage> cam_disp;

    //message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync;
    message_filters::TimeSynchronizer<sensor_msgs::Image, stereo_msgs::DisparityImage> sync;

    visual_odometry::StereoVO vo_;


public:
    ImageConverter() : sync(cam_l, cam_disp,1)//,it_(nh_)
    {
        cam_l.subscribe(nh_,"/stereo/left/image_rect", 1);
        //cam_r.subscribe(nh_,"/stereo/right/image_rect", 1);
        cam_disp.subscribe(nh_,"/stereo/disparity",1);

        sync.registerCallback(boost::bind(&ImageConverter::imageCb,this, _1, _2));
        cv::namedWindow(OPENCV_WINDOW);
        cv::namedWindow(OPENCV_WINDOW2);

        //get cam parameters
       // cv::FileStorage fs_l, fs_r;
//        fs_l.open("/home/name/catkin_ws/src/dataset_pub/cam_calib_l.yml", cv::FileStorage::READ);
  //      fs_r.open("/home/name/catkin_ws/src/dataset_pub/cam_calib_r.yml", cv::FileStorage::READ);
/*        fs_l.open("cam_calib_l.yml", cv::FileStorage::READ);
        fs_r.open("cam_calib_r.yml", cv::FileStorage::READ);

        cv::Mat P1 = fs_l["projection_matrix"];
        cv::Mat D1 = fs_l["distortion_coefficients"];
        cv::Mat P2 = fs_r["projection_matrix"];


        //sensor_msgs::CameraInfo cam_msg;
        //cam_msg.D =
        std::vector<double> P1,D1,P2;
        fs_l["projection_matrix"] >> P1;
        fs_r["projection_matrix"] >> P2;
        fs_l["distortion_coefficients"] >> D1;

        cv::Mat P11(,P1);//,D11,P22;


        std::cout << "\nprojection_matrix1\n" << P1 <<std::endl ;
        std::cout << "\nprojection_matrix2\n" << P2 <<std::endl ;
        std::cout << "\ndistortion\n" << P2 <<std::endl ;
        //give cam parameters for vo
        //vo_.camParam(P1,P2,D1);
*/
    }

    //void convertYAMLROStoOpenCV(const std::vector<double>& ros_vec, int lin, int row, cv::Mat& cv_mat)

    ~ImageConverter()
    {
        cv::destroyWindow(OPENCV_WINDOW);
        cv::destroyWindow(OPENCV_WINDOW2);
    }

    //void imageCb(const sensor_msgs::ImageConstPtr& msg_l, const sensor_msgs::ImageConstPtr& msg_disp)//msg_r)
    void imageCb(const sensor_msgs::ImageConstPtr& msg_l, const stereo_msgs::DisparityImageConstPtr& msg_disp)
    {
        cv_bridge::CvImagePtr cv_ptr_l;
        cv_bridge::CvImageConstPtr cv_ptr_disp;
        try
        {
            cv_ptr_l = cv_bridge::toCvCopy(msg_l, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        try
        {
            //cv_ptr_r = cv_bridge::toCvCopy(msg_r, sensor_msgs::image_encodings::BGR8);
            //cv_ptr_disp = cv_bridge::toCvCopy(msg_disp,sensor_msgs::image_encodings::)
            cv_ptr_disp = cv_bridge::toCvShare( msg_disp->image,msg_disp,sensor_msgs::image_encodings::TYPE_32FC1);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        //vo_.process(cv_ptr_l->image,cv_ptr_r->image);
        std::cout << "chamou" << std::endl;
        vo_.process(cv_ptr_l->image,cv_ptr_disp->image);

        // Update GUI Window
        cv::imshow(OPENCV_WINDOW, cv_ptr_l->image);
        cv::imshow(OPENCV_WINDOW2, cv_ptr_disp->image);
        cv::waitKey(3);

        // Output modified video stream
        //image_pub_.publish(cv_ptr->toImageMsg());
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "visual_odom_node");
    ImageConverter ic;
    ros::spin();
    return 0;
}
