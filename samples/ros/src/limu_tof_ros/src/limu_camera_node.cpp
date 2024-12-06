#include <thread>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <dynamic_reconfigure/server.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>

#include <pcl/features/moment_of_inertia_estimation.h>

#include <pcl/surface/concave_hull.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/video.hpp>

#include "tof.hpp"
#include "frame.hpp"
#include "imu.h"

#include <limu_tof_ros/limu_tof_rosConfig.h>

int lensType;  //0- wide field, 1- standard field, 2 - narrow field
int old_lensType;
int frequencyModulation;
int channel;
int imageType; //image and aquisition type: 0 - grayscale, 1 - distance, 2 - distance_amplitude
int hdr_mode;
int int0 = 50, int1, int2, intGr = 10000;
int minAmplitude;
int lensCenterOffsetX = 0;
int lensCenterOffsetY = 0;

int roi_leftX = 0;
int roi_topY = 0;
int roi_rightX = 319;
int roi_bottomY = 239;

ros::Publisher cloud_publisher;
ros::Publisher depth_Publisher;
ros::Publisher amplitude_publisher;
ros::Publisher rgb_publisher;
ros::Publisher imu_publisher;

ToF* tof;
sensor_msgs::CameraInfo cameraInfo;

static std::string strFrameID = "sensor_frame";

std::chrono::steady_clock::time_point st_time;
std::chrono::steady_clock::time_point en_time;
double interval, frame_rate;
int n_frames = 0;

std::string imu_port;
std::string rgb_camera;

cv::VideoCapture* videoCapture;
IMU* imu_0;

bool rgb_available = false;
bool imu_available = false;

cv::Mat rgb_image;

void publish_image(ros::Publisher publisher, cv::Mat image, ros::Time time) {

	sensor_msgs::Image ros_msg;
	ros_msg.header.frame_id = strFrameID;
	ros_msg.height = image.rows;
	ros_msg.width = image.cols;
	ros_msg.encoding = sensor_msgs::image_encodings::BGR8;
	ros_msg.step = image.cols * image.elemSize();
	size_t size = ros_msg.step * image.rows;
	ros_msg.data.resize(size);

	if (image.isContinuous())
	{
		memcpy((char*)(&ros_msg.data[0]), image.data, size);
	}
	else
	{
		uchar* ros_data_ptr = (uchar*)(&ros_msg.data[0]);
		uchar* cv_data_ptr = image.data;
		for (int i = 0; i < image.rows; ++i)
		{
			memcpy(ros_data_ptr, cv_data_ptr, ros_msg.step);
			ros_data_ptr += ros_msg.step;
			cv_data_ptr += image.step;
		}
	}

	ros_msg.header.stamp   = time;

	publisher.publish(ros_msg);
}

void startStreaming()
{
    switch(imageType) {
    case Frame::DISTANCE:
        tof->streamDistance();
        ROS_INFO("Start streaming distance");
        break;
    case Frame::AMPLITUDE:
        tof->streamDistanceAmplitude();
        ROS_INFO("Start streaming distance-amplitude");
        break;
    default:
        break;
    }
}

void setParameters()
{
    ROS_INFO("set parameters...");
    tof->stopStream();

    uint8_t modIndex;
    if(frequencyModulation == 0) modIndex = 1;
    else if(frequencyModulation == 1)  modIndex = 0;
    else    modIndex = frequencyModulation;

    tof->setModulation(modIndex, channel);

    tof->setMinAmplitude(minAmplitude);
    tof->setIntegrationTime(int0, int1, int2, intGr);
    tof->setHDRMode((uint8_t)hdr_mode);

    tof->setRoi(roi_leftX, roi_topY, roi_rightX, roi_bottomY);

    tof->setLensType(lensType);
    tof->setLensCenter(lensCenterOffsetX, lensCenterOffsetY);
    
    tof->setFilter(0, 0, 0, 0, 0, 0, 0, 0, 0);

    ROS_INFO("lens_type %d", lensType);
    ROS_INFO("frequency_modulation %d", frequencyModulation);
    ROS_INFO("channel %d ", channel);
    ROS_INFO("image_type %d", imageType);
    ROS_INFO("hdr_mode %d", hdr_mode);
    ROS_INFO("integration_time0 %d", int0);
    ROS_INFO("integration_time1 %d", int1);
    ROS_INFO("integration_time2 %d", int2);
    ROS_INFO("min_amplitude %d", minAmplitude);
    ROS_INFO("integration_time_gray %d", intGr);

    ROS_INFO("parameters done.");

    startStreaming();
    
    st_time = std::chrono::steady_clock::now();
    n_frames = 0;
}

void updateConfig(limu_tof_ros::limu_tof_rosConfig &config, uint32_t level)
{
    std::cerr << "Updating config ... " << std::endl;
    lensType = config.lens_type;
    frequencyModulation = config.frequency_modulation;
    channel = config.channel;

    imageType = config.image_type;
    hdr_mode = config.hdr_mode;

    int0 = config.integration_time_tof_1;
    int1 = config.integration_time_tof_2;
    int2 = config.integration_time_tof_3;
    
    minAmplitude = config.min_amplitude;
    
    roi_leftX   = config.roi_left_x;
    roi_rightX  = config.roi_right_x;

    if(roi_rightX - roi_leftX < 7)
        roi_rightX = roi_leftX + 7;

    roi_rightX -= (roi_rightX - roi_leftX + 1) % 4;
    config.roi_right_x = roi_rightX;

    config.roi_height -= config.roi_height % 4;
    if(config.roi_height < 8) config.roi_height = 8;

    roi_topY    = 120 - config.roi_height/2;
    roi_bottomY = 119 + config.roi_height/2;

    setParameters();
}

void updateCameraInfo(std::shared_ptr<CameraInfo> ci)
{
    cameraInfo.width = ci->width;
    cameraInfo.height = ci->height;
    cameraInfo.roi.x_offset = ci->roiX0;
    cameraInfo.roi.y_offset = ci->roiY0;
    cameraInfo.roi.width = ci->roiX1 - ci->roiX0;
    cameraInfo.roi.height = ci->roiY1 - ci->roiY0;
}

void updateFrame(std::shared_ptr<Frame> frame)
{
    geometry_msgs::PoseStamped pose_imu;
    Eigen::Affine3f t_sensor = Eigen::Affine3f::Identity();
    if (imu_available)
    {
        float angle_x = 0;
        float angle_y = 0;
        float angle_z_sensor = 0;

        angle_x        = M_PI * imu_0->Angle[0] / 180.0;
        angle_y        = M_PI * imu_0->Angle[1] / 180.0;
        angle_z_sensor = M_PI * imu_0->Angle[2] / 180.0;

        //std::cout << "angle => angle_x: " << angle_x << " angle_y: " << angle_y << " angle_z_sensor: " << angle_z_sensor << std::endl;

        Eigen::Matrix3f r_imu;

        r_imu =
                Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ())
            * Eigen::AngleAxisf(0,        Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(M_PI * (30) / 180.0,        Eigen::Vector3f::UnitX());
        
        t_sensor.rotate (r_imu);

        Eigen::Quaternionf q_imu(r_imu);

		pose_imu.pose.position.x = 0;
		pose_imu.pose.position.y = 0;
		pose_imu.pose.position.z = 0;
		pose_imu.pose.orientation.w = q_imu.w();
		pose_imu.pose.orientation.x = q_imu.x();
		pose_imu.pose.orientation.y = q_imu.y();
		pose_imu.pose.orientation.z = q_imu.z();

        Eigen::Affine3f t_sensor_2_world = Eigen::Affine3f::Identity();
		Eigen::Matrix3f m;
		m <<
		1, 0, 0,
		0, 0, -1,
		0, -1, 0;
		t_sensor_2_world.rotate (m);

        t_sensor = t_sensor * t_sensor_2_world;

        t_sensor = t_sensor.inverse();
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PointXYZRGB* data_ptr = reinterpret_cast<pcl::PointXYZRGB*>(frame->data_3d_xyz_rgb);
    std::vector<pcl::PointXYZRGB> pts(data_ptr, data_ptr + frame->n_points);
    cloud->points.insert(cloud->points.end(), pts.begin(), pts.end());

    cv::Mat depth_bgr(frame->height, frame->width, CV_8UC3, frame->data_2d_bgr);
    cv::Mat amplitude(frame->height, frame->width, CV_32F,  frame->data_amplitude);

    cv::Mat mat_depth_bgr_flipped; 
    cv::flip(depth_bgr, mat_depth_bgr_flipped, 1);
    
    ros::Time curTime = ros::Time::now();

    const size_t nPixel = frame->width * frame->height;
    cloud->header.frame_id = strFrameID;
    cloud->header.stamp = pcl_conversions::toPCL(curTime);
    cloud->points.resize(nPixel);

    if (imu_available)
    {
		pose_imu.header.frame_id = strFrameID;
		pose_imu.header.stamp = curTime;
        imu_publisher.publish(pose_imu);
    }

    cloud_publisher.publish(cloud);
    publish_image(depth_Publisher, mat_depth_bgr_flipped, curTime);
    if (rgb_available)
    {
        publish_image(rgb_publisher, rgb_image, curTime);
    }

    n_frames ++;
}

//===================================================
void initialise()
{
    ros::NodeHandle nh("~");

    nh.getParam("lens_Type", lensType);
    nh.getParam("frequency_modulation", frequencyModulation);
    nh.getParam("channel", channel);
    nh.getParam("image_type", imageType);
    nh.getParam("hdr_mode", hdr_mode);
    nh.getParam("int0", int0);
    nh.getParam("int1", int1);
    nh.getParam("int2", int2);
    nh.getParam("min_amplitude", minAmplitude);
    nh.getParam("int_gray", intGr);
    nh.getParam("rgb_camera", rgb_camera);
    nh.getParam("imu_port", imu_port);

    cloud_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("cloud", 1);
	depth_Publisher = nh.advertise<sensor_msgs::Image>("depth", 1);
    amplitude_publisher = nh.advertise<sensor_msgs::Image>("amplitude", 1);
    rgb_publisher = nh.advertise<sensor_msgs::Image>("rgb", 1);
    imu_publisher     = nh.advertise<geometry_msgs::PoseStamped>("imu", 1);

    //connect to camera
    tof->subscribeCameraInfo([&](std::shared_ptr<CameraInfo> ci) -> void { updateCameraInfo(ci); });
    tof->subscribeFrame([&](std::shared_ptr<Frame> f) -> void {  updateFrame(f); });

    setParameters();
}

void start_imu_0()
{
	imu_0->start();
}

void start_RGB()
{
	cv::Mat frame;
    if (rgb_available)
    {
        while(ros::ok())
        {
			try
			{
                videoCapture->read(frame);
                if ((! frame.empty()) && frame.rows > 0 && frame.cols > 0)
                {
                        rgb_image = frame.clone();
                }
			} catch (...) {
				std::cerr << "Something wrong with RGB camera!" << std::endl;
			}
        }
    }
}

int main(int argc, char **argv)
{
    std::cerr << "Starting ROS ... " << std::endl;

    tof = ToF::tof320("10.10.31.180", "50660");

    ros::init(argc, argv, "limu_camera_node");
    std::cerr << "Starting config server ... " << std::endl;
    dynamic_reconfigure::Server<limu_tof_ros::limu_tof_rosConfig> server;
    dynamic_reconfigure::Server<limu_tof_ros::limu_tof_rosConfig>::CallbackType f;
    std::cerr << "Binding config ... " << std::endl;
    f = boost::bind(&updateConfig, _1, _2);
    server.setCallback(f);

    initialise();

    std::cerr << "Connecting to IMU: " << imu_port << std::endl;
    
    imu_0 = new IMU(imu_port.c_str(), 9600, 8, 'N', 1);
    imu_available = imu_0->openSerialPort();

    if (imu_available)
    {
        std::thread th_start_imu_0(&start_imu_0);
        th_start_imu_0.detach();
    } else 
    {
        std::cerr << "Warning: IMU is not available! " << std::endl;
    }

    videoCapture = new cv::VideoCapture();
    std::cerr << "Starting RGB camera: " << rgb_camera << std::endl;
    try {
        rgb_available = videoCapture->open(rgb_camera);
        if (rgb_available)
        {
            videoCapture->set(cv::CAP_PROP_FRAME_WIDTH, 640);
            videoCapture->set(cv::CAP_PROP_FRAME_HEIGHT, 480);

            std::cerr << "RGB camera started." << std::endl;

            std::thread th_start_RGB(&start_RGB);
            th_start_RGB.detach();
        } else
        {
            std::cerr << "Warning: RGB camera is not available! " << std::endl;
        }
    } catch (...)
    {
        std::cerr << "Error: unexpected error when initializing RGB camera!" << std::endl;
    }

    st_time = std::chrono::steady_clock::now();

    ros::spin();

    en_time = std::chrono::steady_clock::now();
    interval = ((double) std::chrono::duration_cast<std::chrono::microseconds>(en_time - st_time).count()) / 1000000.0;
    frame_rate = ((double) n_frames) / interval;
    std::cout << "Distance frames: " << n_frames << " time spent: " << interval << " frame rate: " << frame_rate << std::endl;

	std::cerr << "Shutdown ... " << std::endl;

    if (imu_available)
    {
        imu_0->stop();
    }

    if (rgb_available)
    {
        videoCapture->release();
    }

    tof->stopStream();

	usleep(2000000);

    delete imu_0;
    delete videoCapture;
    delete tof;

	std::cerr << "Shutdown ROS ... " << std::endl;

    ros::shutdown();
}
