#include <iostream>
#include <time.h>
#include <thread>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <pwd.h>
#include <algorithm>
#include <signal.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <tof.hpp>

int lensType = 0;  //0- wide field, 1- standard field, 2 - narrow field
int old_lensType;
int frequencyModulation = 2;
int channel = 0;
int imageType = 2; //image and aquisition type: 0 - grayscale, 1 - distance, 2 - distance_amplitude
int hdr_mode = 2;
int int0 = 50, int1 = 400, int2 = 4000, intGr = 25000;
int minAmplitude = 60;
int lensCenterOffsetX = 0;
int lensCenterOffsetY = 0;

int roi_leftX = 0;
int roi_topY = 0;
int roi_rightX = 319;
int roi_bottomY = 239;

ToF* tof;
bool exit_requested = false;

std::chrono::steady_clock::time_point st_time;
std::chrono::steady_clock::time_point en_time;
double interval, frame_rate;
int n_frames = 0;

void startStreaming()
{
    switch(imageType) {
    case Frame::DISTANCE:
        tof->streamDistance();
        std::cout << "Start streaming distance" << std::endl;
        break;
    case Frame::AMPLITUDE:
        tof->streamDistanceAmplitude();
		std::cout << "Start streaming distance-amplitude" << std::endl;
        break;
    case Frame::GRAYSCALE:
        tof->streamGrayscale();
		std::cout << "Start streaming grayscale" << std::endl;
        break;
    default:
        break;
    }
}

void setParameters()
{
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
}

void updateFrame(std::shared_ptr<Frame> frame)
{
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    //pcl::PointXYZRGB* data_ptr = reinterpret_cast<pcl::PointXYZRGB*>(frame->data_3d_xyz_rgb);
    //std::vector<pcl::PointXYZRGB> pts(data_ptr, data_ptr + frame->n_points);
    //cloud->points.insert(cloud->points.end(), pts.begin(), pts.end());

    if (imageType == 1 ||imageType == 2)
    {
        cv::Mat depth_bgr(frame->height, frame->width, CV_8UC3, frame->data_2d_bgr);
        cv::flip(depth_bgr, depth_bgr, 1);    
        cv::imshow("depth_bgr", depth_bgr);
    }
	
    if (imageType == 2)
    {
        cv::Mat amplitude(frame->height, frame->width, CV_32F, frame->data_amplitude);
        cv::normalize(amplitude, amplitude, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        cv::flip(amplitude, amplitude, 1);
        cv::imshow("amplitude", amplitude);
    }
	
    if (imageType == 0)
    {
        cv::Mat grayscale(frame->height, frame->width, CV_8UC1, frame->data_grayscale);
        cv::flip(grayscale, grayscale, 1);
        cv::imshow("grayscale", grayscale);
    }

    n_frames ++;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZRGB* data_ptr = reinterpret_cast<pcl::PointXYZRGB*>(frame->data_3d_xyz_rgb);
    std::vector<pcl::PointXYZRGB> pts(data_ptr, data_ptr + frame->n_points);
    // cloud->points.insert(cloud->points.end(), pts.begin(), pts.end());

    // Amplitude Map
    auto vecsize = frame->n_points * 4;
    auto amp_ptr = frame->data_amplitude;
    auto amp_float = std::vector<float>(amp_ptr, amp_ptr + vecsize);
    // cv::Mat amplitude_mat = cv::Mat(frame->height, frame->width, CV_32F, frame->data_amplitude);

    // Depth Map BGR
    cv::Mat depth_bgr = cv::Mat(frame->height, frame->width, CV_8UC3, frame->data_2d_bgr);

    // Depth Raw Map
    cv::Mat depth_mat = cv::Mat(frame->height, frame->width, CV_32F, frame->data_depth);

    // Saturated Mask
    cv::Mat saturated_mask = cv::Mat(frame->height, frame->width, CV_8UC1, frame->saturated_mask);

    // cv::imshow("Amplitude", amplitude_mat);
	cv::imshow("depth_bgr", depth_bgr);
    cv::imshow("depth_raw", depth_mat);
    cv::imshow("Saturated Mask", saturated_mask);

	if (cv::waitKey(1) == 27)
	{
		exit_requested = true;
	}
}

void exit_handler(int s){
	std::cout << "\nExiting ... " << std::endl;
	exit_requested = true;
	fflush(stdout);
	usleep(1000000);
	signal(SIGINT, exit_handler);
}

int main(int argc, char** argv) {

	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = exit_handler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);
    
    std::string cur_ip = "10.10.31.180";
    if (argc >= 2)
    {
        cur_ip = std::string(argv[1]);
        imageType = std::stoi(argv[2]);
    } else if (argc > 3)
    {
        std::cout << "Usage:   ./demo <camera IP address> <image type: 0 - grayscale, 1 - distance, 2 - distance_amplitude>" << std::endl;
        std::cout << "Example: ./demo 10.10.10.180 1" << std::endl;
        return 1;
    }    

    std::cout << "Connecting to LiMu camera at IP address: " << cur_ip << std::endl;

    tof = ToF::tof320(cur_ip.c_str(), "50660");

    tof->subscribeFrame([&](std::shared_ptr<Frame> f) -> void {  updateFrame(f); });

	std::cout << "Camera is connected." << std::endl;

    setParameters();

    startStreaming();

    st_time = std::chrono::steady_clock::now();
	std::cout << "Camera is started." << std::endl;

	while(! exit_requested){}

    en_time = std::chrono::steady_clock::now();
    interval = ((double) std::chrono::duration_cast<std::chrono::microseconds>(en_time - st_time).count()) / 1000000.0;
    frame_rate = ((double) n_frames) / interval;
    std::cout << "Distance frames: " << n_frames << " time spent: " << interval << " frame rate: " << frame_rate << std::endl;

	std::cerr << "Shutdown ... " << std::endl;

	tof->stopStream();

    usleep(1000000);

	delete tof;

	return 0;
}
