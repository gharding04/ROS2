
#include <rclcpp/rclcpp.hpp>

#include <sl/Camera.hpp>
#include "aruco.hpp"
#include <opencv2/opencv.hpp>
#include "messages/msg/zed_position.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>

#define ROW_COUNT 10
bool jetsonStream = true;
bool laptopStream = false;
//using namespace sl;
//using namespace std;

/** @file
 * @brief Node handling Zed camera
 * 
 * This node does not receive any information from other nodes, therefore, it does not subscribe to any node. Current purpose is to utilize the "ArUco Positional Tracking sample" with the Zed camera and publishes topics relating to it. Node contains no other functions, only main.
 * \see aruco.cpp
 *  
 * The topics that are being published are as follows:
 * \li \b zedPosition
 * "zedPosition" contains the variables x, y, z, ox, oy, oz, ow, and aruco_visible.
 * 
 * The variables x, y, and z are the translation vectors. 
 * To learn more about the translation vectors, see https://en.wikipedia.org/wiki/Translation_(geometry)
 * 
 * The variables ox, oy, oz, and ow are the orientation vectors. The orientation data is also known as "quaternion" data. These vectors help with calculating three-dimensional rotations.
 * To learn more about quaternion, see https://en.wikipedia.org/wiki/Quaternion
 * 
 * The variable "aruco_visible" tells whether or not that at least one marker is detected.
 * 
 * \see ZedPosition.msg
 * 
 * Zed camera currently using WVGA mode which has a FOV of 56(V) and 87(H).
 * 
 * Nodes that subscribe to the published Zed topics include the logic and autonomy node.
 * \see logic_node.cpp
 * 
 * \see autonomy_node.cpp
 * 
 * 
 * */

void jetsonStreamCallback(const std_msgs::msg::Bool::SharedPtr msg){
    jetsonStream = msg->data;
}

void laptopStreamCallback(const std_msgs::msg::Bool::SharedPtr msg){
    laptopStream = msg->data;
}


int main(int argc, char **argv) {
    rclcpp::init(argc,argv);
    rclcpp::Node::SharedPtr nodeHandle = rclcpp::Node::make_shared("zed_tracking");

    RCLCPP_INFO(nodeHandle->get_logger(),"Starting zed_tracking");

    messages::msg::ZedPosition zedPosition;
    auto zedPositionPublisher=nodeHandle->create_publisher<messages::msg::ZedPosition>("zed_position",1);
    auto zedImagePublisher = nodeHandle->create_publisher<sensor_msgs::msg::Image>("zed_image",1);

    auto jetsonStreamSubscriber = nodeHandle->create_subscription<std_msgs::msg::Bool>("jetson_stream",1,jetsonStreamCallback);
    auto laptopStreamSubscriber = nodeHandle->create_subscription<std_msgs::msg::Bool>("laptop_stream",1,laptopStreamCallback);
    
    // Create a ZED camera object
    sl::Camera zed;

    // Set configuration parameters
    sl::InitParameters init_params;

    /*
    Camera Resolution Options (https://www.stereolabs.com/docs/api/group__Video__group.html#gabd0374c748530a64a72872c43b2cc828)
    HD2K 	
    -2208*1242 (x2),
    -available framerates: 15 fps
    -FOV: 47(V), 76(H)

    HD1080 	
    -1920*1080 (x2)
    -available framerates: 15, 30 fps
   -FOV: 42(V), 69(H)

    HD720 	
    -1280*720 (x2)
    -available framerates: 15, 30, 60 fps.
   -FOV: 54(V), 85(H)

    VGA	
    -672*376 (x2)
    -available framerates: 15, 30, 60, 100 fps.   
    -FOV: 56(V), 87(H)
    */
    init_params.camera_resolution = sl::RESOLUTION::VGA;
    init_params.coordinate_units = sl::UNIT::METER;
    init_params.coordinate_system = sl::COORDINATE_SYSTEM::IMAGE;
    init_params.camera_fps = 30;    
//    init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
//    init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP;
//    init_params.coordinate_system = sl::COORDINATE_SYSTEM::LEFT_HANDED_Y_UP;
//    init_params.coordinate_system = sl::COORDINATE_SYSTEM::LEFT_HANDED_Z_UP;
//    init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
    init_params.sensors_required = true;

    // Open the camera
    sl::ERROR_CODE err = zed.open(init_params);
    if (err != sl::ERROR_CODE::SUCCESS) {
	    std::cout << "Error, unable to open ZED camera: " << err << "\n";
        zed.close();
        return 1; // Quit if an error occurred
    }

    auto cameraInfo = zed.getCameraInformation().camera_configuration;
    sl::Resolution image_size = cameraInfo.resolution;
    sl::Mat image_zed(image_size, sl::MAT_TYPE::U8_C4);
    cv::Mat image_ocv = cv::Mat(image_zed.getHeight(), image_zed.getWidth(), CV_8UC4, image_zed.getPtr<sl::uchar1>(sl::MEM::CPU));
    cv::Mat image_ocv_rgb;
    sl::Mat depth, point_cloud;

    auto calibInfo = cameraInfo.calibration_parameters.left_cam;
    cv::Matx33d camera_matrix = cv::Matx33d::eye();
    camera_matrix(0, 0) = calibInfo.fx;
    camera_matrix(1, 1) = calibInfo.fy;
    camera_matrix(0, 2) = calibInfo.cx;
    camera_matrix(1, 2) = calibInfo.cy;

    cv::Matx<float, 4, 1> dist_coeffs = cv::Vec4f::zeros();

    float actual_marker_size_meters = 0.18891; // real marker size in meters
   // float actual_marker_size_meters = 0.16f; //fake marker size in meters
    auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100);

    std::cout << "Make sure the ArUco marker is a 6x6 (100), measuring " << actual_marker_size_meters * 1000 << " mm" << std::endl;

    sl::Transform arucoPose;
    sl::Pose zedPose;
    std::vector<cv::Vec3d> rvecs, tvecs;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    std::string zed_position_txt;
    std::string zed_rotation_txt;
    std::string aruco_position_txt;
    sl::float3 angles;

    sl::SensorsData sensors_data;
    sl::SensorsData::IMUData imu_data;

    bool initialized = false;

    double x_acc, y_acc, z_acc, x_vel, y_vel, z_vel;

    int currentRow=0;
    float pastValues[ROW_COUNT][7];
    float average[7];
    for(int col=0;col<7;col++){
	    average[col]=0;
        for(int row=0;row<ROW_COUNT;row++){
	        pastValues[row][col]=0;
	    }
    }

    sl::PositionalTrackingParameters tracking_params;
    tracking_params.enable_imu_fusion = true;
    tracking_params.enable_area_memory = true;
    auto returned_state = zed.enablePositionalTracking(tracking_params);
    if (returned_state != sl::ERROR_CODE::SUCCESS) {
        zed.close();
        return EXIT_FAILURE;
    }

    rclcpp::Rate rate(30);
    while (rclcpp::ok()) {
        if (zed.grab() == sl::ERROR_CODE::SUCCESS) {
            // Retrieve the left image
            zed.retrieveImage(image_zed, sl::VIEW::LEFT, sl::MEM::CPU, image_size);
            // convert to RGB
            cv::cvtColor(image_ocv, image_ocv_rgb, cv::COLOR_RGBA2RGB);
            // detect marker
            cv::aruco::detectMarkers(image_ocv_rgb, dictionary, corners, ids);
            // if at least one marker detected
            if (ids.size() > 0) {
	        //	int id=ids[0];
                cv::aruco::estimatePoseSingleMarkers(corners, actual_marker_size_meters, camera_matrix, dist_coeffs, rvecs, tvecs);
                arucoPose.setTranslation(sl::float3(tvecs[0](0), tvecs[0](1), tvecs[0](2)));
                arucoPose.setRotationVector(sl::float3(rvecs[0](0), rvecs[0](1), rvecs[0](2)));
                arucoPose.inverse();
                if(!initialized){
                    zed.resetPositionalTracking(arucoPose);
                    initialized = true;                
                }
                angles = zedPose.getEulerAngles(false);
                zedPosition.aruco_pitch = angles[2];
                zedPosition.aruco_yaw = angles[1];
                zedPosition.aruco_roll = angles[0];
		        zedPosition.aruco_visible=true;
	        } 
            else {
	            zedPosition.aruco_visible=false;
	        }

            zed.getSensorsData(sensors_data, sl::TIME_REFERENCE::IMAGE);

            imu_data = sensors_data.imu;

            sl::float3 lin = imu_data.linear_acceleration;
            sl::float3 vel = imu_data.angular_velocity;
            x_acc = lin[0];
            y_acc = lin[1];
            z_acc = lin[2];
            x_vel = vel[0];
            y_vel = vel[1];
            z_vel = vel[2];

/*
            zed.retrieveImage(image_zed, sl::VIEW::LEFT);
            zed.retrieveMeasure(depth, sl::MEASURE::DEPTH);
            zed.retrieveMeasure(point_cloud, sl::MEASURE::XYZRGBA);

            int x = image_zed.getWidth() / 2;
            int y = image_zed.getHeight() / 2;
            sl::float4 point_cloud_value;
            point_cloud.getValue(x, y, &point_cloud_value);

            if(std::isfinite(point_cloud_value.z)){
                distance = sqrt(point_cloud_value.x * point_cloud_value.x + point_cloud_value.y * point_cloud_value.y + point_cloud_value.z * point_cloud_value.z);
                distance *= 10;
                RCLCPP_INFO(nodeHandle->get_logger(), "Distance: %f", distance);
            }
            else{
                distance = -1;
            }
*/
            
            sl::POSITIONAL_TRACKING_STATE tracking_state = zed.getPosition(zedPose, sl::REFERENCE_FRAME::WORLD);

            if (tracking_state == sl::POSITIONAL_TRACKING_STATE::OK) {
                angles = zedPose.getEulerAngles(false);
    	        zedPosition.x=zedPose.getTranslation().x;
        	    zedPosition.y=zedPose.getTranslation().y;
    	        zedPosition.z=zedPose.getTranslation().z;
    	        zedPosition.ox=zedPose.getOrientation().ox;
    	        zedPosition.oy=zedPose.getOrientation().oy;
    	        zedPosition.oz=zedPose.getOrientation().oz;
    	        zedPosition.ow=zedPose.getOrientation().ow;
                zedPosition.pitch = angles[0];
                zedPosition.yaw = angles[1];
                zedPosition.roll = angles[2];
                zedPosition.x_acc = x_acc;
                zedPosition.y_acc = y_acc;
                zedPosition.z_acc = z_acc;
                zedPosition.x_vel = x_vel;
                zedPosition.y_vel = y_vel;
                zedPosition.z_vel = z_vel;
                zedPositionPublisher->publish(zedPosition);
            }
            if(jetsonStream){
                RCLCPP_INFO(nodeHandle->get_logger(), "Published image");
                sensor_msgs::msg::Image::UniquePtr outImage(new sensor_msgs::msg::Image());
                outImage->height = image_ocv.rows;
                outImage->width = image_ocv.cols;
                outImage->encoding = "rgb";
                outImage->is_bigendian = false;
                outImage->step = static_cast<sensor_msgs::msg::Image::_step_type>(image_ocv.step);
                outImage->data.assign(image_ocv.datastart, image_ocv.dataend);
                zedImagePublisher->publish(std::move(outImage));
            }
            if(laptopStream){
                sensor_msgs::msg::Image::UniquePtr outImage(new sensor_msgs::msg::Image());
                outImage->height = image_ocv.rows;
                outImage->width = image_ocv.cols;
                outImage->encoding = "rgb";
                outImage->is_bigendian = false;
                outImage->step = static_cast<sensor_msgs::msg::Image::_step_type>(image_ocv.step);
                outImage->data.assign(image_ocv.datastart, image_ocv.dataend);
                zedImagePublisher->publish(std::move(outImage));
            }

        }
	    rate.sleep();
    }
    zed.close();
    return 0;

}
