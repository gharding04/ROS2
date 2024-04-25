#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <linux/if_packet.h>
#include <thread>
#include <chrono>
#include <vector>
#include <linux/reboot.h>
#include <sys/reboot.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"

#define PORT 31338

bool videoStreaming=true;
int new_socket;
rclcpp::Node::SharedPtr nodeHandle;
int total = 0;

/** @brief Receives the ZED camera image
 * 
 * This function hasn't been fully implemented yet.  In the future, this
 * will send the received image to the client-side GUI.
 * @param inputImage 
 */
void zedImageCallback(const sensor_msgs::msg::Image::SharedPtr inputImage){
    if(videoStreaming){
        cv::Mat outputImage = cv_bridge::toCvCopy(inputImage, "bgr8")->image;
        RCLCPP_INFO(nodeHandle->get_logger(), "Image width: %d, image height: %d", outputImage.width, outputImage.height);
        outputImage = outputImage.reshape(0,1);
        int imgSize = outputImage.total()*outputImage.elemSize();
        if(send(new_socket, outputImage.data, imgSize, 0)== -1){
            RCLCPP_INFO(nodeHandle->get_logger(), "Failed to send message.");   
        }
    }
}


void laptopStreamCallback(const std_msgs::msg::Bool::SharedPtr msg){
    videoStreaming = msg->data;
}


void jetsonStreamCallback(const std_msgs::msg::Bool::SharedPtr msg){
    videoStreaming = msg->data;
}


int main(int argc, char **argv){
    rclcpp::init(argc,argv);

    nodeHandle = rclcpp::Node::make_shared("video_streaming");
    RCLCPP_INFO(nodeHandle->get_logger(),"Starting video streaming node");


    auto laptopStreamSubscriber=nodeHandle->create_subscription<std_msgs::msg::Bool>("laptop_stream",1, laptopStreamCallback);
    auto jetsonStreamSubscriber=nodeHandle->create_subscription<std_msgs::msg::Bool>("jetson_stream",1, jetsonStreamCallback);

    auto zedImageSubscriber = nodeHandle->create_subscription<sensor_msgs::msg::Image>("zed_image", 10, zedImageCallback);

    int server_fd, bytesRead; 
    struct sockaddr_in address; 
    int opt = 1; 
    int addrlen = sizeof(address); 
    uint8_t buffer[1024] = {0}; 
    std::string hello("Hello from server");


    // Creating socket file descriptor
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) { 
        perror("socket failed"); 
        exit(EXIT_FAILURE); 
    }

    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) { 
        perror("setsockopt"); 
        exit(EXIT_FAILURE); 
    } 
    address.sin_family = AF_INET; 
    address.sin_addr.s_addr = INADDR_ANY; 
    address.sin_port = htons( PORT ); 

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address))<0) { 
        perror("bind failed"); 
        exit(EXIT_FAILURE); 
    } 
    if (listen(server_fd, 3) < 0) { 
        perror("listen"); 
        exit(EXIT_FAILURE); 
    } 
    if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen))<0) { 
        perror("accept"); 
        exit(EXIT_FAILURE); 
    }

    bytesRead = read(new_socket, buffer, 1024); 
    send(new_socket, hello.c_str(), strlen(hello.c_str()), 0); 

    fcntl(new_socket, F_SETFL, O_NONBLOCK);
    

    rclcpp::Rate rate(30);
    while(rclcpp::ok()){
        rclcpp::spin_some(nodeHandle);
        rate.sleep();
    }

}
