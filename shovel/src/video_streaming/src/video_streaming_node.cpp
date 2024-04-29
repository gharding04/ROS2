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

#define PORT 31338

bool videoStreaming=true;
int new_socket;
rclcpp::Node::SharedPtr nodeHandle;
int total = 0;
bool broadcast = true;

/** @brief Receives the ZED camera image
 * 
 * This function hasn't been fully implemented yet.  In the future, this
 * will send the received image to the client-side GUI.
 * @param inputImage 
 */
void zedImageCallback(const sensor_msgs::msg::Image::SharedPtr inputImage){
    RCLCPP_INFO(nodeHandle->get_logger(), "Received image.");
    if(videoStreaming){
    }
}


void laptopStreamCallback(const std_msgs::msg::Bool::SharedPtr msg){
    videoStreaming = msg->data;
}


void jetsonStreamCallback(const std_msgs::msg::Bool::SharedPtr msg){
    videoStreaming = msg->data;
}

std::string getAddressString(int family, std::string interfaceName){
    std::string addressString("");
    ifaddrs* interfaceAddresses = nullptr;
    for (int failed=getifaddrs(&interfaceAddresses); !failed && interfaceAddresses; interfaceAddresses=interfaceAddresses->ifa_next){
        if(strcmp(interfaceAddresses->ifa_name,interfaceName.c_str())==0 && interfaceAddresses->ifa_addr->sa_family == family) {
            if (interfaceAddresses->ifa_addr->sa_family == AF_INET) {
                sockaddr_in *socketAddress = reinterpret_cast<sockaddr_in *>(interfaceAddresses->ifa_addr);
                addressString += inet_ntoa(socketAddress->sin_addr);
            }
            if (interfaceAddresses->ifa_addr->sa_family == AF_INET6) {
                sockaddr_in6 *socketAddress = reinterpret_cast<sockaddr_in6 *>(interfaceAddresses->ifa_addr);
                for (int index = 0; index < 16; index += 2) {
                    char bits[5];
                    sprintf(bits,"%02x%02x", socketAddress->sin6_addr.s6_addr[index],socketAddress->sin6_addr.s6_addr[index + 1]);
                    if (index)addressString +=":";
                    addressString +=bits;
                }
            }
            if (interfaceAddresses->ifa_addr->sa_family == AF_PACKET) {
                sockaddr_ll *socketAddress = reinterpret_cast<sockaddr_ll *>(interfaceAddresses->ifa_addr);
                for (int index = 0; index < socketAddress->sll_halen; index++) {
                    char bits[3];
                    sprintf(bits,"%02x", socketAddress->sll_addr[index]);
                    if (index)addressString +=":";
                    addressString +=bits;
                }
            }
        }
    }
    freeifaddrs(interfaceAddresses);
    return addressString;
}

std::string robotName="shovel";
void broadcastIP(){
    while(true){
        if(broadcast){
            std::string addressString=getAddressString(AF_INET,"wlan0");

            std::string message(robotName+"@"+addressString);
            std::cout << message << std::endl << std::flush;

            int socketDescriptor=socket(AF_INET, SOCK_DGRAM, 0);

            //if(socket>=0){
            if(socketDescriptor>=0){
                struct sockaddr_in socketAddress;
                socketAddress.sin_family=AF_INET;
                socketAddress.sin_addr.s_addr = inet_addr("226.1.1.1");
                socketAddress.sin_port = htons(4321);

                struct in_addr localInterface;
                localInterface.s_addr = inet_addr(addressString.c_str());
                if(setsockopt(socketDescriptor, IPPROTO_IP, IP_MULTICAST_IF, (char*)&localInterface, sizeof(localInterface))>=0){
                    sendto(socketDescriptor,message.c_str(),message.length(),0,(struct sockaddr*)&socketAddress, sizeof(socketAddress));
                }
            }
            close(socketDescriptor);
        }
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }
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
    uint8_t buffer[2048] = {0}; 
    std::string hello("Hello from server");


    // Creating socket file descriptor
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) { 
        perror("socket failed"); 
        exit(EXIT_FAILURE); 
    }
    std::thread broadcastThread(broadcastIP);

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

    bytesRead = read(new_socket, buffer, 2048); 
    send(new_socket, hello.c_str(), strlen(hello.c_str()), 0); 

    fcntl(new_socket, F_SETFL, O_NONBLOCK);
    

    std::list<uint8_t> messageBytesList;
    uint8_t message[256];
    rclcpp::Rate rate(30);
    while(rclcpp::ok()){
        bytesRead = recv(new_socket, buffer, 2048, 0);
        for(int index=0;index<bytesRead;index++){
            messageBytesList.push_back(buffer[index]);
        }

        if(bytesRead==0){
	        RCLCPP_INFO(nodeHandle->get_logger(),"Lost Connection");
            broadcast=true;
            //wait for reconnect
            if (listen(server_fd, 3) < 0) { 
                perror("listen"); 
                exit(EXIT_FAILURE); 
            } 
            if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen))<0) { 
                perror("accept"); 
                exit(EXIT_FAILURE); 
            }
            broadcast=false;
            bytesRead = read(new_socket, buffer, 2048); 
            send(new_socket, hello.c_str(), strlen(hello.c_str()), 0); 
            fcntl(new_socket, F_SETFL, O_NONBLOCK);
        }
        rclcpp::spin_some(nodeHandle);
        rate.sleep();
    }

    broadcastThread.join();

}
