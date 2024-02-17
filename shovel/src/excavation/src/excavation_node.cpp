#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>

#include "messages/msg/linear_out.hpp"

rclcpp::Node::SharedPtr nodeHandle;

std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > talon14Publisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > talon15Publisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > talon16Publisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > talon17Publisher;

/** @file
 * @brief Node to control excavation motors
 * 
 * This node receives information intended to control the
 * linear actuators, then modifies the data to synchronize the
 * two linear actuators attached to the excavation assembly. 
 * This node also sends information about the linear actuators
 * back to the client-side GUI to integrate information about the
 * position data and error state. This node subscribes to the 
 * following topics:
 * \li \b potentiometer_data
 * \li \b shoulder_speed
 * \li \b dump_speed
 * \li \b automationGo
 * 
 * 
 * This node publishes the following topics:
 * \li \b talon_14_speed
 * \li \b talon_15_speed
 * \li \b talon_16_speed
 * \li \b talon_17_speed
 * \li \b linearOut1
 * \li \b linearOut2
 * \li \b linearOut3
 * \li \b linearOut4
 * 
 */


enum Error {
    ActuatorsSyncError,
    ActuatorNotMovingError,
    PotentiometerError,
    ConnectionError,
    None
};


std::map<Error, const char*> errorMap = {{ActuatorsSyncError, "ActuatorsSyncError"},
    {ActuatorNotMovingError, "ActuatorNotMovingError"},
    {PotentiometerError, "PotentiometerError"},
    {ConnectionError, "ConnectionError"},
    {None, "None"}};


struct LinearActuator{
    float speed = 0.0;
    int potentiometer = 0;
    int timeWithoutChange = 0;
    int max = 0;
    int min = 1024;
    int count = 0;
    Error error = ConnectionError;
    bool run = true;
    bool atMin = false;
    bool atMax = false;
    float stroke = 11.8;
    float extended = 0.0;
};


LinearActuator linear1;
LinearActuator linear2;
LinearActuator linear3;
LinearActuator linear4;

float currentSpeed = 0;
int thresh1 = 60;
int thresh2 = 120;
int thresh3 = 180;

bool automationGo = false;


/** @brief Function to sync the linear actuators
 * 
 * The sync function works by checking if the currentSpeed is
 * greater than zero. If the speed is greater than zero, the val
 * checks which actuator is more extended and sets the speed of
 * the actuator to a lower value if the diff is greater than the 
 * thresh values.  If the value is less than zero, the val checks 
 * which actuator is less extended and sets the speed of the 
 * actuator to a lower value. The function then publishes the 
 * updated speed.
 * @return void
 * */
void sync(LinearActuator *linear1, LinearActuator *linear2){
    float diff = abs(linear1->potentiometer - linear2->potentiometer);
    // Might change this from ternary to if statements to improve readability
    bool val = (currentSpeed > 0) ? (linear1->potentiometer > linear2->potentiometer) : (linear1->potentiometer < linear2->potentiometer);
    if (diff > thresh3){
        (val) ? linear1->speed = 0 : linear2->speed = 0;
    }
    else if (diff > thresh2){
        (val) ? linear1->speed *= 0.5 : linear2->speed *= 0.5;
    }
    else if (diff > thresh1){
        (val) ? linear1->speed *= 0.9 : linear2->speed *= 0.9;
    }
    else{
        linear1->speed = currentSpeed;
        linear2->speed = currentSpeed;
    }
}


void setSpeeds(LinearActuator *linear1, LinearActuator *linear2){
    if(!automationGo){
        linear1->speed = currentSpeed;
        linear2->speed = currentSpeed;
    }
    else{
        if(linear1->error != ConnectionError && linear1->error != PotentiometerError && linear2->error != ConnectionError && linear2->error != PotentiometerError){
            linear1->speed = currentSpeed;
            linear2->speed = currentSpeed;
        }
    }
    if(linear1->error != ConnectionError && linear1->error != PotentiometerError && linear2->error != ConnectionError && linear2->error != PotentiometerError){
        sync(linear1, linear2);
        if(linear1->atMax && currentSpeed > 0){
            linear1->speed = 0.0;
        }
        if(linear2->atMax && currentSpeed > 0){
            linear2->speed = 0.0;
        }
        if(linear1->atMin && currentSpeed < 0){
            linear1->speed = 0.0;
        }
        if(linear2->atMin && currentSpeed < 0){
            linear2->speed = 0.0;
        }
    }
}


/** @brief Function to set potentiometer error
 * 
 * Thsi function is used to set the value of the error
 * of the linear object.  If the potentiometer is equal
 * to 1024, which is the value that occurs when the
 * potentiometer is disconnected from the Arduino. Refer
 * to the ErrorState state diagram for more information.
 * @param potentData - Int value of potentiometer
 * @param *linear - Pointer to linear object
 * @return void
 * */
void setPotentiometerError(int potentData, LinearActuator *linear){
    if(msg->data == -1){
        linear->error = ConnectionError;
    }
    else{
        if(linear->error == ConnectionError){
            linear->error = None;
        }
    }
    if(potentData == 1024){
        linear->error = PotentiometerError;
        RCLCPP_INFO(nodeHandle->get_logger(),"EXCAVATION ERROR: PotentiometerError");
    }
    else{
        if(linear->error == PotentiometerError){
            linear->error = None;
        }
    }
}


/** @brief Function to process potentiometer data
 * 
 * This function processes the passed potentiometer data
 * and adjusts the passed linear values accordingly. First
 * the function sets the min and max values if the new data
 * is beyond the previous limits. Next, the function checks
 * if the value is within a threshold of the previous value
 * that is stored in the linear->potentiometer variable. If
 * the value is within this threshold, it's assumed that 
 * the actuator isn't moving. If the speed isn't equal to
 * zero, ie the actuator should be moving, the count
 * variable gets increased. If the count is greater than 5,
 * the function checks if the actuator is at the min or max
 * positions and sets the corresponding values to true if
 * it is.  If the data is outside of the threshold, the 
 * actuator is moving as intended and is not at the min or
 * max positions.
 * @param potentData - Int value of potentiometer
 * @param *linear - Pointer to linear object
 * @return void
 * */
void processPotentiometerData(int potentData, LinearActuator *linear){
    if(potentData < linear->min){
        linear->min = potentData;
    }

    if(potentData > linear->max){
        linear->max = potentData;
    }

    if(linear->potentiometer >= potentData - 10 && linear->potentiometer <= potentData + 10){
        if(linear->speed != 0.0){
            linear->count += 1;
            if(linear->count >= 5){
                if(linear->max > 800 && linear->speed > 0.0 && potentData >= linear->max - 20){
                    linear->atMax = true;
                    linear->count = 0;
                }
                else if(linear->min < 200 && linear->speed < 0.0 && potentData <= linear->min + 20){
                    linear->atMin = true;
                    linear->count = 0;
                }
                else{
                    if(linear->error == None || linear->error == ActuatorsSyncError){
                        linear->error = ActuatorNotMovingError;
                        RCLCPP_INFO(nodeHandle->get_logger(),"EXCAVATION ERROR: ActuatorNotMovingError");
                    }
                }
            }
        }
    }
    else{
        linear->count = 0;
        if(linear->error == ActuatorNotMovingError){
            linear->error = None;
        }
        if(linear->atMax){
            if(linear->speed < 0.0){
                linear->atMax = false;
            }
        }
        else{
            linear->atMax = false;
        }
        if(linear->atMin){
            if(linear->speed > 0.0){
                linear->atMin = false;
            }
        }
        else{
            linear->atMin = false;
        }
    }
    linear->potentiometer = potentData;
}


/** @brief Callback function for the automationGo topic
 * 
 * This function sets the automationGo value to the value
 * in the message.
 * @param msg - ROS2 message containing automationGo value
 * @return void
 * */
void automationGoCallback(const std_msgs::msg::Bool::SharedPtr msg){
    automationGo = msg->data;
}


void setSyncErrors(LinearActuator *linear1, LinearActuator *linear2){
    if(abs(linear1->potentiometer - linear2->potentiometer) > thresh1){
        if(linear1->error == None){
            linear1->error = ActuatorsSyncError;
        }
        if(linear2->error == None){
            linear2->error = ActuatorsSyncError;
        }
    }
    else{
        if(linear1->error == ActuatorsSyncError){
            linear1->error = None;
        }
        if(linear2->error == ActuatorsSyncError){
            linear2->error = None;
        }
    }
    sync(linear1, linear2);
}


void potentiometer1Callback(const std_msgs::msg::Int16::SharedPtr msg){
    setPotentiometerError(msg->data, &linear1);

    if(linear1.error != ConnectionError && linear1.error != PotentiometerError && linear2.error != ConnectionError && linear2.error != PotentiometerError){
        processPotentiometerData(msg->data, &linear1);
        setSyncErrors(&linear1, &linear2);
    }
}


void potentiometer2Callback(const std_msgs::msg::Int16::SharedPtr msg){
    setPotentiometerError(msg->data, &linear2);

    if(linear1.error != ConnectionError && linear1.error != PotentiometerError && linear2.error != ConnectionError && linear2.error != PotentiometerError){
        processPotentiometerData(msg->data, &linear2);
        setSyncErrors(&linear1, &linear2);
    }
}


void potentiometer3Callback(const std_msgs::msg::Int16::SharedPtr msg){
    setPotentiometerError(msg->data, &linear3);

    if(linear3.error != ConnectionError && linear3.error != PotentiometerError){
        processPotentiometerData(msg->data, &linear3);
        setSyncErrors(&linear3, &linear4);
    }
}


void potentiometer4Callback(const std_msgs::msg::Int16::SharedPtr msg){
    setPotentiometerError(msg->data, &linear4);

    if(linear4.error != ConnectionError && linear4.error != PotentiometerError){
        processPotentiometerData(msg->data, &linear4);
        setSyncErrors(&linear3, &linear4);
    }
}


void armSpeedCallback(const std_msgs::msg::Float32::SharedPtr msg){

}


void bucketSpeedCallback(const std_msgs::msg::Float32::SharedPtr msg){

}


/** @brief Function to get the LinearOut values
 * 
 * This function sets the values of the LinearOut message
 * with the values from the linear actuator. 
 * @param *linearOut - Pointer for the LinearOut object
 * @param *linear - Pointer for the linear actuator
 * @return void
 * */
void getLinearOut(messages::msg::LinearOut *linearOut, LinearActuator *linear){
    linearOut->speed = linear->speed;
    linearOut->potentiometer = linear->potentiometer;
    linearOut->time_without_change = linear->timeWithoutChange;
    linearOut->max = linear->max;
    linearOut->min = linear->min;
    linearOut->error = errorMap.at(linear->error);
    linearOut->run = linear->run;
    linearOut->at_min = linear->atMin;
    linearOut->at_max = linear->atMax;
}


int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    nodeHandle = rclcpp::Node::make_shared("excavation");

    auto automationGoSubscriber = nodeHandle->create_subscription<std_msgs::msg::Bool>("automationGo",1,automationGoCallback);

    auto armSpeedSubscriber = nodeHandle->create_subscription<std_msgs::msg::Float32>("arm_speed",1,armSpeedCallback);
    auto bucketSpeedSubscriber = nodeHandle->create_subscription<std_msgs::msg::Float32>("bucket_speed",1,bucketSpeedCallback);

    auto potentiometerDataSubscriber1 = nodeHandle->create_subscription<std_msgs::msg::Int16>("potentiometer_1_data",1,potentiometer1Callback);
    auto potentiometerDataSubscriber2 = nodeHandle->create_subscription<std_msgs::msg::Int16>("potentiometer_2_data",1,potentiometer2Callback);
    auto potentiometerDataSubscriber3 = nodeHandle->create_subscription<std_msgs::msg::Int16>("potentiometer_3_data",1,potentiometer3Callback);
    auto potentiometerDataSubscriber4 = nodeHandle->create_subscription<std_msgs::msg::Int16>("potentiometer_4_data",1,potentiometer4Callback);


    talon14Publisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("talon_14_speed",1);
    talon15Publisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("talon_15_speed",1);
    talon16Publisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("talon_16_speed",1);
    talon17Publisher = nodeHandle->create_publisher<std_msgs::msg::Float32>("talon_17_speed",1);
    
    messages::msg::LinearOut linearOut1;
    messages::msg::LinearOut linearOut2;
    messages::msg::LinearOut linearOut3;
    messages::msg::LinearOut linearOut4;

    auto linearOut1Publisher = nodeHandle->create_publisher<messages::msg::LinearOut>("linearOut1",1);
    auto linearOut2Publisher = nodeHandle->create_publisher<messages::msg::LinearOut>("linearOut2",1);
    auto linearOut3Publisher = nodeHandle->create_publisher<messages::msg::LinearOut>("linearOut3",1);
    auto linearOut4Publisher = nodeHandle->create_publisher<messages::msg::LinearOut>("linearOut4",1);

    auto start = std::chrono::high_resolution_clock::now();
    while(rclcpp::ok()){
        auto finish = std::chrono::high_resolution_clock::now();
        if(std::chrono::duration_cast<std::chrono::seconds>(finish-start).count() > 2){
            getLinearOut(&linearOut1, &linear1);
            linearOut1Publisher->publish(linearOut1);

            getLinearOut(&linearOut2, &linear2);
            linearOut2Publisher->publish(linearOut2);

            getLinearOut(&linearOut3, &linear3);
            linearOut3Publisher->publish(linearOut3);

            getLinearOut(&linearOut4, &linear4);
            linearOut4Publisher->publish(linearOut4);
        }
        rclcpp:spin_some(nodeHandle);
    }
}