/***
 * Modbus master for communicating over serial port
 */
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include "wit_driver/modbus_srv.h"
#include <chrono>
#define DEFAULT_SERIAL_PORT "/dev/ttyUSB0"
#define DEFAULT_BAUD_RATE 9600


serial::Serial ser;
ros::Publisher read_pub;
//--------------------------------------------------------------------------------------------------
//convert char to integer
int GetNumberFromHexString(char c1)
{
    if (c1 > 60) return c1 - 97 + 10; else return c1 - 48;
}
//--------------------------------------------------------------------------------------------------

//convert one byte number to hex string
std::string GetStringFromHexNumber(unsigned int number)
{
    char br1 = number / 16;
    char br2 = number % 16;
    std::string rez="";

    if (br1 > 9) br1 = 97 + br1 - 10; else br1 = 48 + br1;
    if (br2 > 9) br2 = 97 + br2 - 10; else br2 = 48 + br2;
    rez = rez + br1 + br2;
    return rez;
}


int failed_count = 0;
//list of received messages for sending on bus
std::list<std::string> message_list;
//required response size for messages send on bus
std::list<int> message_size;

//--------------------------------------------------------------------------------------------------
//callback
void write_callback(const std_msgs::String::ConstPtr& msg){
//	ROS_INFO_STREAM("Request "<<req.req );
	message_list.push_back((*msg).data);
	if (message_list.size()>4) message_list.pop_front();
	std::cout<<(*msg).data<<std::endl;
}

//--------------------------------------------------------------------------------------------------
// Write message to bus
int write_message(std::string message)
{

    uint8_t data[50];
    int length=0;
    for (int i = 0;i < message.length();i += 2)
    {
        data[length++] = GetNumberFromHexString(message[i]) * 16 + GetNumberFromHexString(message[i + 1]);
    }
	std::cout<<"written message "<<message<<std::endl;

    //send message over bus
    ser.write(data, length-1);
    return data[length-4]*2+5;
}
//--------------------------------------------------------------------------------------------------
// Read message of desired length from bus
void wait_response(int length)
{
    uint8_t data1[100];

   	std::string result;

	int count = ser.read(data1, length);

	ROS_INFO_STREAM("length "<<count<<" "<<length<< " "<<ser.available());

	result = "";
	//convert data to string
	for (int i = 0; i < count; i++)
	{
		result = result + GetStringFromHexNumber(data1[i]);
	}

	if (result == "")
	{
		ROS_INFO_STREAM("failed receiving " << failed_count);
		failed_count++;
		return;
	}
	else
	{
		failed_count = 0;
	}

	std_msgs::String res;
	res.data=result;

	//publish response on topic
	read_pub.publish(res);
	return;

}

int main (int argc, char** argv){
    ros::init(argc, argv, "modbus_master_serial");
    ros::NodeHandle nh;
    ros::NodeHandle nh_ns("~");

    std::string port, modbus_service;
    int baudrate;
    nh_ns.param("port", port, (std::string) DEFAULT_SERIAL_PORT); 
    nh_ns.param("baudrate", baudrate, DEFAULT_BAUD_RATE);
    nh_ns.param("modbus_service", modbus_service , (std::string) "modbus_service");


    //set subscriber callback
    ros::Subscriber message_subscriber = nh.subscribe(modbus_service+"_write", 1000, write_callback);

    //set publisher callback
    read_pub = nh.advertise<std_msgs::String>(modbus_service+"_read", 1000);

   //open port
    try
    {
        ser.setPort(port);
        ser.setBaudrate(baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(40);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

    ros::Rate loop_rate(30);
    uint8_t data1[200];

    while(ros::ok()){

    	ros::spinOnce();
        ros::spinOnce();
        ros::spinOnce();
        ros::spinOnce();
        ros::spinOnce();
        ros::spinOnce();
        ros::spinOnce();
        ros::spinOnce();
        ros::spinOnce();
        ros::spinOnce();
        ros::spinOnce();

        int size=message_list.size();
        //if there are no messages wait for 1ms
        if (size==0)
        {
        	usleep(1000);
        	continue;
        }
        std::cout<<std::endl<<"message list size "<<message_list.size()<<std::endl;

        //send one message every 5ms
        for (int i=0;i<size;i++)
        {
        	message_size.push_back(write_message(message_list.front()));
        	message_list.pop_front();
        	usleep(5000);
        }


        //wait response
        usleep(12000);
        size=message_size.size();
        for (int i=0;i<size;i++)
        {
        	if (ser.available()>=message_size.front())
        		wait_response(message_size.front());
        	message_size.pop_front();
        }

        //if there are left over messages, read them
        if (ser.available()>0)
        {
        	std::cout<<"left over"<<std::endl;
        	int count = ser.read(data1, ser.available());
        }

        //sleep to 30 Hz
        loop_rate.sleep();


    }
}

