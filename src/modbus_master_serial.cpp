/***
 * Modbus master for communicating over serial port
 */
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include "wit_driver/modbus_srv.h"
#define DEFAULT_SERIAL_PORT "/dev/ttyUSB0"
#define DEFAULT_BAUD_RATE 9600


serial::Serial ser;
ros::Publisher read_pub;

//convert char to integer
int GetNumberFromHexString(char c1)
{
    if (c1 > 60) return c1 - 97 + 10; else return c1 - 48;
}

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

//call this on service request
int failed_count = 0;
bool write_callback(wit_driver::modbus_srv::Request &req,
		wit_driver::modbus_srv::Response &res){
	ROS_INFO_STREAM("Request "<<req.req );
    uint8_t data[50];
    uint8_t data1[50];
    int length=0;
    for (int i = 0;i < req.req.length();i += 2)
    {
        data[length++] = GetNumberFromHexString(req.req[i]) * 16 + GetNumberFromHexString(req.req[i + 1]);
    }
    //send message over bus
    ser.write(data, length-1);
    
    //read message
   	std::string result;
	int count = ser.read(data1, data[length-1]);
	if (ser.available()>0 && ser.available() + count < 50)
	{
		ser.read(data1 + count, ser.available());
	}

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
		return false;
	}
	else
	{
		failed_count = 0;
	}
	//set response
	res.res = result;
	return true;

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

    //set service callback
    ros::ServiceServer service = nh.advertiseService(modbus_service, write_callback);

   //open port
    try
    {
        ser.setPort(port);
        ser.setBaudrate(baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(30);
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

    ros::Rate loop_rate(1000);


    while(ros::ok()){

        ros::spinOnce();
        if (failed_count > 10)
        {
        	//reopen port if in fault
        	ser.close();
        	ser.open();
        	failed_count = 0;
        }
        usleep(500);
        loop_rate.sleep();

    }
}

