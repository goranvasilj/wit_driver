/***
 * Node for communicating with wit motion IMU over Modbus
 */

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h>
#include "wit_driver/modbus_srv.h"

int IMU_address;

ros::Publisher magnetic_field_pub;
ros::Publisher imu_pub;

ros::Publisher write_pub;

// Compute the MODBUS RTU CRC
uint16_t ModRTU_CRC(uint8_t* buf, int len)
{
  uint16_t crc = 0xFFFF;

  for (int pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];          // XOR byte into least sig. byte of crc

    for (int i = 8; i != 0; i--) {    // Loop over each bit
      if ((crc & 0x0001) != 0) {      // If the LSB is set
        crc >>= 1;                    // Shift right and XOR 0xA001
        crc ^= 0xA001;
      }
      else                            // Else LSB is not set
        crc >>= 1;                    // Just shift right
    }
  }
  // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
  return crc;
}

//Convert char to number
int GetNumberFromHexString(char c1)
{
    if (c1 > 60) return c1 - 97 + 10; else return c1 - 48;
}

//Convert one byte number to hex string
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
sensor_msgs::MagneticField data;
sensor_msgs::Imu imu;

//parse data from message to ROS meesages
//raw = true  - reads raw data
//raw = false - reads quaternion
void Parse(uint8_t *message, int length, bool raw)
{

    int pos = 0;
    if (message[pos] == IMU_address && length>5)
    {
    	//check CRC
    	uint16_t crc=ModRTU_CRC(message, length-2);
    	if (crc%256 != message[length-2] || crc/256 != message[length-1]) return;
    	pos++;
    	//if message is read data
        if (message[pos++] == 3)
    	{
        	//check message length
    	    int count=message[pos++];
            if (length-pos >= count + 2)
            {
            	if (raw == true)
            	{
            		// read raw data
                	pos++;//year
                	pos++;//month
                	pos++;//day
                	pos++;//hour
                	pos++; //minute
                	pos++; //second
                	pos++; //
                	pos++; //


                	int value = (unsigned char) message[pos++] * 256 + (unsigned char) message[pos++];
                	if (value > 32767) value = value - 65536;
                	imu.linear_acceleration.x = value / 32768. * 16 * 9.81;
                	value = (unsigned char) message[pos++] * 256 + (unsigned char) message[pos++];
                	if (value > 32767) value = value - 65536;
                	imu.linear_acceleration.y = value / 32768. * 16 * 9.81;
                	value = (unsigned char) message[pos++] * 256 + (unsigned char) message[pos++];
                	if (value > 32767) value = value - 65536;
                	imu.linear_acceleration.z = value / 32768. * 16 * 9.81;
                	pos++;pos++; //absolute a
                	value = (unsigned char) message[pos++] * 256 + (unsigned char) message[pos++];
                	if (value > 32767) value = value - 65536;
                	imu.angular_velocity.x = value / 32768. * 2000;
                	value = (unsigned char) message[pos++] * 256 + (unsigned char) message[pos++];
                	if (value > 32767) value = value - 65536;
                	imu.angular_velocity.y = value / 32768. * 2000;
                	value = (unsigned char) message[pos++] * 256 + (unsigned char) message[pos++];
                	if (value > 32767) value = value - 65536;
                	imu.angular_velocity.z = value / 32768. * 2000;
                	pos++;pos++; //absolute w

                	data.magnetic_field.x = (unsigned char) message[pos++] * 256 + (unsigned char) message[pos++];
                    data.magnetic_field.y = (unsigned char) message[pos++] * 256 + (unsigned char) message[pos++];
                    data.magnetic_field.z = (unsigned char) message[pos++] * 256 + (unsigned char) message[pos++];
                    if (data.magnetic_field.x > 32767) data.magnetic_field.x = data.magnetic_field.x - 65536;
                    if (data.magnetic_field.y > 32767) data.magnetic_field.y = data.magnetic_field.y - 65536;
                    if (data.magnetic_field.z > 32767) data.magnetic_field.z = data.magnetic_field.z - 65536;
                	pos++; //roll
                	pos++; //pitch
                	pos++; //yaw
                	pos++; //T

                    data.header.stamp = ros::Time::now();
                    magnetic_field_pub.publish(data);
            	}
            	else
            	{
            		//read quaternion
                	int value = (unsigned char) message[pos++] * 256 + (unsigned char) message[pos++];
                	if (value > 32767) value = value - 65536;
                	imu.orientation.w = value / 32768.;
                	value = (unsigned char) message[pos++] * 256 + (unsigned char) message[pos++];
                	if (value > 32767) value = value - 65536;
                	imu.orientation.x = value / 32768.;
                	value = (unsigned char) message[pos++] * 256 + (unsigned char) message[pos++];
                	if (value > 32767) value = value - 65536;
                	imu.orientation.y = value / 32768.;
                	value = (unsigned char) message[pos++] * 256 + (unsigned char) message[pos++];
                	if (value > 32767) value = value - 65536;
                	imu.orientation.z = value / 32768.;
                    imu.header.stamp = ros::Time::now();

                	imu_pub.publish(imu);

            	}
            }
         }
    }
     
}

//split string with hex data to uint8_t
int SplitString(std::string message,uint8_t *data)
{
    int length = 0;
    for (int i = 0; i < message.length(); i += 2)
    {
        data[length++] = GetNumberFromHexString(message[i]) * 16 + GetNumberFromHexString(message[i+1]);
    }
    return length;
}



int main (int argc, char** argv){
    ros::init(argc, argv, "wit_driver");
    ros::NodeHandle nh;
    ros::NodeHandle nh_ns("~");

    int refresh_rate;
    std::string address, modbus_service, write_topic, read_topic;
    nh_ns.param("address", IMU_address, 80); 
    nh_ns.param("refresh_rate", refresh_rate,100);
    nh_ns.param("write_topic", write_topic, (std::string) "write_topic0");
    nh_ns.param("read_topic", read_topic, (std::string) "read_topic0");
    nh_ns.param("modbus_service", modbus_service , (std::string) "modbus_service");

    ROS_INFO_STREAM("client  "<<modbus_service);


    char a=48+IMU_address-0x50;
    std::string published_topic_name = "imu_magnetic";
    published_topic_name = published_topic_name + a;
    std::string published_imu_topic_name = "imu";
    published_imu_topic_name = published_imu_topic_name + a;

    magnetic_field_pub = nh.advertise<sensor_msgs::MagneticField>(published_topic_name, 1000);

    imu_pub = nh.advertise<sensor_msgs::Imu>(published_imu_topic_name, 1000);


    ros::Rate loop_rate(refresh_rate);

    uint8_t data1[50];
    //command for magnetometer
    //    uint8_t data[20]={0x50,0x03,0x00,0x3a,0x00,0x04,0x00,0x00,0x69,0x85,0x0d};
    //command for all data
     uint8_t data[20]={0x50,0x03,0x00,0x30,0x00,0x0d,0x00,0x00,0x69,0x85,0x0d};
     //command for quaternion
     uint8_t dataquat[20]={0x50,0x03,0x00,0x51,0x00,0x04,0x00,0x00,0x69,0x85,0x0d};


    data[0]=IMU_address;
    int crc=ModRTU_CRC(data, 6);
    data[7]=crc/256;
    data[6]=crc%256;

    dataquat[0]=IMU_address;
    crc = ModRTU_CRC(dataquat, 6);
    dataquat[7]=crc/256;
    dataquat[6]=crc%256;

    ros::ServiceClient client = nh.serviceClient<wit_driver::modbus_srv>(modbus_service);

    std_msgs::String result;

    wit_driver::modbus_srv srv ;
    uint8_t data_received[50];
    int counter=0;
    bool raw=false;
    while(ros::ok()){

        ros::spinOnce();

        //set message to send
        result.data = "";
        for (int i = 0; i < 9; i++)
        {
        	if (counter % 2 ==0)
        	{
        		result.data = result.data + GetStringFromHexNumber(data[i]);
        		raw=true;
        	}
        	else
        	{
            	result.data = result.data + GetStringFromHexNumber(dataquat[i]);
            	raw = false;
        	}
        }  
        ROS_INFO_STREAM(result);
        srv.request.req = result.data;
        //call service
        if (client.call(srv))
        {
          std::cout<<"response "<<  srv.response.res<<std::endl;
          int length = SplitString(srv.response.res,data1);
          Parse(data1, length, raw);
        }
        else
        {
          ROS_ERROR("Failed to call service");
        }
        counter++;
       if (counter % 2 > 0)
       {
    	   loop_rate.sleep();
       }
	

    }
}


