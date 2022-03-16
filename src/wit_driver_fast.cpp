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

#include <tf/tf.h>
int IMU_address;
bool first=true;


ros::Publisher magnetic_field_pub;
ros::Publisher imu_pub;

ros::Publisher write_pub;
//--------------------------------------------------------------------------------------------------
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

//--------------------------------------------------------------------------------------------------
//Convert char to number
int GetNumberFromHexString(char c1)
{
    if (c1 > 60) return c1 - 97 + 10; else return c1 - 48;
}

//--------------------------------------------------------------------------------------------------
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


//--------------------------------------------------------------------------------------------------

sensor_msgs::MagneticField data;
sensor_msgs::Imu imu;

//parse data from message to ROS meesages
void Parse(uint8_t *message, int length)
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
            	// read raw data
                pos++;//year
                pos++;//month
                pos++;//day
                pos++;//hour
                pos++; //minute
                pos++; //second
                pos++; //
                pos++; //

                //read accelerometer
                int value = (unsigned char) message[pos++] * 256 + (unsigned char) message[pos++];
                if (value > 32767) value = value - 65536;
                imu.linear_acceleration.x = value / 32768. * 16 * 9.81;
                value = (unsigned char) message[pos++] * 256 + (unsigned char) message[pos++];
                if (value > 32767) value = value - 65536;
                imu.linear_acceleration.y = value / 32768. * 16 * 9.81;
                value = (unsigned char) message[pos++] * 256 + (unsigned char) message[pos++];
                if (value > 32767) value = value - 65536;
                imu.linear_acceleration.z = value / 32768. * 16 * 9.81;

                //read gyro
                value = (unsigned char) message[pos++] * 256 + (unsigned char) message[pos++];
                if (value > 32767) value = value - 65536;
                imu.angular_velocity.x = value / 32768. * 2000;
                value = (unsigned char) message[pos++] * 256 + (unsigned char) message[pos++];
                if (value > 32767) value = value - 65536;
                imu.angular_velocity.y = value / 32768. * 2000;
                value = (unsigned char) message[pos++] * 256 + (unsigned char) message[pos++];
                if (value > 32767) value = value - 65536;
                imu.angular_velocity.z = value / 32768. * 2000;

                //read magnetic field
                data.magnetic_field.x = (unsigned char) message[pos++] * 256 + (unsigned char) message[pos++];
                data.magnetic_field.y = (unsigned char) message[pos++] * 256 + (unsigned char) message[pos++];
                data.magnetic_field.z = (unsigned char) message[pos++] * 256 + (unsigned char) message[pos++];
                if (data.magnetic_field.x > 32767) data.magnetic_field.x = data.magnetic_field.x - 65536;
                if (data.magnetic_field.y > 32767) data.magnetic_field.y = data.magnetic_field.y - 65536;
                if (data.magnetic_field.z > 32767) data.magnetic_field.z = data.magnetic_field.z - 65536;

                //read roll pitch yaw
              	double roll=(unsigned char) message[pos++] * 256 + (unsigned char) message[pos++];
              	double pitch=(unsigned char) message[pos++] * 256 + (unsigned char) message[pos++];
              	double yaw=(unsigned char) message[pos++] * 256 + (unsigned char) message[pos++];
                if (roll > 32767) roll = roll - 65536;
                if (pitch > 32767) pitch = pitch - 65536;
                if (yaw > 32767) yaw = yaw - 65536;
                roll=roll/32768*3.14159265;
                pitch=pitch/32768*3.14159265;
                yaw=yaw/32768*3.14159265;

                //convert roll pitch yaw to quaternion
                tf:: Quaternion rotation_rpy;
                rotation_rpy.setRPY(roll,pitch,yaw);
                imu.orientation.x=rotation_rpy.x();
                imu.orientation.y=rotation_rpy.y();
                imu.orientation.z=rotation_rpy.z();
                imu.orientation.w=rotation_rpy.w();

                //add stamp and publish imu and imu magnetic
                data.header.stamp = ros::Time::now();
                magnetic_field_pub.publish(data);
                imu.header.stamp = ros::Time::now();
              	imu_pub.publish(imu);
            }
         }
    }
     
}

//--------------------------------------------------------------------------------------------------

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

//--------------------------------------------------------------------------------------------------
// read response from modbus request
void read_callback(const std_msgs::String::ConstPtr& msg){
    uint8_t data1[100];
	std_msgs::String response = *msg;
    std::cout<<"response driver "<<  response.data<<std::endl;
    int length = SplitString(response.data,data1);

    //parse response
    Parse(data1, length);
}

//--------------------------------------------------------------------------------------------------
int main (int argc, char** argv){
    ros::init(argc, argv, "wit_driver");
    ros::NodeHandle nh;
    ros::NodeHandle nh_ns("~");

    int refresh_rate;
    std::string address, modbus_service, write_topic, read_topic;
    int mag_offset_x, mag_offset_y, mag_offset_z;
    nh_ns.param("address", IMU_address, 80); 
    nh_ns.param("refresh_rate", refresh_rate,100);
    nh_ns.param("mag_offset_x", mag_offset_x,5);
    nh_ns.param("mag_offset_y", mag_offset_y,6);
    nh_ns.param("mag_offset_z", mag_offset_z,7);
    nh_ns.param("write_topic", write_topic, (std::string) "write_topic0");
    nh_ns.param("read_topic", read_topic, (std::string) "read_topic0");
    nh_ns.param("modbus_service", modbus_service , (std::string) "modbus_service");

    ROS_INFO_STREAM("client  "<<modbus_service);


    char a=48+IMU_address-0x50;
    std::string published_topic_name = "imu_magnetic";
    published_topic_name = published_topic_name + a;
    std::string published_imu_topic_name = "imu";
    published_imu_topic_name = published_imu_topic_name + a;

    //magnetic field topic publisher
    magnetic_field_pub = nh.advertise<sensor_msgs::MagneticField>(published_topic_name, 1000);

    //imu topic publisher
    imu_pub = nh.advertise<sensor_msgs::Imu>(published_imu_topic_name, 1000);

    //publisher for modbus request
    write_pub = nh.advertise<std_msgs::String>(modbus_service+"_write", 1000);
    ros::Subscriber message_subscriber = nh.subscribe(modbus_service+"_read", 1000, read_callback);

    ros::Rate loop_rate(refresh_rate);

    uint8_t data1[50];
    uint8_t data[20]={0x50,0x03,0x00,0x30,0x00,0x10,0x00,0x00,0x69,0x85,0x10};

    //calculate message CRC
    data[0]=IMU_address;
    int crc=ModRTU_CRC(data, 6);
    data[7]=crc/256;
    data[6]=crc%256;


    std_msgs::String result;

    uint8_t data_received[50];
    int counter=0;
    while(ros::ok()){

        ros::spinOnce();
	    //set message to send
	    result.data = "";
	    for (int i = 0; i < 9; i++)
	    {
	    	result.data = result.data + GetStringFromHexNumber(data[i]);
	    }
	
	    write_pub.publish(result);
    	loop_rate.sleep();
	

    }
}


