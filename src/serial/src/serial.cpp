# include "ros/ros.h"
# include "serial/cmd.h"
# include "../include/CSerialComp.h"
# include <string>

// define a Port
CSerialComp port = CSerialComp();
// define callback function
bool handle_function(serial::cmd::Request &req,
					serial::cmd::Response &res)
{
	// define a char array to send data
    char send[50];
    char recv;
    // define reply string
    std::string reply = "";
    //define var to send
    int nBytesSent = 0, sendlength = req.cmd.length();

	strcpy(send, req.cmd.data());
	if(port.serialOpen()){
		nBytesSent = port.serialWrite(send, sendlength);
		if(nBytesSent == sendlength){
			while(true){
				if(port.serialRead(&recv)){
					if(recv == '$'){break;}
					else{reply = reply + recv;}
				}
			}
		}
		else{ROS_INFO("Serial: Failed to send cmd!");}
		res.reply = reply;
		reply = "";
	}
	else
	    ROS_INFO("Serial: Failed to open port!");
	
	return true;
}

int main(int argc, char **argv)
{
	
	// initial node
	ros::init(argc, argv, "serial");
	
	// define the reply function
	ros::NodeHandle nh;
	ros::ServiceServer service = nh.advertiseService("serial", handle_function);
	
	// spin
	ros::spin();

	return 0;
}
