# include "ros/ros.h"
#include "serial/cmd.h"
#include <linux/input.h> // io devices
#include <unistd.h> // unix api
#include <fcntl.h> // unix api
#include <string>

int main(int argc,char** argv)
{
    //wheel direction
    int left_direction = 1, right_direction = 1;
    //PWM_speed
    unsigned int speed = 255;
    //obtain keyboard
    int keys_fd;
    struct input_event t;
    auto size = sizeof(struct input_event);
    keys_fd = open("/dev/input/event6", O_RDONLY);
    if(keys_fd <= 0)
    {   
        ROS_INFO("Controller: Fail to open keyboard!");
        return -1; 
    }

    //initial controller node
    ros::init(argc, argv, "controller");
    //define service
    ros::NodeHandle nh;
	ros::ServiceClient client = nh.serviceClient<serial::cmd>("serial");
    //define a service(command)
    serial::cmd com;

    while(true)
    {   
        read(keys_fd, &t, size);//key_value: Q:16, E:18, A:30, D: 32
        // std::cout<<t.code<<std::endl;
        // std::cout<<t.value<<std::endl;
        // std::cout<<t.type<<std::endl;
        // std::cout<<"----------------------"<<std::endl;
        if(t.type != EV_KEY){continue;}
        if(t.code == 30){
            ROS_INFO("qqq");
            if(t.value == 2){continue;}
            if(t.value == 0)
                {com.request.cmd = "k 0\r";}
            else
                {com.request.cmd = "k "+std::to_string(left_direction*speed)+"\r";}
            client.call(com);
            if(com.response.reply != "ok"){
                ROS_INFO("Controller: Fail to execute command!");
            }
            continue;
        }
        if(t.code == 32){
            if(t.value == 2){continue;}
            if(t.value == 0)
                {com.request.cmd = "l 0\r";}
            else
                {com.request.cmd = "l "+std::to_string(right_direction*speed)+"\r";}
            client.call(com);
            if(com.response.reply != "ok"){
                ROS_INFO("Controller: Fail to execute command!");
            }
            continue;
        }
        if(t.code == 16){
            if(t.value == 0 || t.value == 2){continue;}
            left_direction = -left_direction;
            continue;
        }
        if(t.code == 18){
            if(t.value == 0 || t.value == 2){continue;}
            right_direction = -right_direction;
            continue;
        }
        if(t.code == 22){
            if(t.value == 0 || t.value == 2){continue;}
            if(speed < 250){speed += 5;}
            else{speed = 255;}
            continue;
        }
        if(t.code == 49){
            if(t.value == 0 || t.value == 2){continue;}
            if(speed > 5){speed -= 5;}
            else{speed = 0;}
            continue;
        }
        if(t.code == 1){break;}
        if(t.value == 1){ROS_INFO("only keys:Q,E,A,D,U,N are supported.");}
    }
    //close keyboard and exit
    ROS_INFO("exit safely.");
    close(keys_fd);
    return 0;
}