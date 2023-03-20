/*
This scripts initialize the 4 steering motors
The parameters are:

*/

#include "driving_motors.hpp"

DrivingMotors::DrivingMotors()
{

}

int DrivingMotors::connSocket()
{
    // Create the socket 
    if (client_ = socket(AF_INET, SOCK_STREAM,0) < 0)
    {
        ROS_FATAL("Socket creation error")
        return -1;
    }

    serv_addr_.sin_family = AF_INET;
    serv_addr_.sin_port = htons(PORT);
    serv_addr_.sin_addr.s_addr = inet_addr(SERVER_IP);

    // Connect to the server
    while (status_ = connect(client_, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0)
    {
        ROS_ERROR("Socket connection to the server failed");
        status_ = connect(client_, (struct sockaddr*)&serv_addr, sizeof(serv_addr));
    }

    // Connected
    ROS_INFO("Connected to the server successfully");
    return 0;
}


void DrivingMotors::setSpeed()
{
    // bytes_ = DrivingMotors::parser(buffer_);
    uint8_t prueba[13] = {0x08, 0x00, 0x00, 0x00, 0x01, 0x82, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    send(client_, prueba, 13, 0);

}

uint8_t * DrivingMotors::parser(message buffer)
{
    uint8_t bytes_temp[13];
    bytes_temp[0] = buffer.DLC;
    bytes_temp[1] = buffer.NC1;
    bytes_temp[2] = buffer.NC2;
    bytes_temp[3] = buffer.NC3;
    bytes_temp[4] = buffer.ID;
    bytes_temp[5] = buffer.PID;
    bytes_temp[6] = buffer.D1;
    bytes_temp[7] = buffer.D2;
    bytes_temp[8] = buffer.D3;
    bytes_temp[9] = buffer.D4;
    bytes_temp[10] = buffer.D5;
    bytes_temp[11] = buffer.D6;
    bytes_temp[12] = buffer.D7;

    return bytes_temp;
}
