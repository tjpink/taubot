#ifndef TAUBOT_HARDWARE_SERIAL_COMM_H
#define TAUBOT_HARDWARE_SERIAL_COMM_H
 
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <cmath>
 
class SerialComms
{
  private:
    bool   	serial_connected;
    int    	serial_port;
    struct  	termios tty;
    int16_t	encLeftTicks;
    int16_t	encRightTicks;

    double	cmdVelL;
    double     	cmdVelR;
    double	distL;
    double	distR;

  public:
    SerialComms();
    void setup(std::string uart_port);
    // void setPidValues(float k_p, float k_d, float k_i, float k_o);
    bool connected();
    bool serialConnection();
    void sendMsg(char* msg, int len);
    void closeComms();
    int receiveMsg(int len);
    double getCmdVelL();
    double getCmdVelR();
    double getDistL();
    double getDistR();
    int16_t getEncLeftTicks();
    int16_t getEncRightTicks();

    int dtoa(char* str, double input, int strlen);
};
 
#endif // TAUBOT_HARDWARE_SERIAL_COMMS_H

