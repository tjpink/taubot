#include "serial_comms/serial_comms.hpp"

SerialComms::SerialComms() : serial_connected(false), serial_port(0), cmdVelL(0), cmdVelR(0), distL(0), distR(0) { }

void SerialComms::setup(std::string uart_port)
{
  serial_port = open(uart_port.c_str(),  O_RDWR);
  
  if(tcgetattr(serial_port, &tty) != 0)
  {
    printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    serial_connected=false;
  }
  else
  {
    serial_connected=true;
  }

  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CREAD | CLOCAL;
  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;
  tty.c_lflag &= ~ECHOE;
  tty.c_lflag &= ~ECHONL;
  tty.c_lflag &= ~ISIG;
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
  tty.c_oflag &= ~OPOST;
  tty.c_oflag &= ~ONLCR;
  tty.c_cc[VTIME] = 10; // tenths of a second elapses between bytes, see http://unixwiz.net/techtips/termios-vmin-vtime.html
  tty.c_cc[VMIN] = 85;  // characters have been received, with no more data available
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);
}

// void setPidValues(float k_p, float k_d, float k_i, float k_o)
// {;}

bool SerialComms::connected()
{ 
  return serial_connected; 
}

bool SerialComms::serialConnection()
{
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0)
  {
    std::cout << "error " << errno << " from tcsetattr: " << strerror(errno) << std::endl;
    serial_connected=false;
  }   
  else
  {
    serial_connected=true;
  }
  return serial_connected;
}

void SerialComms::sendMsg(char* msg, int len)
{
  write(serial_port, msg, len);
  //usleep(2000); 	// wait need for transmit all data to uController before the next cycle 
}

void SerialComms::closeComms()
{
  close(serial_port);
}

int SerialComms::receiveMsg(int len)
{
  // For variable length array
  char * read_buf;
  read_buf = (char *)calloc(len, sizeof(char));
  // Read uart receive buffer
  int num_bytes = read(serial_port, read_buf, len);
  if (num_bytes < 0)
  {
    std::cout << "Error reading: " << strerror(errno) << std::endl;
    free(read_buf);
    return 1;
  }

  // Extract string and convert to double
  std::string str_ = (char*) read_buf; 	// convert to string for using substr method
//  std::cout << "read_buf is : " << read_buf << std::endl; 
  std::string encLeftTicks_str = str_.substr(0, 6);
  std::string encRightTicks_str = str_.substr(6, 6);
  encLeftTicks = std::stod(encLeftTicks_str);
  encRightTicks = std::stod(encRightTicks_str);

  free(read_buf);
  return 0;
}

double SerialComms::getCmdVelL()
{
  return cmdVelL;
}
double SerialComms::getCmdVelR()
{
  return cmdVelR;
}
double SerialComms::getDistL()
{
  return distL;
}
double SerialComms::getDistR()
{
  return distR;
}

int16_t SerialComms::getEncLeftTicks()
{
  return encLeftTicks;
}

int16_t SerialComms::getEncRightTicks()
{
  return encRightTicks;
}

int SerialComms::dtoa(char* str, double input, int strlen)  // Limitation -999999999.9999999999 to 9999999999.9999999999
{
        if(input >999999999 || input <-999999999)
        {
                printf(" Error: Input number shall be limited between -999,999,999 till 999,999,999");
                return 1;
        }

        int digit_max = (strlen - 2)/2;         // Number of digits equal for both before and after decimal point.
                                                // e.g. strlen=22, minus decimal point minus '\0' = (22-1-1)/2=10 digits on each side
        int32_t int_num;
        int64_t deci_num; // 32bit only until 2,147,483,647, therefore need 64bit to reach 9,999,999,999
        float deci_num_f;
        //char int_array[digit_max] = {0};
        // For variable length array
        char * int_array;
        int_array = (char*)calloc(digit_max, sizeof(char));
        int_array[0] = '0';     // For the first character, if hte number start with 0.XXXXX

        // char deci_array[digit_max] = {0};
        // For variable length array
        char * deci_array;
        deci_array = (char*)calloc(digit_max, sizeof(char));

        int8_t i = 0;
        int8_t a = 0;

        int_num = (int32_t) input;
        deci_num_f = (input - int_num);
        deci_num = deci_num_f * pow(10, digit_max);

	// To check if negative number
        if(input < 0)
        {
          int_num = int_num * (-1);
          deci_num = deci_num * (-1);
          str[0] = '-';
          a = 1;
        }
        // Convert int to int_array and deci_array
        while(int_num)
        {
          int_array[i++] = int_num % 10 + '0';
          int_num = int_num / 10;
        }
        for(i=0; i < digit_max; i++)    // To cater to cases like 0.00XX, else it becomes 0.XX
        {
          deci_array[i] = deci_num % 10 + '0';
          deci_num = deci_num / 10;
        }
        //Transfer inte_array and deci_array to str for return
        for(i=0; i<digit_max; i++)
        {
          if(int_array[(digit_max-1) - i])
          {
            str[a++] = int_array[(digit_max-1) - i];
          }
        }
        str[a++] = '.';
        for(i=0; i<digit_max; i++)
        {
          if(deci_array[(digit_max-1) - i])
          {
            str[a++] = deci_array[(digit_max-1) - i];
          }
        }
        while(a < strlen-1)
        {
          str[a++] = '0';
        }
        str[a] = '\0';

        return 0;
}

