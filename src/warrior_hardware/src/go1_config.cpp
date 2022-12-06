#include "warrior_hardware/go1_config.hpp"
#include <stdio.h>
#include <errno.h>
#include <fcntl.h> /* File control definitions */
#include <unistd.h>
#include <asm/termbits.h>
#include <sys/ioctl.h>
#include <rclcpp/rclcpp.hpp>

using namespace warrior_hardware;

go1_config::go1_config()
    : go1_port_(-1)
{
}

int go1_config::open(const std::string & port_name)
{
    go1_port_ = ::open(port_name.c_str(), O_RDWR);

    if (go1_port_ < 0) {
        fprintf(stderr, "Failed to open serial port: %s (%d)\n", strerror(errno), errno);
        return 1;
    }

    struct termios2 option = {0};
    ioctl(go1_port_, TCGETS2, &option);

    option.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG | ECHONL);

    option.c_iflag &= ~(IXON | IXOFF | IXANY);   // Turn off s/w flow ctrl
    option.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);  // Disable any special handling of received bytes
    option.c_oflag &= ~(OPOST); // Prevent special interpretation of output bytes
    option.c_oflag &= ~(ONLCR); // Prevent conversion of newline to carriage return/line fee 
    
    option.c_cflag &= ~PARENB;          // Clear parity bit, disabling parity 无校验位
    option.c_cflag &= ~CSTOPB;          // only one stop bit used in communication
    option.c_cflag &= ~CRTSCTS;         // Disable RTS/CTS hardware flow control 
    option.c_cflag &= ~CSIZE;           // Clear all the size bits,
    option.c_cflag |= CS8;              // 8 bits per byte
    option.c_cflag |= (CLOCAL | CREAD); // Turn on READ & ignore ctrl lines

    option.c_cc[VMIN]  = (cc_t)(20/100);   //20ms  单位转换为 0.1s 
    option.c_cc[VTIME] = 0;

    // Custom Baud Rates
    option.c_cflag &= ~CBAUD;
    option.c_cflag |= BOTHER;
    option.c_ispeed = 4000000;
    option.c_ospeed = 4000000;  

    ioctl(go1_port_, TCSETS2, &option);

    return 0;
}

int go1_config::write_frame(const uint8_t* data, size_t size)
{
    ::write(go1_port_,data,size);
    return 1;
}