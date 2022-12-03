#define termios asmtermios
#include <asm/ioctls.h>
#include <asm/termbits.h>
#undef termios

#include <sys/ioctl.h>
#include <termios.h>

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>

#include "warrior_hardware/can_driver.hpp"
namespace warrior_hardware{
    CanCommon::CanCommon()
        :count_(0)
        ,num_(0)
        ,reclen_(0)
    {
    }
    // MecanumbotSerialPort::MecanumbotSerialPort()
    //     : serial_port_(-1)
    //     , rx_frame_length_(0)
    //     , rx_frame_crc_(HDLC_CRC_INIT_VALUE)
    //     , rx_frame_escape_(false)
    //     , tx_frame_length_(0)
    //     , tx_frame_crc_(HDLC_CRC_INIT_VALUE)
    // {

    // }

    // MecanumbotSerialPort::~MecanumbotSerialPort()
    // {
    //     close();
    // }
    void CanCommon::decode_imu_byte(uint8_t imu_data)
    {
        if(imu_data>0) imu_data = 0;
    }


}