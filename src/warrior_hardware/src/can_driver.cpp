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

}