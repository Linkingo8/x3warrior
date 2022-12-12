#include "warrior_hardware/bsp_go1.hpp"
#include <stdio.h>
#include <errno.h>
#include <fcntl.h> /* File control definitions */
#include <unistd.h>
#include <asm/termbits.h>
#include <sys/ioctl.h>
#include <rclcpp/rclcpp.hpp>

using namespace warrior_hardware;

Go1Config::Go1Config()
    : go1_port_(-1)
{
}

int Go1Config::open(const std::string & port_name)
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

int Go1Config::write_frame(const uint8_t* data, size_t size)
{
    ::write(go1_port_,data,size);
    return 1;
}

int Go1Config::read_frames(uint8_t* data, size_t size)
{
    ::read(go1_port_,data,size);
    return 1;
}
Go1DataProcess::Go1DataProcess(uint16_t CRC16_CCITT_INIT) : crc(CRC16_CCITT_INIT)
{ 
}

/**
* @brief Motor state control states 
* @param ControlData_t go1_control_data
* @return 0 sucess 1 fail
*/
void Go1DataProcess::Go1_head_set(void)
{
    for(int i = 0; i < GO1_NUM; i++)
    {
        go1_control_data_[i].tx.data.head[0] = 0XFE;
        go1_control_data_[i].tx.data.head[1] = 0xEE;
        // RCLCPP_INFO(rclcpp::get_logger("Go1_config"), "go[%d] Go1 head1:%x  head2:%x",i ,go1_control_data_[i].tx.tx_buff[0],go1_control_data_[i].tx.tx_buff[1]);
    }
}

void Go1DataProcess::Go1_id_set(void)
{
    for(int i = 0; i < GO1_NUM; i++)
    {
        go1_control_data_[i].tx.data.mode.id = 0x00;
        go1_control_data_[i].tx.data.mode.status = 0x01;
        go1_control_data_[i].tx.data.mode.none = 0x00;
        // RCLCPP_INFO(rclcpp::get_logger("Go1_config"), "go[%d] Go1 messageSendBuffer mode: %x",i ,go1_control_data_[i].tx.tx_buff[3]);       
    }   
    
}

void Go1DataProcess::Go1_speed_set(uint8_t position,double k_sped,double spd_set)
{
    float k_temp = k_sped;
    float target_temp = spd_set;
    go1_control_data_[position].tx.data.comd.spd_set = (256 * target_temp)  / (2 * PI);
    go1_control_data_[position].tx.data.comd.k_spd = (1280 * k_temp);
    RCLCPP_INFO(rclcpp::get_logger("Go1_config"), "go1_control_data_[position].tx.data.comd.k_spd %x",go1_control_data_[position].tx.data.comd.k_spd);    
    //change to little endian
    uint8_t n_right,n_left; //right表示低8位，left表示高8位
    n_right = go1_control_data_[position].tx.data.comd.spd_set &0xFF; //取低8位 n_right =  2 ^8 -1 = 255  
    n_left = (go1_control_data_[position].tx.data.comd.spd_set  >> 8) & 0xff; //取高8位 n_right =  2 ^7 -1 = 127
    go1_control_data_[position].tx.tx_buff[5] =  n_right;
    go1_control_data_[position].tx.tx_buff[6] =  n_left;

    n_right = go1_control_data_[position].tx.data.comd.k_spd &0xFF; //取低8位 n_right =  2 ^8 -1 = 255  
    n_left = (go1_control_data_[position].tx.data.comd.k_spd  >> 8) & 0xff; //取高8位 n_right =  2 ^7 -1 = 127
    go1_control_data_[position].tx.tx_buff[13] =  n_right;
    go1_control_data_[position].tx.tx_buff[14] =  n_left;


    //RCLCPP_INFO(rclcpp::get_logger("Go1_config"), "go[%d] Go1 spd_set:%d",position ,go1_control_data_[position].tx.data.comd.spd_set);    
    // RCLCPP_INFO(rclcpp::get_logger("Go1_config"), "go[%d] Go1 speed buff check:tx_buff[5]: %x tx_buff[6]: %x",position ,go1_control_data_[position].tx.tx_buff[5],go1_control_data_[position].tx.tx_buff[6]);    
    RCLCPP_INFO(rclcpp::get_logger("Go1_config"), "go1_control_data_[position].tx.tx_buff[13] %x",go1_control_data_[position].tx.tx_buff[13]);    
    RCLCPP_INFO(rclcpp::get_logger("Go1_config"), "go1_control_data_[position].tx.tx_buff[14] %x",go1_control_data_[position].tx.tx_buff[14]);        
}

void Go1DataProcess::Go1_crc_append(void)
{
    for(int i = 0; i < GO1_NUM; i++)
    {
        // //calc and assert
        Append_CRC16_Check_Sum((uint8_t *)go1_control_data_[i].tx.tx_buff,17);
        // RCLCPP_INFO(rclcpp::get_logger("Go1_config"), "go[%d] Go1 messageSendBuffer crc[0] %x crc[1] :%x",i ,go1_control_data_[i].tx.tx_buff[15],go1_control_data_[i].tx.tx_buff[16]);       
    }
}
uint8_t* Go1DataProcess::Go1_buff_get(uint8_t index){
    return go1_control_data_[index].tx.tx_buff;
}
void Go1DataProcess::Go1_head_print(void)
{
    /*head check*/
    for(int i = 0; i < GO1_NUM; i++)
    {
        // RCLCPP_INFO(rclcpp::get_logger("Go1_config"), "go[%d] Go1 head1:%x  head2:%x",i ,go1_control_data_[i].tx.tx_buff[0],go1_control_data_[i].tx.tx_buff[1]);
    }
}

void Go1DataProcess::Go1_buff_print(void)
{
    /*head check*/
    for(int i = 0; i < 4; i++)
    {
        for (size_t j = 0; j < 17; j++)
        {
         RCLCPP_INFO(rclcpp::get_logger("Go1_config"), "buff[%d]:%x",j,go1_control_data_[i].tx.tx_buff[j]);
        }
        // RCLCPP_INFO(rclcpp::get_logger("Go1_config"), "go[%d] Go1 head1:%x  head2:%x",i ,go1_control_data_[i].tx.tx_buff[0],go1_control_data_[i].tx.tx_buff[1]);
    }
}
