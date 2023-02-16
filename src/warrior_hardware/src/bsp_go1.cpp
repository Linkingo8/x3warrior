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

Go1DataProcess::Go1DataProcess(uint16_t CRC16_CCITT_INIT) 
: crc(CRC16_CCITT_INIT)
, tramsmit_status_(0)
, id_temp_(0)
{}
/**
* @brief Motor state control states 
* @param ControlData_t go1_control_data
* @return 0 sucess 1 fail
*/

/*recieve*/
double Go1DataProcess::Go1_velocities_export(uint8_t id_temp)
{   
    double velocities = go1_export_data_[id_temp].velocity;
    return velocities; /* rad / s */
}

double Go1DataProcess::Go1_positions_export(uint8_t id_temp)
{ 
    double position = go1_export_data_[id_temp].position;
    return position;/* rad */
}

double Go1DataProcess::Go1_torques_export(uint8_t id_temp)
{ 
    double torque = double(go1_feedback_data_[id_temp].fbk.torque / 256.0);
    return torque;/*N * m*/
}

void Go1DataProcess::Go1_data_rec(uint8_t id,uint8_t *buff_temp)
{
    // buff_temp[0] = 0xFD;
    // buff_temp[1] = 0xEE;

    // buff_temp[2] = 0x10;

    // buff_temp[3] = 0xFF;
    // buff_temp[4] = 0x0;

    // buff_temp[5] = 0xFF;
    // buff_temp[6] = 0x0;

    // buff_temp[7] = 0xFF;
    // buff_temp[8] = 0x0;
    // buff_temp[9] = 0x0;
    // buff_temp[10] = 0x0;
    
    // buff_temp[11] = 0xFF;

    // buff_temp[12] = 0x52;
    // buff_temp[13] = 0x48;

    // buff_temp[14] = 0xEB;
    // buff_temp[15] = 0x15;


    /*init to 0*/
    for (size_t i = 0; i < GO1_NUM; i++)
    {
        go1_feedback_data_[i].head[0] = 0;
        go1_feedback_data_[i].head[1] = 0;
        go1_feedback_data_[i].mode.id = -1;
        go1_feedback_data_[i].mode.status = 0;
        go1_feedback_data_[i].mode.none = 0;
        go1_feedback_data_[i].fbk.force = 0;
        go1_feedback_data_[i].fbk.MError = 0;
        go1_feedback_data_[i].fbk.none = 0;
        go1_feedback_data_[i].fbk.pos = 0;
        go1_feedback_data_[i].fbk.speed = 0;
        go1_feedback_data_[i].fbk.temp = 0;
        go1_feedback_data_[i].fbk.torque = 0;
        go1_feedback_data_[i].CRC16 = 0;
    }
    
    if(buff_temp[0] == 0xFD && buff_temp[1]==0xEE)
    {
        uint8_t id_temp = (buff_temp[2] & 0xF);
        //analysis id
        if(id == id_temp)
        {
            if((id_temp > GO1_NUM-1) || id_temp < 0)
            {
                return;
            }
            else
            {
                // RCLCPP_INFO(rclcpp::get_logger("Go1_config"), "id[%d]  ",id_temp);

                go1_feedback_data_[id_temp].head[0] = 0xFD;
                go1_feedback_data_[id_temp].head[1] = 0xEE;
                go1_feedback_data_[id_temp].mode.id = buff_temp[2] & 0xF;
                go1_feedback_data_[id_temp].mode.status = (buff_temp[2] & 0x70) >> 4;
                go1_feedback_data_[id_temp].mode.none = 0;
                //torque //merge to big-endian
                go1_feedback_data_[id_temp].fbk.torque = buff_temp[4];
                go1_feedback_data_[id_temp].fbk.torque = go1_feedback_data_[id_temp].fbk.torque << 8;
                go1_feedback_data_[id_temp].fbk.torque |= buff_temp[3];
                // RCLCPP_INFO(rclcpp::get_logger("Go1_config"), "go1_feedback_data_[%d] torque: %d",id_temp,go1_feedback_data_[go1_feedback_data_[id_temp].mode.id].fbk.torque);
                //speed //merge to big-endian
                go1_feedback_data_[id_temp].fbk.speed = buff_temp[6];
                go1_feedback_data_[id_temp].fbk.speed = go1_feedback_data_[id_temp].fbk.speed << 8;
                go1_feedback_data_[id_temp].fbk.speed |= buff_temp[5];
                // RCLCPP_INFO(rclcpp::get_logger("Go1_config"), "speed: %d",go1_feedback_data_[id_temp].fbk.speed);
                //position //merge to big-endian
                uint32_t merge_to_32[4]{0};
                merge_to_32[0] = buff_temp[10];
                merge_to_32[0] = merge_to_32[0] << 24;
                merge_to_32[1] = buff_temp[9];
                merge_to_32[1] = merge_to_32[1] << 16;
                merge_to_32[2] = buff_temp[8];
                merge_to_32[2] = merge_to_32[2] << 8;
                merge_to_32[3] = buff_temp[7];
                merge_to_32[3] = merge_to_32[3];
                go1_feedback_data_[id_temp].fbk.pos |= merge_to_32[0];
                go1_feedback_data_[id_temp].fbk.pos |= merge_to_32[1];
                go1_feedback_data_[id_temp].fbk.pos |= merge_to_32[2];
                go1_feedback_data_[id_temp].fbk.pos |= merge_to_32[3];
                // if(id_temp == 2)
                //     RCLCPP_INFO(rclcpp::get_logger("Go1_config"), "pos:%d",go1_feedback_data_[id_temp].fbk.pos);
                // RCLCPP_INFO(rclcpp::get_logger("Go1_config"), "pos: %d",go1_feedback_data_[id_temp].fbk.pos);
                //temperature
                go1_feedback_data_[id_temp].fbk.temp = buff_temp[11];
                /*bit fields*/
                //MError
                go1_feedback_data_[id_temp].fbk.MError = (buff_temp[12] & 0xE0) >> 5;
                // RCLCPP_INFO(rclcpp::get_logger("Go1_config"), "pos:%x",go1_feedback_data_[id_temp].fbk.MError);
                // RCLCPP_INFO(rclcpp::get_logger("Go1_config"), "pos:%x",go1_feedback_data_[id_temp].fbk.MError);
                //fource
                uint16_t bit12_temp[2]{0x0000};
                bit12_temp[0] |= (((buff_temp[12] & 0x1F) << 3) | ((buff_temp[13]&0xE0) >> 5));
                // RCLCPP_INFO(rclcpp::get_logger("Go1_config"), "bit12_temp[0]: %x",bit12_temp[0]);
                bit12_temp[1] = (buff_temp[13] & 0x1E) >> 1;
                // RCLCPP_INFO(rclcpp::get_logger("Go1_config"), "bit12_temp[1]: %x",bit12_temp[1]);
                bit12_temp[0] = bit12_temp[0] << 4;
                // RCLCPP_INFO(rclcpp::get_logger("Go1_config"), "bit12_temp[0] = bit12_temp[0] << 8: %x\n",bit12_temp[0]);
                bit12_temp[1] = (bit12_temp[0] | bit12_temp[1]);
                // RCLCPP_INFO(rclcpp::get_logger("Go1_config"), "bit12_temp[1]: %x",bit12_temp[1]);
                go1_feedback_data_[id_temp].fbk.force = bit12_temp[1];
                go1_feedback_data_[id_temp].fbk.none = 0;
                //crc //merge to big-endian
                go1_feedback_data_[id_temp].CRC16 = buff_temp[15];
                go1_feedback_data_[id_temp].CRC16 = go1_feedback_data_[id_temp].CRC16 << 8;
                go1_feedback_data_[id_temp].CRC16 |= buff_temp[14];
                // RCLCPP_INFO(rclcpp::get_logger("Go1_config"), "CRC: %x",go1_feedback_data_[id_temp].CRC16);

                go1_export_data_[id_temp].velocity = float(go1_feedback_data_[id_temp].fbk.speed /256.00) * float(2 * PI);
                go1_export_data_[id_temp].position = float(go1_feedback_data_[id_temp].fbk.pos /32768.00) * float(2 * PI);
                go1_export_data_[id_temp].torque = (go1_feedback_data_[id_temp].fbk.torque /256.00);
                // RCLCPP_INFO(rclcpp::get_logger("Go1_config"), "go1_export_data_[%d].velocity: %f",id_temp,go1_export_data_[id_temp].velocity);
                // RCLCPP_INFO(rclcpp::get_logger("Go1_config"), "go1_export_data_[%d].position: %f",id_temp,go1_export_data_[id_temp].position);
                // RCLCPP_INFO(rclcpp::get_logger("Go1_config"), "go1_export_data_[%d].torque: %.3f",id_temp,go1_export_data_[id_temp].torque);
            }
        }
    }
    else
    {
        for (size_t i = 0; i < GO1_NUM; i++)
        {
            go1_feedback_data_[i].head[0] = 0;
            go1_feedback_data_[i].head[1] = 0;
            go1_feedback_data_[i].mode.id = 0;
            go1_feedback_data_[i].mode.status = 0;
            go1_feedback_data_[i].mode.none = 0;
            go1_feedback_data_[i].fbk.force = 0;
            go1_feedback_data_[i].fbk.MError = 0;
            go1_feedback_data_[i].fbk.none = 0;
            go1_feedback_data_[i].fbk.pos = 0;
            go1_feedback_data_[i].fbk.speed = 0;
            go1_feedback_data_[i].fbk.temp = 0;
            go1_feedback_data_[i].fbk.torque = 0;
            go1_feedback_data_[i].CRC16 = 0;
        }
    }
}
/*send*/
void Go1DataProcess::Go1_head_set(uint8_t id)
{
        go1_control_data_[id].tx.data.head[0] = 0XFE;
        go1_control_data_[id].tx.data.head[1] = 0xEE;
}

void Go1DataProcess::Go1_id_set(uint8_t id)
{
        go1_control_data_[id].tx.data.mode.id = id;
        go1_control_data_[id].tx.data.mode.status = 0x01;
        go1_control_data_[id].tx.data.mode.none = 0x00;  
}

void Go1DataProcess::Go1_speed_set(uint8_t index,double k_sped,double spd_set)
{
    if(index > 3 || index == 0) {
        RCLCPP_INFO(rclcpp::get_logger("Go1_config"), "Go1_speed_set id error!");    
        return;
    }
    float k_temp = k_sped;
    float target_temp = spd_set;

    int16_t spd_set_to_buffer = (256 * target_temp)  / (2 * PI);
    uint16_t  k_spd_to_buffer = (1280 * k_temp);
    //change to little endian
    uint8_t n_right,n_left; //right表示低8位，left表示高8位
    n_right = spd_set_to_buffer &0xFF; //取低8位 n_right =  2 ^8 -1 = 255  
    n_left = (spd_set_to_buffer  >> 8) & 0xff; //取高8位 n_right =  2 ^7 -1 = 127
    go1_control_data_[index].tx.tx_buff[5] =  n_right;
    go1_control_data_[index].tx.tx_buff[6] =  n_left;

    n_right = k_spd_to_buffer &0xFF; //取低8位 n_right =  2 ^8 -1 = 255  
    n_left = (k_spd_to_buffer  >> 8) & 0xff; //取高8位 n_right =  2 ^7 -1 = 127
    go1_control_data_[index].tx.tx_buff[13] =  n_right;
    go1_control_data_[index].tx.tx_buff[14] =  n_left;    

}

void Go1DataProcess::Go1_speed_set(uint8_t index,double spd_set)
{
    float target_temp = spd_set;
    go1_control_data_[index].tx.data.comd.spd_set = (256 * target_temp)  / (2 * PI);
    RCLCPP_INFO(rclcpp::get_logger("Go1_config"), "go1_control_data_[position].tx.data.comd.k_spd %x",go1_control_data_[index].tx.data.comd.k_spd);    
    //change to little endian
    uint8_t n_right,n_left; //right表示低8位，left表示高8位
    n_right = go1_control_data_[index].tx.data.comd.spd_set &0xFF; //取低8位 n_right =  2 ^8 -1 = 255  
    n_left = (go1_control_data_[index].tx.data.comd.spd_set  >> 8) & 0xff; //取高8位 n_right =  2 ^7 -1 = 127
    go1_control_data_[index].tx.tx_buff[5] =  n_right;
    go1_control_data_[index].tx.tx_buff[6] =  n_left;   
}

void Go1DataProcess::Go1_zero_torque_set(uint8_t index)
{
    go1_control_data_[index].tx.data.comd.k_pos = 0x00;
    go1_control_data_[index].tx.data.comd.k_spd = 0x00;
    go1_control_data_[index].tx.data.comd.tor_set = 0x00;
    go1_control_data_[index].tx.data.comd.pos_set = 0x00;
    go1_control_data_[index].tx.data.comd.spd_set = 0x00;
    for(int i = 0;i < 17;i++)
        RCLCPP_INFO(rclcpp::get_logger("Go1_config"), "go1_control_data_[%d].tx.tx_buff[%d] %x",index,i,go1_control_data_[index].tx.tx_buff[i]); 
}

void Go1DataProcess::Go1_torque_set(uint8_t index,double tor_set)
{
    if(index > 3) {
        RCLCPP_INFO(rclcpp::get_logger("Go1_config"), "Go1_torque_set id error!");    
        return;
    }
    float target_temp = tor_set;
    int16_t torset_to_buffer = (256 * target_temp); 
    //change to little endian
    uint8_t n_right,n_left; //right表示低8位，left表示高8位
    n_right = torset_to_buffer &0xFF; //取低8位 n_right =  2 ^8 -1 = 255  
    n_left  = (torset_to_buffer >>8) & 0xff; //取高8位 n_right =  2 ^7 -1 = 127
    go1_control_data_[index].tx.tx_buff[3] =  n_right;
    go1_control_data_[index].tx.tx_buff[4] =  n_left;
}

void Go1DataProcess::Go1_position_set(uint8_t index,double k_pos,double pos_set)
{
    float k_temp = k_pos;
    float target_temp = pos_set;
    go1_control_data_[index].tx.data.comd.pos_set = (32768 * pos_set) / (PI*2);
    go1_control_data_[index].tx.data.comd.k_pos = (1280 * k_temp);
    // RCLCPP_INFO(rclcpp::get_logger("Go1_config"), "pos: %d",go1_control_data_[index].tx.data.comd.pos_set);
    //change to little endian
    uint8_t n_buffer[4]{0}; //0 - 3 依次位大端模式排序
    n_buffer[0]=(go1_control_data_[index].tx.data.comd.pos_set>>24)&0XFF;
    n_buffer[1]=(go1_control_data_[index].tx.data.comd.pos_set>>16)&0XFFFF;
    n_buffer[2]=(go1_control_data_[index].tx.data.comd.pos_set>>8)&0XFFFFFF;
    n_buffer[3]= go1_control_data_[index].tx.data.comd.pos_set&0XFF;

    go1_control_data_[index].tx.tx_buff[7] =  n_buffer[3];
    go1_control_data_[index].tx.tx_buff[8] =  n_buffer[2];
    go1_control_data_[index].tx.tx_buff[9] =  n_buffer[1];
    go1_control_data_[index].tx.tx_buff[10] = n_buffer[0];

    uint8_t n_right,n_left; //right表示低8位，left表示高8位
    n_right = go1_control_data_[index].tx.data.comd.k_pos &0xFF; //取低8位 n_right =  2 ^8 -1 = 255  
    n_left = (go1_control_data_[index].tx.data.comd.k_pos  >> 8) & 0xff; //取高8位 n_right =  2 ^7 -1 = 127
    go1_control_data_[index].tx.tx_buff[11] =  n_right;
    go1_control_data_[index].tx.tx_buff[12] =  n_left;        
}

void Go1DataProcess::Go1_crc_append(uint8_t id)
{
    Append_CRC16_Check_Sum((uint8_t *)go1_control_data_[id].tx.tx_buff,17);     
}

uint8_t* Go1DataProcess::Go1_buff_get(uint8_t index){
    return go1_control_data_[index].tx.tx_buff;
}

void Go1DataProcess::Go1_head_print(void)
{
    /*head check*/
    for(int i = 0; i < GO1_NUM; i++)
    {
        RCLCPP_INFO(rclcpp::get_logger("Go1_config"), "go[%d] Go1 head1:%x  head2:%x",i ,go1_control_data_[i].tx.tx_buff[0],go1_control_data_[i].tx.tx_buff[1]);
    }
}

void Go1DataProcess::Go1_buff_print(void)
{
    /*head check*/
    for(int i = 0; i < 4; i++)
    {
        RCLCPP_INFO(rclcpp::get_logger("Go1_config"), "go[%d]",i);
        for (size_t j = 0; j < 17; j++)
        {
         RCLCPP_INFO(rclcpp::get_logger("Go1_config"), "buff[%d]:%x",j,go1_control_data_[i].tx.tx_buff[j]);
        }
    }
}

void Go1DataProcess::Go1_buff_zero(void)
{
    /*head check*/
    for(int i = 0; i < 4; i++)
    {
        memset(go1_control_data_[i].tx.tx_buff,0,17);
    }
}

void Go1DataProcess::give_id_to_go1_processor(void)
{
    // id_temp_ = 1;
    id_temp_++;
    if(id_temp_> 3) id_temp_ = 0;
    RCLCPP_INFO(rclcpp::get_logger("Go1_config"), "id_temp_ %d",id_temp_);
    // std::cout << id_temp_ << std::endl;

}