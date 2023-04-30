#include "warrior_hardware/bsp_LK_MF9025.hpp"
#include <cmath>
#include <stdio.h>
#include <rclcpp/rclcpp.hpp>

using namespace warrior_hardware;

void MF9025DataProcess::MF9025_speed_set(uint8_t id_shift, double speed)
{
    int32_t speed_integer{0};
    speed *=100;
    //Round up
    speed_integer = ceil(speed);
//    RCLCPP_FATAL(rclcpp::get_logger("MF9025_speed_set"),"speed_integer %d", speed_integer);}
    send_9025_[id_shift - 1].ID=0x140 + id_shift;
    send_9025_[id_shift - 1].SendType=0;
    send_9025_[id_shift - 1].RemoteFlag=0;
    send_9025_[id_shift - 1].ExternFlag=0;
    send_9025_[id_shift - 1].DataLen=8;
    send_9025_[id_shift - 1].Data[0] = 0xA2;
    send_9025_[id_shift - 1].Data[1] = 0x00;
    send_9025_[id_shift - 1].Data[2] = 0x00;
    send_9025_[id_shift - 1].Data[3] = 0x00;

    send_9025_[id_shift - 1].Data[4] = *(uint8_t *)(&speed_integer);
    send_9025_[id_shift - 1].Data[5] = *((uint8_t *)(&speed_integer)+1);
    send_9025_[id_shift - 1].Data[6] = *((uint8_t *)(&speed_integer)+2);
    send_9025_[id_shift - 1].Data[7] = *((uint8_t *)(&speed_integer)+3);
    // RCLCPP_FATAL(rclcpp::get_logger("MF9025_speed_set"),"id: %x", send_9025_[0].ID);
}

void MF9025DataProcess::MF9025_torque_set(uint8_t id_shift, double torque)
{
    int32_t torque_integer{0};
    torque *=100;
    //Round up
    torque_integer = ceil(torque);
//    RCLCPP_FATAL(rclcpp::get_logger("MF9025_torque_set"),"torque_integer %d", torque_integer);}
    send_9025_[id_shift - 1].ID=0x140 + id_shift;
    send_9025_[id_shift - 1].SendType=0;
    send_9025_[id_shift - 1].RemoteFlag=0;
    send_9025_[id_shift - 1].ExternFlag=0;
    send_9025_[id_shift - 1].DataLen=8;
    send_9025_[id_shift - 1].Data[0] = 0xA1;
    send_9025_[id_shift - 1].Data[1] = 0x00;
    send_9025_[id_shift - 1].Data[2] = 0x00;
    send_9025_[id_shift - 1].Data[3] = 0x00;

    send_9025_[id_shift - 1].Data[4] = *(uint8_t *)(&torque_integer);
    send_9025_[id_shift - 1].Data[5] = *((uint8_t *)(&torque_integer)+1);
    send_9025_[id_shift - 1].Data[6] = 0x00;
    send_9025_[id_shift - 1].Data[7] = 0x00;
    // RCLCPP_FATAL(rclcpp::get_logger("MF9025_torque_set"),"id: %x", send_9025_[id_shift - 1].ID);
}

void MF9025DataProcess::MF9025_position_set(uint8_t id_shift, double position)
{
    int32_t position_integer{0};
    position *=100;
    //Round up
    position_integer = ceil(position);
    send_9025_[id_shift - 1].ID=0x140 + id_shift;
    send_9025_[id_shift - 1].SendType=0;
    send_9025_[id_shift - 1].RemoteFlag=0;
    send_9025_[id_shift - 1].ExternFlag=0;
    send_9025_[id_shift - 1].DataLen=8;
    send_9025_[id_shift - 1].Data[0] = 0xA3;
    send_9025_[id_shift - 1].Data[1] = 0x00;
    send_9025_[id_shift - 1].Data[2] = 0x00;
    send_9025_[id_shift - 1].Data[3] = 0x00;

    send_9025_[id_shift - 1].Data[4] = *(uint8_t *)(&position_integer);
    send_9025_[id_shift - 1].Data[5] = *((uint8_t *)(&position_integer)+1);
    send_9025_[id_shift - 1].Data[6] = *((uint8_t *)(&position_integer)+2);
    send_9025_[id_shift - 1].Data[7] = *((uint8_t *)(&position_integer)+3);
    RCLCPP_FATAL(rclcpp::get_logger("MF9025_position_set"),"id: %x", send_9025_[0].ID);
}

void MF9025DataProcess::MF9025_commond_send(uint16_t id)
{
 //left
    switch (id)
    {
    case RIGHT_ID:
    if(VCI_Transmit(VCI_USBCAN2, 0, 0, send_9025_, 1) == 1)
    {
    }
        break;
    case LEFT_ID:
 //right
    if(VCI_Transmit(VCI_USBCAN2, 0, 0, send_9025_ + 1, 1) == 1)
    {
    }
        break;    
    }
}

void MF9025DataProcess::MF9025_message_rec(VCI_CAN_OBJ *rec_,int16_t q1) 
{
    // RCLCPP_FATAL(rclcpp::get_logger("MF9025_speed_rec"),"speed 0: %d", rec_9025_[0].speed);
    // RCLCPP_FATAL(rclcpp::get_logger("MF9025_speed_rec"),"speed 1: %d", rec_9025_[1].speed);

    // RCLCPP_FATAL(rclcpp::get_logger("MF9025_temperature_rec"),"temperature 0: %d", rec_9025_[0].temperature);
    // RCLCPP_FATAL(rclcpp::get_logger("MF9025_temperature_rec"),"temperature 1: %d", rec_9025_[1].temperature);
    switch (rec_[q1].ID)
    {
    case RIGHT_ID:
        rec_9025_[0].commond = (uint8_t)rec_[q1].Data[0];

        rec_9025_[0].temperature = (int8_t)rec_[q1].Data[1];//DATA[1] = *(uint8_t *)(&temperature)
        
        rec_9025_[0].iq = (int16_t)rec_[q1].Data[2];//DATA[2] = *(uint8_t *)(&iq) 
        rec_9025_[0].iq = rec_9025_[0].iq | ((int16_t)rec_[q1].Data[3] << 8);//DATA[3] = *((uint8_t *)(&iq)+1) 

        rec_9025_[0].speed = (int16_t)rec_[q1].Data[4];//DATA[2] = *(uint8_t *)(&iq) 
        rec_9025_[0].speed = rec_9025_[0].speed | ((int16_t)rec_[q1].Data[5] << 8);//DATA[3] = *((uint8_t *)(&iq)+1) 
        


        rec_9025_[0].encoder = (uint16_t)rec_[q1].Data[6];//DATA[2] = *(uint8_t *)(&iq) 
        rec_9025_[0].encoder = rec_9025_[0].encoder | ((uint16_t)rec_[q1].Data[7] << 8);//DATA[3] = *((uint8_t *)(&iq)+1) 
        // RCLCPP_FATAL(rclcpp::get_logger("MF9025_position_set"),"speed: %f", rec_9025_[0].speed);
        break;
    case LEFT_ID:
        rec_9025_[1].commond = (uint8_t)rec_[q1].Data[0];

        rec_9025_[1].temperature = (int8_t)rec_[q1].Data[1];//DATA[1] = *(uint8_t *)(&temperature)
        
        rec_9025_[1].iq = (int16_t)rec_[q1].Data[2];//DATA[2] = *(uint8_t *)(&iq) 
        rec_9025_[1].iq = rec_9025_[1].iq | ((int16_t)rec_[q1].Data[3] << 8);//DATA[3] = *((uint8_t *)(&iq)+1) 

        rec_9025_[1].speed = (int16_t)rec_[q1].Data[4];//DATA[2] = *(uint8_t *)(&iq) 
        rec_9025_[1].speed = rec_9025_[1].speed | ((int16_t)rec_[q1].Data[5] << 8);//DATA[3] = *((uint8_t *)(&iq)+1) 
        // RCLCPP_FATAL(rclcpp::get_logger("MF9025_position_set"),"id: %d", rec_9025_[1].speed);


        rec_9025_[1].encoder = (uint16_t)rec_[q1].Data[6];//DATA[2] = *(uint8_t *)(&iq) 
        rec_9025_[1].encoder = rec_9025_[1].encoder | ((uint16_t)rec_[q1].Data[7] << 8);//DATA[3] = *((uint8_t *)(&iq)+1) 
        break;    
    default:
        break;
    }

}

double MF9025DataProcess::MF9025_velocitise_export(VCI_CAN_OBJ *rec_,uint8_t id)
{
    uint16_t id_temp = 0x140 + id;
    switch (id_temp)
    {
    case RIGHT_ID:
        if(rec_9025_[0].speed!=0)
            return -rec_9025_[0].speed;
        else
            return 0;
        break;
    case LEFT_ID:
        return rec_9025_[1].speed;
        break;    
    default:
        return 0;
        break;
    }    
}

double MF9025DataProcess::MF9025_position_export(VCI_CAN_OBJ *rec_,uint8_t id)
{
    uint16_t id_temp = 0x140 + id;
    switch (id_temp)
    {
    case RIGHT_ID:
        return rec_9025_[0].encoder;
        break;
    case LEFT_ID:
        return rec_9025_[1].encoder;
        break;    
    default:
        return 0;
        break;
    }    
}

double MF9025DataProcess::MF9025_torque_export(VCI_CAN_OBJ *rec_,uint8_t id)
{
    uint16_t id_temp = 0x140 + id;
    switch (id_temp)
    {
    case RIGHT_ID:
        return -rec_9025_[0].iq;
        break;
    case LEFT_ID:
        return rec_9025_[1].iq;
        break;    
    default:
        return 0;
        break;
    }    
}