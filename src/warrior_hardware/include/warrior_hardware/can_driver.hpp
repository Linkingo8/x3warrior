#ifndef WARRIOR_HARDWARE__CAN_COMMON_INTERFACE_HPP_
#define WARRIOR_HARDWARE__CAN_COMMON_INTERFACE_HPP_
#include <cstdint>
////文件版本：v2.02 20190609
//接口卡类型定义

#define VCI_USBCAN1             3
#define VCI_USBCAN2             4
#define VCI_USBCAN2A            4

#define VCI_USBCAN_E_U          20
#define VCI_USBCAN_2E_U         21

//函数调用返回状态值
#define STATUS_OK                                       1
#define STATUS_ERR                                      0

#define USHORT unsigned short int
#define BYTE unsigned char
#define CHAR char
#define UCHAR unsigned char
#define UINT unsigned int
#define DWORD unsigned int
#define PVOID void*
#define ULONG unsigned int
#define INT int
#define UINT32 UINT
#define LPVOID void*
#define BOOL BYTE
#define TRUE 1
#define FALSE 0
//can imu 
#define IMU_QUAT_ID  0x401
#define IMU_GYRO_ID  0x402
#define IMU_ACCEL_ID 0x403
#define IMU_MAG_ID   0x404
#define IMU_PARAM_ID 0x405

//转换成 m/s^2
#define ACCEL_3G_SEN 0.0008974358974f
#define ACCEL_6G_SEN 0.00179443359375f
#define ACCEL_12G_SEN 0.0035888671875f
#define ACCEL_24G_SEN 0.007177734375f
//转换成 rad/s
#define GYRO_2000_SEN 0.00106526443603169529841533860381f
#define GYRO_1000_SEN 0.00053263221801584764920766930190693f
#define GYRO_500_SEN 0.00026631610900792382460383465095346f
#define GYRO_250_SEN 0.00013315805450396191230191732547673f
#define GYRO_125_SEN 0.000066579027251980956150958662738366f

//1.ZLGCAN系列接口卡信息的数据类型。
typedef  struct  _VCI_BOARD_INFO{
                USHORT  hw_Version;
                USHORT  fw_Version;
                USHORT  dr_Version;
                USHORT  in_Version;
                USHORT  irq_Num;
                BYTE    can_Num;
                CHAR    str_Serial_Num[20];
                CHAR    str_hw_Type[40];
                USHORT  Reserved[4];
} VCI_BOARD_INFO,*PVCI_BOARD_INFO;

//2.定义CAN信息帧的数据类型。
typedef  struct  _VCI_CAN_OBJ{
        UINT    ID;
        UINT    TimeStamp;
        BYTE    TimeFlag;
        BYTE    SendType;
        BYTE    RemoteFlag;//是否是远程帧
        BYTE    ExternFlag;//是否是扩展帧
        BYTE    DataLen;
        BYTE    Data[8];
        BYTE    Reserved[3];
}VCI_CAN_OBJ,*PVCI_CAN_OBJ;

//3.定义初始化CAN的数据类型
typedef struct _INIT_CONFIG{
        DWORD   AccCode;
        DWORD   AccMask;
        DWORD   Reserved;
        UCHAR   Filter;
        UCHAR   Timing0;
        UCHAR   Timing1;
        UCHAR   Mode;
}VCI_INIT_CONFIG,*PVCI_INIT_CONFIG;

///////// new add struct for filter /////////
typedef struct _VCI_FILTER_RECORD{
        DWORD ExtFrame; //是否为扩展帧
        DWORD Start;
        DWORD End;
}VCI_FILTER_RECORD,*PVCI_FILTER_RECORD;

#ifdef __cplusplus
#define EXTERN_C  extern "C"
#else
#define EXTERN_C
#endif

EXTERN_C DWORD VCI_OpenDevice(DWORD DeviceType,DWORD DeviceInd,DWORD Reserved);
EXTERN_C DWORD VCI_CloseDevice(DWORD DeviceType,DWORD DeviceInd);
EXTERN_C DWORD VCI_InitCAN(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd, PVCI_INIT_CONFIG pInitConfig);

EXTERN_C DWORD VCI_ReadBoardInfo(DWORD DeviceType,DWORD DeviceInd,PVCI_BOARD_INFO pInfo);

EXTERN_C DWORD VCI_SetReference(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd,DWORD RefType,PVOID pData);

EXTERN_C ULONG VCI_GetReceiveNum(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd);
EXTERN_C DWORD VCI_ClearBuffer(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd);

EXTERN_C DWORD VCI_StartCAN(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd);
EXTERN_C DWORD VCI_ResetCAN(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd);

EXTERN_C ULONG VCI_Transmit(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd,PVCI_CAN_OBJ pSend,UINT Len);
EXTERN_C ULONG VCI_Receive(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd,PVCI_CAN_OBJ pReceive,UINT Len,INT WaitTime);

EXTERN_C DWORD  VCI_UsbDeviceReset(DWORD DevType,DWORD DevIndex,DWORD Reserved);
EXTERN_C DWORD  VCI_FindUsbDevice2(PVCI_BOARD_INFO pInfo);

namespace warrior_hardware
{
  class CanCommon
    {
        public:
            CanCommon();
            ~CanCommon();
            
        protected:
            void decode_imu_byte(uint8_t imu_data);

        private:
                VCI_BOARD_INFO pInfo_;
                int count_;
                VCI_BOARD_INFO pInfo1_[50];
                int num_;
                int reclen_;
                VCI_CAN_OBJ rec_[3000];//接收缓存，设为3000为佳。
    };
}
#endif