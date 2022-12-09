#include "warrior_hardware/crc.hpp"
using namespace warrior_hardware;
crc::crc()
{

}
uint16_t crc::CRC16_CCITT(uint8_t* bytes,int dwLength)
{
        uint16_t wCRC = 0X0000;
        int polynomial = 0x8408;// poly value reversed 0x1021;
        int i, j;
        for (i = 0; i < dwLength; i++) {
            wCRC ^= ((int) bytes[i] & 0x000000ff);
            for (j = 0; j < 8; j++) {
                if ((wCRC & 0x00000001) != 0) {
                    wCRC >>= 1;
                    wCRC ^= polynomial;
                } else {
                    wCRC >>= 1;
                }
            }
        }
        return wCRC;    
}
void crc::Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength)
{
		uint16_t wCRC = 0;

		if ((pchMessage == NULL) || (dwLength <= 2))
		{
				return;
		}

		wCRC = CRC16_CCITT ( (uint8_t *)pchMessage, dwLength-2);

		pchMessage[dwLength-2] = (uint8_t)(wCRC & 0x00ff);
		pchMessage[dwLength-1] = (uint8_t)((wCRC >> 8)& 0x00ff);    
}