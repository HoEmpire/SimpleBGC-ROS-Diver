#include <iostream>
#include "stdio.h"
using namespace std;

void crc16_update(uint16_t length, uint8_t *data, uint8_t crc[2])
{
    uint16_t counter;
    uint16_t polynom = 0x8005;
    uint16_t crc_register = (uint16_t)crc[0] | ((uint16_t)crc[1] << 8);
    uint8_t shift_register;
    uint8_t data_bit, crc_bit;
    for (counter = 0; counter < length; counter++)
    {
        for (shift_register = 0x01; shift_register > 0x00; shift_register <<= 1)
        {
            data_bit = (data[counter] & shift_register) ? 1 : 0;
            crc_bit = crc_register >> 15;
            crc_register <<= 1;
            if (data_bit != crc_bit)
                crc_register ^= polynom;
        }
    }
    crc[0] = crc_register;
    crc[1] = (crc_register >> 8);
}

void crc16_calculate(uint16_t length, uint8_t *data, uint8_t crc[2])
{
    crc[0] = 0;
    crc[1] = 0;
    crc16_update(length, data, crc);
}

int main(int argc, char *argv[])
{
    int bag_length;
    uint8_t buffer[100];
    uint8_t crc[2];
    uint tmp;
    cout << "input bag length:" << endl;

    cin >> bag_length;
    //0x58 0x08 0x60 0xc1 0x0e 0xfd 0xff 0x0c 0x00 0x49 0x00
    //58 08 60 c1 0e fd ff 0c 00 49 00
    for (int i = 0; i < bag_length; i++)
    {
        cin >> hex >> tmp;
        buffer[i] = uint8_t(tmp);
    }

    crc16_calculate(bag_length, buffer, crc);
    cout << "result:" << endl;
    // printf("crc0: %x", crc[0]);
    // printf("crc1: %x", crc[1]);
    cout << hex << uint(crc[0]) << endl;
    cout << hex << uint(crc[1]) << endl;
}
