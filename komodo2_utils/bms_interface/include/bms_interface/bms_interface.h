//
// Created by sub on 22/10/17.
//

#ifndef BMS_BMS_INTERFACE_H
#define BMS_BMS_INTERFACE_H

#include <cerrno>
#include <cstring>
#include <stdlib.h>
#include <stdio.h>
#include <termios.h>
#include <errno.h>
#include <string>

#include <fcntl.h>
#include <unistd.h>  // UNIX standard function definitions
#include <stdexcept>
#include <stdint.h>
#include <vector>

#define BUFF_SIZE 500
#define MAX_BAD_READS 200
#define BAUD_RATE B9600

#define BMS_PKG_START_DELIM ':'
#define BMS_PKG_START_DELIM_LEN 1
#define BMS_PKG_END_DELIM '~'
#define BMS_PKG_CRC_BACK_OFFSET 3
#define BMS_PKG_CRC_SIZE 2
#define BMS_PKG_BASE 16 //hexa
#define BMS_PKG_CMD_SUCCESS 130
#define BMS_PKG_PROTOCOL_VERSION 82

#define BMS_PKG_ADDR_INDX                      1
#define BMS_PKG_ADDR_SIZE                      2

#define BMS_PKG_CMD_INDX                       3
#define BMS_PKG_CMD_SIZE                       2

#define BMS_PKG_VER_INDX                       5
#define BMS_PKG_VER_SIZE                       2

#define BMS_PKG_LEN_INDX                       7
#define BMS_PKG_LEN_SIZE                       4

#define BMS_PKG_TIME_INDX                      11
#define BMS_PKG_TIME_SIZE                      14

#define BMS_PKG_VBAT_INDX                      25
#define BMS_PKG_VBAT_SIZE                      4

#define BMS_PKG_CELL_NUM_INDX                  29
#define BMS_PKG_CELL_NUM_SIZE                  2

#define BMS_PKG_CELLS_ARR_INDX                 31
#define BMS_PKG_CELL_SIZE                      4

#define BMS_PKG_CHRG_CURR_INDX                 31
#define BMS_PKG_CHRG_CURR_SIZE                 4

#define BMS_PKG_DCHRG_CURR_INDX                35
#define BMS_PKG_DCHRG_CURR_SIZE                4

/* assuming cell size is 0 (for other values, multiply) */
#define BMS_PKG_TEMP_NUM_INDX                  39
#define BMS_PKG_TEMP_NUM_SIZE                  2

#define BMS_PKG_TEMPS_ARRAY_INDX               41
#define BMS_PKG_TEMP_SIZE                      2

/* assuming cell size is 0 (for other values, multiply) */
#define BMS_PKG_VSTATE_INDX                    41
#define BMS_PKG_VSTATE_SIZE                    4

#define BMS_PKG_CSTATE_INDX                    45
#define BMS_PKG_CSTATE_SIZE                    4

#define BMS_PKG_TSTATE_INDX                    49
#define BMS_PKG_TSTATE_SIZE                    4

#define BMS_PKG_ALARM_INDX                     53
#define BMS_PKG_ALARM_SIZE                     4

#define BMS_PKG_FET_STATE_INDX                 57
#define BMS_PKG_FET_STATE_SIZE                 2

#define BMS_PKG_WARN_VOV_INDX                  59
#define BMS_PKG_WARN_VOV_SIZE                  4

#define BMS_PKG_WARN_VUV_INDX                  63
#define BMS_PKG_WARN_VUV_SIZE                  4

#define BMS_PKG_NUM_WARN_VHIGH_INDX            67
#define BMS_PKG_NUM_WARN_VHIGH_SIZE            4

#define BMS_PKG_NUM_WARN_VLOW_INDX             71
#define BMS_PKG_NUM_WARN_VLOW_SIZE             4

#define BMS_PKG_BALANCE_STATE_INDX             75
#define BMS_PKG_BALANCE_STATE_SIZE             4

#define BMS_PKG_DCHG_NUM_INDX                  79
#define BMS_PKG_DCHG_NUM_SIZE                  4

#define BMS_PKG_CHG_NUM_INDX                   83
#define BMS_PKG_CHG_NUM_SIZE                   4

#define BMS_PKG_SOC_INDX                       87
#define BMS_PKG_SOC_SIZE                       2

#define BMS_PKG_CAP_NOW_INDX                   89
#define BMS_PKG_CAP_NOW_SIZE                   4

#define BMS_PKG_CAP_FULL_INDX                  93
#define BMS_PKG_CAP_FULL_SIZE                  4


namespace bms
{
    struct data
    {
        bool valid = false;
        uint8_t addr = 0; //Device Address
        uint8_t cmd = 0; //Command for BMS
        uint8_t ver = 0; //Protocol Version
        uint16_t len = 0; //Length of the entire posted data
        ulong time = 0;
        float vbat = 0; //battery total voltage (V)
        uint8_t cell_num = 0; //Number of cells 1~16
        std::vector<float> vcells; //all voltages by order (V)
        float chrg_current = 0; //CHG current
        float dchrg_current = 0; //DSG current
        uint8_t temp_num = 0; //Number of temp sensors (m)
        std::vector<uint8_t> temps; //temp sensors array
        uint8_t temp_max = 0;
        uint16_t vstate = 0;

        /* raw cstate_state and bits flags fetched from it */
        uint16_t cstate = 0;
        uint8_t is_dchrg = 0; //1 if discharging, 0 if not
        uint8_t is_chrg = 0; //1 if charging, 0 if not

        uint16_t tstate = 0;
        uint16_t alarm = 0;

        uint8_t fet_state = 0;

        uint16_t warn_vov = 0; //single cell high voltage warning value mV
        uint16_t warn_vuv = 0; //single cell undervoltage warning value
        uint16_t num_warn_vhigh = 0; //battery pack high voltage warning value
        uint16_t num_warn_vlow = 0; //battery pack undervoltage warning value
        uint16_t balance_state = 0; //Balance state for V0 to V15
        uint16_t dchg_num = 0; //Number of discharge events
        uint16_t chg_num = 0; //Number of charge events
        uint8_t soc = 0; //Battery SOC [%]
        uint16_t cap_now = 0; //Current capacity
        uint16_t cap_full = 0; //Full Charge Capacity [Ah]
    };


    class BMSException : public std::runtime_error
    {
    public:
        BMSException(std::string err_msg) : std::runtime_error(err_msg){}
    };

    class BMSErrorException : public BMSException
    {
    public:
        BMSErrorException(std::string err_msg) : BMSException(err_msg){}
    };

    class BMSWarnException : public BMSException
    {
    public:
        BMSWarnException(std::string err_msg) : BMSException(err_msg){}
    };


    class BMSInterface
    {
    private:
        int file_handle_;
        const std::string SEND_ME_DATA_ = ":000200000ee8~";

        void setAttributes();
        int readPkgFromBMS(char *buff, size_t buff_size_limit);
        data decodePkg(char buff[], size_t buff_size);
        int fetchParam(char buff[], int param_indx, size_t param_size);

    public:
        void connect(std::string port);
        data read();
        BMSInterface() {int a = BMS_PKG_CAP_FULL_INDX;}
        ~BMSInterface();
        uint8_t getBitInByte(uint8_t byte, uint8_t position)
        {
            return (byte >> position) & 0x1;
        }

        uint8_t getBitInWord(uint16_t byte, uint8_t position)
        {
            return (byte >> position) & 0x1;
        }

    };
}


#endif //BMS_BMS_INTERFACE_H
