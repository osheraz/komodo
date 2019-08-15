//
// Created by sub on 22/10/17.
//

#include <iostream>
#include "bms_interface/bms_interface.h"

namespace bms
{
    void BMSInterface::connect(std::string port)
    {
        file_handle_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (file_handle_ < 0)
            throw BMSErrorException("Failed to open BMS port");

        // Check if the file descriptor is pointing to a TTY device or not.
        if(!isatty(file_handle_))
            throw BMSErrorException("BMS port is not a tty device");

        setAttributes();
    }

    void BMSInterface::setAttributes()
    {
        struct termios tty;

        if (tcgetattr(file_handle_, &tty) < 0)
            throw BMSErrorException("Failed to get BMS port attributes");

        cfsetospeed(&tty, (speed_t)BAUD_RATE);
        cfsetispeed(&tty, (speed_t)BAUD_RATE);
        tty.c_cflag |= (CLOCAL | CREAD); /* ignore modem controls */
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;      /* 8-bit characters */
        tty.c_cflag &= ~PARENB;  /* no parity bit */
        tty.c_cflag &= ~CSTOPB;  /* only need 1 stop bit */
        tty.c_cflag &= ~CRTSCTS; /* no hardware flowcontrol */

        /* setup for non-canonical mode */
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
        tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
        tty.c_oflag &= ~OPOST;

        /* fetch bytes as they become available */
        tty.c_cc[VMIN] = 1;
        tty.c_cc[VTIME] = 1;
        if (tcsetattr(file_handle_, TCSANOW, &tty) != 0)
            throw BMSErrorException("Failed to set BMS port attributes");

        tcflush(file_handle_,TCIOFLUSH);
    }

    data BMSInterface::read()
    {
        char buff[BUFF_SIZE];
        int pkg_size = readPkgFromBMS(buff, BUFF_SIZE);

        //fprintf(stderr, "\n") ; /////////////////////////////////////////////////////////////////////////////////////////

        return decodePkg(buff, pkg_size);
    }


    //:018252008A0000000000000037B0070FE90FEA0FEA0FE80FE90FE90FEA0000000005434343424300000000000000000F00000000000000000000000000035E00B900C87E~
    /* param: chars buffer, buffer size limit */
    /* return: number of incoming bytes       */
    int BMSInterface::readPkgFromBMS(char *buff, size_t buff_size_limit)
    {
        int write_len, read_len;
        unsigned char data_in;

        int indx = 0;
        int bad_reads = 0;
        bool first_read = true;

        /* ask BMS to send back data */
        write_len = ::write(file_handle_, SEND_ME_DATA_.c_str(), SEND_ME_DATA_.length());
        if (write_len != SEND_ME_DATA_.length())
            throw BMSWarnException("Failed to write to BMS");

        /* read incoming BMS data */
        while (bad_reads < MAX_BAD_READS &&
                indx + 1 < buff_size_limit)
        {

            read_len = ::read(file_handle_, &data_in, 1);
            if (read_len != 1)
            {
                bad_reads ++;
                continue;
            }
            else
            {
                //fprintf(stderr, "%c", data_in) ; /////////////////////////////////////////////////////////////////////////////////////////
                if (first_read)
                {
                    if (data_in != BMS_PKG_START_DELIM)
                        throw BMSWarnException("Invalid BMS pkg start delimiter");
                    else
                        first_read = false;
                }
                buff[indx] = data_in;
                if (data_in == BMS_PKG_END_DELIM) //done reading
                    return indx + 1; //add 1 because indx starts from 0
                indx++;
            }
        }

        throw BMSWarnException("Invalid BMS data");
    }

    /********************************************************************
     * ------------------BMS packet example------------------------------
     *: (SOI), 01 (addr), 82 (cmd), 52 (ver), 0090 (len), 00000000000000
     * (time_t) 48F8 (Vbat), 0A (cell_num=10), 0EA9 (v1), 0EB3 (v2), 0EB6
     * (v3), 0EB4 (v4), 0E8C (v5), 0EB4 (v6), 0E45 (v7), 0E9E (v8), 0E9E
     * (v9), 0E6A (v10), 0000 (curr[0]), 0000 (curr[1]), 02 (TempNum), 47
     * (temp[0]), 45 (temp[1]), 0000 (Vstate), 0000 (Cstate), 0000
     * (Tstate), 0000 (Alarm), 0F (FetState),0000 (WARN_VOV), 0000
     * (WARN_VUV), 0000 (NUM_WARN_VHIGH), 0000 (NUM_WARN_VLOW), 0000
     * (BlanceState), 0000 (DchgNum), 0000 (ChgNum), 2D (soc), 0048
     * ( CapNow ),00A0( CapFull ),4E( CRC ),~( EOI )
     ********************************************************************/
    data BMSInterface::decodePkg(char *buff, size_t buff_size)
    {
        //fprintf(stderr, "\nbuff_size: %d \n", buff_size);
        data pkg_data;

        /****** validate pkg size ******/
        pkg_data.len = fetchParam(buff, BMS_PKG_LEN_INDX, BMS_PKG_LEN_SIZE);
        //fprintf(stderr, "\nlen: %d \n", pkg_data.len);
        if (pkg_data.len != buff_size)
            throw BMSWarnException("BMS pkg len doesn't mach incoming pkg length - corrupted pkg");

        /****** validate crc checksum ******/
        char crc[BMS_PKG_CRC_SIZE + 1]; //+1 to make room for \0
        crc[BMS_PKG_CRC_SIZE] = '\0';
        const int crc_indx = buff_size - BMS_PKG_CRC_BACK_OFFSET;

        uint8_t pkg_crc = fetchParam(buff, crc_indx, BMS_PKG_CRC_SIZE);
        //fprintf(stderr, "\ncrc: %d \n", pkg_crc);

        //calc pkg data crc
        uint8_t calc_crc = 0;
        for (int i = BMS_PKG_START_DELIM_LEN; i < crc_indx; i++)
            calc_crc += buff[i];
        calc_crc ^= 0xff;

        if (pkg_crc != calc_crc)
            throw BMSWarnException("Invalid BMS pkg CRC");

        /****** fetch addr ******/
        pkg_data.addr = fetchParam(buff, BMS_PKG_ADDR_INDX, BMS_PKG_ADDR_SIZE);
        //fprintf(stderr, "\naddr: %d \n", pkg_data.addr);

        /****** fetch cmd ******/
        pkg_data.cmd = fetchParam(buff, BMS_PKG_CMD_INDX, BMS_PKG_CMD_SIZE);
        if (pkg_data.cmd != BMS_PKG_CMD_SUCCESS)
            throw BMSWarnException("BMS cmd flag indicating failure");
        //fprintf(stderr, "\ncmd: %d \n", pkg_data.cmd);

        /****** fetch version ******/
        pkg_data.ver = fetchParam(buff, BMS_PKG_VER_INDX, BMS_PKG_VER_SIZE);
        if (pkg_data.ver != BMS_PKG_PROTOCOL_VERSION)
            throw BMSErrorException("BMS protocol version is not supported by this API");
        //fprintf(stderr, "\nversion: %d \n", pkg_data.ver);

        /****** fetch time ******/
        //protocol doesn't implementing time (battery always return time 0)
        pkg_data.time = fetchParam(buff, BMS_PKG_TIME_INDX, BMS_PKG_TIME_SIZE);
        //fprintf(stderr, "\ntime: %d \n", pkg_data.time);

        /****** fetch Vbat ******/
        uint16_t raw_vbat = fetchParam(buff, BMS_PKG_VBAT_INDX, BMS_PKG_VBAT_SIZE); //output is 0.5 of the total voltage
        pkg_data.vbat = (float)raw_vbat * 2.0 / 1000.0;
        //fprintf(stderr, "\nVbat: %f \n", pkg_data.vbat);

        /****** fetch cells num ******/
        pkg_data.cell_num = fetchParam(buff, BMS_PKG_CELL_NUM_INDX, BMS_PKG_CELL_NUM_SIZE);
        //fprintf(stderr, "\ncell_num: %d \n", pkg_data.cell_num);

        const uint8_t cells_indx_offset = pkg_data.cell_num * BMS_PKG_CELL_SIZE;


        /****** fetch cells voltage ******/
        for (uint8_t cell_indx = 0; cell_indx < pkg_data.cell_num; cell_indx++)
        {
            uint16_t raw_vcell = fetchParam(buff, BMS_PKG_CELLS_ARR_INDX + (cell_indx * BMS_PKG_CELL_SIZE),
                                            BMS_PKG_CELL_SIZE);
            pkg_data.vcells.push_back((float)raw_vcell / 1000.0);
            //fprintf(stderr, "cell num: %d volt: %f | ", cell_indx ,pkg_data.vcells[cell_indx]);
        }
        //fprintf(stderr, "\n");

        /****** fetch CHG ******/
        uint16_t raw_chg = fetchParam(buff, BMS_PKG_CHRG_CURR_INDX + cells_indx_offset, BMS_PKG_CHRG_CURR_SIZE);
        pkg_data.chrg_current = (float)raw_chg / 100.0;
        //fprintf(stderr, "\nchrg_current: volt: %f\n", pkg_data.chrg_current);

        /****** fetch DCHG ******/
        uint16_t raw_dchg = fetchParam(buff, BMS_PKG_DCHRG_CURR_INDX + cells_indx_offset, BMS_PKG_DCHRG_CURR_SIZE);
        pkg_data.dchrg_current = (float)raw_dchg / 100.0;
        //fprintf(stderr, "\ndchrg_current: volt: %f\n", pkg_data.dchrg_current);

        /****** fetch TempNum ******/
        pkg_data.temp_num = fetchParam(buff, BMS_PKG_TEMP_NUM_INDX + cells_indx_offset, BMS_PKG_TEMP_NUM_SIZE);
        //fprintf(stderr, "\ntemp_num: %d\n", pkg_data.temp_num);

        const uint8_t temp_indx_offset = pkg_data.temp_num * BMS_PKG_TEMP_SIZE;

        /****** fetch temp sensors ******/
        for (uint8_t temp_indx = 0; temp_indx < pkg_data.temp_num; temp_indx++)
        {
            uint8_t temp = fetchParam(buff, BMS_PKG_TEMPS_ARRAY_INDX + cells_indx_offset + (temp_indx * BMS_PKG_TEMP_SIZE),
                                      BMS_PKG_TEMP_SIZE);
            if (temp > pkg_data.temp_max)
                pkg_data.temp_max = temp;
            pkg_data.temps.push_back(temp);
            //fprintf(stderr, "temp num: %d temp: %d | ", temp_indx ,pkg_data.temps[temp_indx]);
        }
        //fprintf(stderr, "\ntemp max: %d", pkg_data.temp_max);
        //fprintf(stderr, "\n");

        /****** fetch Vstate ******/
        pkg_data.vstate = fetchParam(buff, BMS_PKG_VSTATE_INDX + temp_indx_offset + cells_indx_offset,  /////////////////////////////////TODO: FETCH ERROR BIT
                                     BMS_PKG_VSTATE_SIZE);
        //fprintf(stderr, "\nVSTATE: %d\n", pkg_data.vstate);

        /****** fetch Cstate ******/
        pkg_data.cstate = fetchParam(buff, BMS_PKG_CSTATE_INDX + temp_indx_offset + cells_indx_offset,  /////////////////////////////////TODO: FETCH ERROR BIT
                                     BMS_PKG_CSTATE_SIZE);
        //fprintf(stderr, "\nCSTATE: %d\n", pkg_data.cstate);
        pkg_data.is_chrg = getBitInWord(pkg_data.cstate, 0);
        pkg_data.is_dchrg = getBitInWord(pkg_data.cstate, 1);

        /****** fetch Tstate ******/
        pkg_data.tstate = fetchParam(buff, BMS_PKG_TSTATE_INDX + temp_indx_offset + cells_indx_offset,  /////////////////////////////////TODO: FETCH ERROR BIT
                                     BMS_PKG_TSTATE_SIZE);
        //fprintf(stderr, "\nTSTATE: %d\n", pkg_data.tstate);

        /****** fetch Alarm ******/
        pkg_data.alarm = fetchParam(buff, BMS_PKG_ALARM_INDX + temp_indx_offset + cells_indx_offset,  /////////////////////////////////TODO: FETCH ERROR BIT
                                     BMS_PKG_ALARM_SIZE);
        //fprintf(stderr, "\nALARM: %d\n", pkg_data.alarm);

        /****** fetch FetState ******/
        pkg_data.fet_state = fetchParam(buff, BMS_PKG_FET_STATE_INDX + temp_indx_offset + cells_indx_offset,  /////////////////////////////////TODO: FETCH ERROR BIT
                                    BMS_PKG_FET_STATE_SIZE);
        //fprintf(stderr, "\nFET-STATE: %d\n", pkg_data.fet_state);

        /****** fetch WARN_VOV ******/
        pkg_data.warn_vov = fetchParam(buff, BMS_PKG_WARN_VOV_INDX + temp_indx_offset + cells_indx_offset,  /////////////////////////////////TODO: FETCH ERROR BIT
                                        BMS_PKG_WARN_VOV_SIZE);
        //fprintf(stderr, "\nWARN_VOV: %d\n", pkg_data.warn_vov);

        /****** fetch WARN_VUV ******/
        pkg_data.warn_vuv = fetchParam(buff, BMS_PKG_WARN_VUV_INDX + temp_indx_offset + cells_indx_offset,  /////////////////////////////////TODO: FETCH ERROR BIT
                                       BMS_PKG_WARN_VUV_SIZE);
        //fprintf(stderr, "\nWARN_VUV: %d\n", pkg_data.warn_vuv);

        /****** fetch NUM_WARN_VHIGH ******/
        pkg_data.num_warn_vhigh = fetchParam(buff, BMS_PKG_NUM_WARN_VHIGH_INDX + temp_indx_offset + cells_indx_offset,  /////////////////////////////////TODO: FETCH ERROR BIT
                                       BMS_PKG_NUM_WARN_VHIGH_SIZE);
        //fprintf(stderr, "\nNUM_WARN_VHIGH: %d\n", pkg_data.num_warn_vhigh);

        /****** fetch NUM_WARN_VLOW ******/
        pkg_data.num_warn_vlow = fetchParam(buff, BMS_PKG_NUM_WARN_VLOW_INDX + temp_indx_offset + cells_indx_offset,  /////////////////////////////////TODO: FETCH ERROR BIT
                                             BMS_PKG_NUM_WARN_VLOW_SIZE);
        //fprintf(stderr, "\nNUM_WARN_VLOW: %d\n", pkg_data.num_warn_vlow);

        /****** fetch BalanceState ******/
        pkg_data.balance_state = fetchParam(buff, BMS_PKG_BALANCE_STATE_INDX + temp_indx_offset + cells_indx_offset,  /////////////////////////////////TODO: FETCH ERROR BIT
                                            BMS_PKG_BALANCE_STATE_SIZE);
        //fprintf(stderr, "\nBALANCE_STATE: %d\n", pkg_data.balance_state);

        /****** fetch DchgNum ******/
        pkg_data.dchg_num = fetchParam(buff, BMS_PKG_DCHG_NUM_INDX + temp_indx_offset + cells_indx_offset,  /////////////////////////////////TODO: FETCH ERROR BIT
                                       BMS_PKG_DCHG_NUM_SIZE);
        //fprintf(stderr, "\nDCHG_NUM: %d\n", pkg_data.dchg_num);

        /****** fetch ChgNum ******/
        pkg_data.chg_num = fetchParam(buff, BMS_PKG_CHG_NUM_INDX + temp_indx_offset + cells_indx_offset,  /////////////////////////////////TODO: FETCH ERROR BIT
                                       BMS_PKG_CHG_NUM_SIZE);
        //fprintf(stderr, "\nCHG_NUM: %d\n", pkg_data.chg_num);

        /****** fetch soc ******/
        pkg_data.soc = fetchParam(buff, BMS_PKG_SOC_INDX + temp_indx_offset + cells_indx_offset,  /////////////////////////////////TODO: FETCH ERROR BIT
                                  BMS_PKG_SOC_SIZE);
        //fprintf(stderr, "\nSOC: %d\n", pkg_data.soc);

        /****** fetch CapNow ******/
        pkg_data.cap_now = fetchParam(buff, BMS_PKG_CAP_NOW_INDX + temp_indx_offset + cells_indx_offset,  /////////////////////////////////TODO: FETCH ERROR BIT
                                      BMS_PKG_CAP_NOW_SIZE) / 10;
        //fprintf(stderr, "\nCAP_NOW: %d\n", pkg_data.cap_now);

        /****** fetch CapFull ******/
        pkg_data.cap_full = fetchParam(buff, BMS_PKG_CAP_FULL_INDX + temp_indx_offset + cells_indx_offset,  /////////////////////////////////TODO: FETCH ERROR BIT
                                      BMS_PKG_CAP_FULL_SIZE) / 10;
        //fprintf(stderr, "\nCAP_FULL: %d\n", pkg_data.cap_full);

        return pkg_data;
    }

    int BMSInterface::fetchParam(char *buff, int param_indx, size_t param_size)
    {
        char param[param_size + 1];
        param[param_size] = '\0';
        strncpy(param, buff + param_indx, param_size);
        return strtol(param, 0, BMS_PKG_BASE);
    }

    BMSInterface::~BMSInterface()
    {
        close(file_handle_);
    }
//:018252008A000000000000002EDE070D640D640D650D640D650D660D60***0000***008A***053F3F3F3F3E00000002000000000F00000000000000000000000000030A001500C8B4~
//:018252008A000000000000002FA0070D950D9F0D9E0D9E0D9F0DA00D91025B0000053F3F3F3F3E00000001000000000F00000000000000000000000000030A001500C86F~
}