#ifndef ZLAC8015
#define ZLAC8015

#include <string>
#include <iostream>
#include <chrono>

// OS Specific sleep
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
#include "zlac_ros2/crc_check.h"

class ZLAC
{
private:
    LibSerial::SerialStream serial_conn_;
    std::chrono::time_point<std::chrono::steady_clock> start, end;

    uint8_t hex_cmd[8] = {0};
    uint8_t receive_hex[8] = {0};
    uint8_t ID = 0x00;
    uint8_t READ = 0x03;
    uint8_t WRITE = 0x06;
    uint8_t CONTROL_REG[2] = {0X20, 0X31};
    uint8_t ENABLE[2] = {0x00, 0X08};
    uint8_t DISABLE[2] = {0x00, 0X07};
    uint8_t OPERATING_MODE[2] = {0X20, 0X32};
    uint8_t VEL_MODE[2] = {0x00, 0X03};
    uint8_t SET_RPM[2] = {0x20, 0X3A};
    uint8_t GET_RPM[2] = {0x20, 0X2C};
    uint8_t SET_ACC_TIME[2] = {0x20, 0X37};
    uint8_t SET_DECC_TIME[2] = {0x20, 0X38};

    void calculate_crc();
    uint8_t read_hex(uint8_t num_bytes);
    // void print_hex_cmd();
    // void print_rec_hex();

public:
    // ZLAC();
    // ~ZLAC();

    /**
     * @brief open serial port communication
     * @param port COM port eg. "/dev/ttyUSB0"
     * @param baudRate is hard coded to 115200
     * @param _ID Set the modbus ID of the motor driver
     */
    void beginn(const std::string &port, uint8_t ID = 0x00);

    /**
     * @brief close serial port communication
     */
    void close();

    /**
     * @brief Determines if the serial port is open for I/O.
     * @return Returns true iff the serial port is open.
     */
    bool connected();

    uint8_t set_vel_mode();
    // uint8_t set_acc_time(uint16_t acc_time);
    // uint8_t set_decc_time(uint16_t decc_time);
    // uint8_t enable();
    // uint8_t disable();
    // uint8_t set_rpm(int16_t rpm);
    // int16_t get_rpm();

    // void sleep(unsigned long milliseconds);
    void say_hello();
};

#endif