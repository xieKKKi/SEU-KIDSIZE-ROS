#ifndef __IMU_HPP
#define __IMU_HPP

#include <boost/asio.hpp>
#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <eigen3/Eigen/Dense>
#include <vector>

enum FallDirection
{
    FALL_NONE = 0,
    FALL_FORWARD = 1,
    FALL_BACKWARD = -1,
    FALL_LEFT = 2,
    FALL_RIGHT = -2
};

class Imu: public std::enable_shared_from_this<Imu>
{
public:
    struct imu_data
    {
        float pitch=0.0, roll=0.0, yaw=0.0;
        float ax=0.0, ay=0.0, az=0.0;
        float wx=0.0, wy=0.0, wz=0.0;
        int timestamp=0;
    };

    enum {MAX_PACKET_LEN=128};

    enum ItemID_t
    {
        kItemKeyStatus =            0x80,   /* key status           size: 4 */
        kItemID =                   0x90,   /* user programed ID    size: 1 */
        kItemUID =                  0x91,   /* Unique ID            size: 4 */
        kItemIPAdress =             0x92,   /* ip address           size: 4 */
        kItemAccRaw =               0xA0,   /* raw acc              size: 3x2 */
        kItemAccCalibrated =        0xA1,
        kItemAccFiltered =          0xA2,   
        kItemAccLinear =            0xA5,
        kItemGyoRaw =               0xB0,   /* raw gyro             size: 3x2 */  
        kItemGyoCalibrated =        0xB1,
        kItemGyoFiltered =          0xB2, 
        kItemMagRaw =               0xC0,   /* raw mag              size: 3x2 */
        kItemMagCalibrated =        0xC1,
        kItemMagFiltered =          0xC2,
        kItemRotationEular =        0xD0,   /* eular angle          size:3x2 */
        kItemRotationEular2 =       0xD9,   /* new eular angle      size:3x4 */
        kItemRotationQuat =         0xD1,   /* att q,               size:4x4 */
        kItemTemperature =          0xE0,   
        kItemPressure =             0xF0,   /* pressure             size:1x4 */
        kItemEnd =                  0x00,   
    };

    struct Packet_t
    {
        uint32_t ofs;
        uint8_t buf[MAX_PACKET_LEN];    /* total frame buffer */
        uint16_t payload_len;           
        uint16_t len;                   /* total frame len */
        uint8_t type;
    };

    Imu();
    ~Imu();

    bool start(std::string dev_name, unsigned int baudrate);
    void stop();

    imu_data data() const
    {
        return imu_data_;
    }

    float imu_yaw() const
    {
        return imu_data_.yaw;
    }

    int fall_direction() const
    {
        return fall_direction_;
    }
private:
    bool open(std::string dev_name, unsigned int baudrate);
    void read_data();

    bool Packet_Decode(uint8_t c);
    void OnDataReceived(Packet_t &pkt);

    enum {imu_data_size = sizeof(imu_data)};

    unsigned char buff_[2];
    imu_data imu_data_;
    std::thread td_;

    boost::asio::serial_port serial_;
    std::atomic_int fall_direction_;

    Eigen::Vector2f pitch_range_;
    Eigen::Vector2f roll_range_;

    Packet_t rx_pkt_;
    int16_t acc[3];
    int16_t gyo[3];
    int16_t mag[3];
    float eular[3];
    float quat[4];
    uint8_t id;

    float init_dir_;
};

#endif
