#include "gpio.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include "logger.hpp"

#define SYSFS_GPIO_DIR "/sys/class/gpio"
#define POLL_TIMEOUT (3 * 1000)
#define MAX_BUF 64

std::map<std::string, Gpio::pin_number> Gpio::gpio_map = {
    {"GPIO0", Gpio::PIN_GPIO0},
    {"GPIO1", Gpio::PIN_GPIO1},
    {"GPIO2", Gpio::PIN_GPIO2},
    {"GPIO3", Gpio::PIN_GPIO3}
};


Gpio::Gpio(pin_number gpio)
    : io_(gpio)
{
    opened_ = gpio_export();
}

bool Gpio::gpio_export()
{
    int fileDescriptor, length;
    char commandBuffer[MAX_BUF];
    char fnBuffer[MAX_BUF];

    fileDescriptor = open(SYSFS_GPIO_DIR "/export", O_WRONLY);
    if (fileDescriptor < 0) // 若文件打开失败
    {
        LOG(LOG_WARN) << "gpio_export unable to open gpio: "<< find_io(io_) << endll;
        return false;
    }

    length = snprintf(commandBuffer, sizeof(commandBuffer), "%d", io_);

    snprintf(fnBuffer, sizeof(fnBuffer), SYSFS_GPIO_DIR  "/gpio%d", io_);

    if(access(fnBuffer, F_OK) != 0) // 若文件存在
    {
        if (write(fileDescriptor, commandBuffer, length) != length) // 写入, 若长度不一致则报错
        {
            LOG(LOG_WARN) << "gpio "<<find_io(io_)<<" export error!" <<endll;
            return false;
        }
    }
    close(fileDescriptor);
    return true;
}

bool Gpio::gpio_unexport()
{
    if(!opened_) return false;
    int fileDescriptor, length;
    char commandBuffer[MAX_BUF];

    fileDescriptor = open(SYSFS_GPIO_DIR "/unexport", O_WRONLY);
    if (fileDescriptor < 0) 
    {
        LOG(LOG_WARN) << "gpio_unexport unable to open gpio: "<< find_io(io_) <<endll;
        return false;
    }

    length = snprintf(commandBuffer, sizeof(commandBuffer), "%d", io_);
    if (write(fileDescriptor, commandBuffer, length) != length) 
    {
        LOG(LOG_WARN) << "gpio "<<find_io(io_)<<" unexport error!" <<endll;
        return false ;
    }
    close(fileDescriptor);
    return true;
}

bool Gpio::set_direction(pin_direction dir)
{
    if(!opened_) return false;
    int fileDescriptor;
    char commandBuffer[MAX_BUF];
    snprintf(commandBuffer, sizeof(commandBuffer), SYSFS_GPIO_DIR  "/gpio%d/direction", io_);

    fileDescriptor = open(commandBuffer, O_WRONLY);
    if (fileDescriptor < 0) 
    {
        LOG(LOG_WARN) << "gpio_export unable to open gpio: "<< find_io(io_) <<endll;
        return false;
    }

    if (dir == PIN_OUTPUT) 
    {
        if (write(fileDescriptor, "out", 4) != 4) 
        {
            LOG(LOG_WARN) << "gpio set direction error!" <<endll;
            return false ;
        }
    }
    else 
    {
        if (write(fileDescriptor, "in", 3) != 3) 
        {
            LOG(LOG_WARN) << "gpio set direction error!" <<endll;
            return false ;
        }
    }
    close(fileDescriptor);
    return true;
}

bool Gpio::set_value(pin_value v)
{
    if(!opened_) return false;
    int fileDescriptor;
    char commandBuffer[MAX_BUF];

    snprintf(commandBuffer, sizeof(commandBuffer), SYSFS_GPIO_DIR "/gpio%d/value", io_);

    fileDescriptor = open(commandBuffer, O_WRONLY);
    if (fileDescriptor < 0) 
    {
        LOG(LOG_WARN) << "unable to open gpio: "<< find_io(io_) <<endll;
        return false;
    }

    if (v == PIN_HIGH) 
    {
        if (write(fileDescriptor, "1", 2) != 2) 
        {
            LOG(LOG_WARN) << "gpio set value error!" <<endll;
            return false ;
        }
    }
    else 
    {
        if (write(fileDescriptor, "0", 2) != 2) 
        {
            LOG(LOG_WARN) << "gpio set value error!" <<endll;
            return false ;
        }
    }
    close(fileDescriptor);
    return 0;
}

bool Gpio::set_edge(char *edge)
{
    if(!opened_) return false;
    int fileDescriptor;
    char commandBuffer[MAX_BUF];

    snprintf(commandBuffer, sizeof(commandBuffer), SYSFS_GPIO_DIR "/gpio%d/edge", io_);

    fileDescriptor = open(commandBuffer, O_WRONLY);
    if (fileDescriptor < 0) 
    {
        LOG(LOG_WARN) << "unable to open gpio: "<< find_io(io_) <<endll;
        return false;
    }

    if (write(fileDescriptor, edge, strlen(edge) + 1) != ((int)(strlen(edge) + 1))) 
    {
        LOG(LOG_WARN) << "gpio set edge error!" <<endll;
        return false ;
    }
    close(fileDescriptor);
    return true;
}

int Gpio::get_value()
{
    if(!opened_) return -1;
    int fileDescriptor;
    char commandBuffer[MAX_BUF];
    char ch;
    int res;

    snprintf(commandBuffer, sizeof(commandBuffer), SYSFS_GPIO_DIR "/gpio%d/value", io_);

    fileDescriptor = open(commandBuffer, O_RDONLY);
    if (fileDescriptor < 0) 
    {
        LOG(LOG_WARN) << "unable to open gpio: "<< find_io(io_) <<endll;
        return -1;
    }

    if (read(fileDescriptor, &ch, 1) != 1) 
    {
        LOG(LOG_WARN) << "gpio get value error!" <<endll;
        return -1;
     }

    if (ch != '0') 
    {
        res = 1;
    } else 
    {
        res = 0;
    }

    close(fileDescriptor);
    return res;
}