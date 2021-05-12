#ifndef __GPIO_HPP
#define __GPIO_HPP

#include <map>
#include <string>

class Gpio
{
public:
    enum pin_direction
    {
        PIN_INPUT  = 0,
        PIN_OUTPUT = 1
    } ;

    enum pin_value
    {
        PIN_LOW = 0,
        PIN_HIGH = 1
    };

    enum pin_number
    {
        PIN_GPIO0 = 388,
        PIN_GPIO1 = 298,
        PIN_GPIO2 = 480,
        PIN_GPIO3 = 486
    };
    
    // 字符串到GPIO编号的map
    static std::map<std::string, pin_number> gpio_map;

    // 根据给定GPIO编号找到对应GPIO对象
    static std::string find_io(pin_number p)
    {
        for(auto &iom: gpio_map)
        {
            if(iom.second == p)
                return iom.first;
        }
        return "NONE";
    }
    
public:
    Gpio(pin_number pin);

    bool set_direction(pin_direction dir);
    bool set_value(pin_value v);
    bool set_edge(char *edge);
    int get_value();
    bool gpio_unexport(); // ? 与gpio_export()相反, 移除一个GPIO节点
    
    inline bool opened()
    {
        return opened_;
    }

private:
    bool gpio_export(); // ? 通过写GPIO的号码到此文件，用户空间可以要求内核导出一个GPIO的控制到用户空间
    
    bool opened_;
    pin_number io_;
};

#endif
