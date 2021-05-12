#ifndef __TIMER_HPP
#define __TIMER_HPP

#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include "class_exception.hpp"

class Timer
{
public:
    Timer(const int &ms)
    {
        is_alive_ = false;
        period_ms_ = ms;
    }

    int period_ms() const
    {
        return period_ms_;
    }

    static void timer_thread(union sigval v)
    {
        Timer *t = (Timer *) v.sival_ptr;

        if (t->is_alive_)
        {
            t->run();
        }
    }

    void start_timer()
    {
        if (period_ms_ > 0)
        {
            is_alive_ = true;

            memset(&se_, 0, sizeof(struct sigevent));

            se_.sigev_value.sival_ptr = (void *)this;
            se_.sigev_notify = SIGEV_THREAD;
            se_.sigev_notify_function = timer_thread;

            int sec = period_ms_ / 1000;
            int msc = period_ms_ % 1000;
            it_.it_interval.tv_sec = sec;
            it_.it_interval.tv_nsec = msc * 1000000;
            it_.it_value.tv_sec = sec;
            it_.it_value.tv_nsec = msc * 1000000;

            if (timer_create(CLOCK_REALTIME, &se_, &t_) < 0)
            {
                throw ClassException<Timer>("Timer create failed.");
            }

            if (timer_settime(t_, 0, &it_, 0) < 0)
            {
                throw ClassException<Timer>("Timer setting failed.");
            }
        }
    }

    void delete_timer()
    {
        if (period_ms_ > 0)
        {
            timer_delete(t_);
            is_alive_ = false;
        }
    }

    ~Timer()
    {
        if (is_alive_)
        {
            delete_timer();
        }
    }
    virtual void run() = 0;//虚函数，运行时会调用派生类的run函数
private:
    timer_t t_;
    sigevent se_;
    itimerspec it_;
protected:
    bool is_alive_;
    int period_ms_;
};

#endif
