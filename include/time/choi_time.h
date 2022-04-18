#ifndef CHOI_TIME_H
#define CHOI_TIME_H

#pragma once

#include <ctime>
#include <cstdlib>
#include <chrono>
#include <string>
#include <ros/ros.h>

namespace seok {

    class TimeChecker
    {
    public:
        TimeChecker(){ t1 = std::chrono::system_clock::now(); }

        void interval(std::string str)
        {
            t2 = std::chrono::system_clock::now();
            std::chrono::duration<double> dur = t2 - t1;
            ROS_INFO("Time elapsed for %s : %lfs", str.c_str() , dur.count());
        }

    private:
        std::chrono::time_point<std::chrono::system_clock> t1;
        std::chrono::time_point<std::chrono::system_clock> t2;
    };
}
#endif 
