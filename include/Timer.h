#pragma once

#include <iostream>
#include <opencv/cv.h>

class Timer
{
public:
  Timer()
    : t_start_(0.0)
    , freq_((double)cvGetTickFrequency())
  {
  }

  virtual ~Timer()
  {
  }

  void start()
  {
    t_start_ = (double)cvGetTickCount();
  }

  double getTimeMicroSec()
  {
    double t = (double)cvGetTickCount();
    return ((t - t_start_)/freq_);
  }

  double getTimeMilliSec()
  {
    double t = (double)cvGetTickCount();
    return ((t - t_start_)/(freq_*1000.f));
  }

  double getTimeSec()
  {
    double t = (double)cvGetTickCount();
    return ((t - t_start_)/(freq_*1000000.f));
  }

  double getFPS()
  {
    return (1.f/getTimeSec());
  }

  double printTimeMicroSec(const std::string& msg = std::string(""))
  {
    double t = getTimeMicroSec();
    std::cout << msg << ": "  << t << " usec" << std::endl;
    return (t);
  };

  double printTimeMilliSec(const std::string& msg = std::string(""))
  {
    double t = getTimeMilliSec();
    std::cout << msg << ": " << t << " msec" << std::endl;
    return (t);
  };

  double printTimeSec(const std::string& msg = std::string(""))
  {
    double t = getTimeSec();
    std::cout << msg << ": " << t << " sec" << std::endl;
    return (t);
  };

protected:
  double t_start_;
  double freq_;
};
