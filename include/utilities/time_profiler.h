#ifndef TimeProfiler_H
#define TimeProfiler_H

#include <iostream>
#include <sstream>
#include <chrono>
#include <fstream>
#include <map>

class TimeProfiler {

  public:
    struct ProfilerEntry{
      double min, max, total, avg;
      int count;
    };

    TimeProfiler();
    TimeProfiler(bool b);
    void start();
    void start(std::string s);
    void toc();
    void toc(std::string s);
    void stop();
    void stop(std::string s);
    void dump();

  private:
    bool verbose;
    std::map<std::string,std::chrono::steady_clock::time_point> timers;
    std::map<std::string,ProfilerEntry> entries;
};

#endif // TimeProfiler_H
