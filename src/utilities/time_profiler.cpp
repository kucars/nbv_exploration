#include "utilities/time_profiler.h"

TimeProfiler::TimeProfiler() {
    verbose = true;
    this->start();
}

TimeProfiler::TimeProfiler(bool b) {
    verbose = b;
    this->start();
}

void TimeProfiler::start() {
    start("");
}

void TimeProfiler::start(std::string s) {
    timers[s] = std::chrono::steady_clock::now();
}

void TimeProfiler::stop(std::string s) {
    std::chrono::steady_clock::time_point begin = timers[s];
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    double t = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() / 1e3;

    std::map<std::string,ProfilerEntry>::iterator it;
    it = entries.find(s);

    if (it == entries.end())
    {
      // Entry not found, create new entry
      ProfilerEntry e;
      e.avg = t;
      e.max = t;
      e.min = t;
      e.count = 1;
      entries[s] = e;
    }
    else
    {
      // Update existing entry
      ProfilerEntry e = it->second;
      e.total += t;
      e.count ++;
      e.avg = e.total/e.count;


      if (e.max < t)
        e.max = t;

      if (e.min > t)
        e.min = t;

      it->second = e;
    }

    if(verbose)
    {
      std::cout << entries.size() << "\n";
      std::cout << s << " Time elapsed = " << entries[s].count << "," << entries[s].avg << std::endl;
    }

}

void TimeProfiler::stop() {
    this->stop("");
}

void TimeProfiler::toc() {
    this->toc("");
}

void TimeProfiler::toc(std::string s) {
    this->stop(s);
    this->start();
}

void TimeProfiler::dump() {
  logfile << "Name, Calls, Min Time (ms), Avg Time (ms), Max Time (ms), Total Time (ms)\n";

  std::map<std::string,ProfilerEntry>::iterator it;
  for (it=entries.begin(); it!=entries.end(); ++it)
    logfile << it->first << "," << it->second.count << "," << it->second.min << "," << it->second.avg << ","  << it->second.max << ","  << it->second.total << '\n';

  std::cout << entries.size() << "\n";

  std::ofstream myfile;
  myfile.open("/tmp/TimeProfiler.csv");
  myfile << logfile.rdbuf();
  myfile.close();
}
