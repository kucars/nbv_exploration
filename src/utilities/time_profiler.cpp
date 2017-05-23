#include "utilities/time_profiler.h"
#include <iomanip>


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

    /*
    if(verbose)
    {
      std::cout << entries.size() << "\n";
      std::cout << s << " Time elapsed = " << entries[s].count << "," << entries[s].avg << std::endl;
    }
    */

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
  // Write to CSV
  std::stringstream logfile;
  std::map<std::string,ProfilerEntry>::iterator it;

  logfile << "Name, Calls, Min Time (ms), Avg Time (ms), Max Time (ms), Total Time (ms)\n";

  for (it=entries.begin(); it!=entries.end(); ++it)
    logfile << it->first << "," << it->second.count << "," << it->second.min << "," << it->second.avg << ","  << it->second.max << ","  << it->second.total << '\n';

  std::ofstream myfile;
  myfile.open("/tmp/TimeProfiler.csv");
  myfile << logfile.rdbuf();
  myfile.close();



  if(!verbose)
    return;

  // Human readable console version
  std::stringstream stdout;

  size_t headerWidths[6] = {
      std::string("Name").size(),
      std::string("Calls").size(),
      std::string("Min Time (ms)").size(),
      std::string("Avg Time (ms)").size(),
      std::string("Max Time (ms)").size(),
      std::string("Total Time (ms)").size()
  };

  // Get max size of name
  for (it=entries.begin(); it!=entries.end(); ++it)
  {
    if (it->first.size() > headerWidths[0])
      headerWidths[0] = it->first.size();
  }

  stdout << std::left  << std::setw(headerWidths[0]) << "Name";
  stdout << std::right << std::setw(headerWidths[1]) << "  Calls";
  stdout << std::right << std::setw(headerWidths[2]) << "  Min Time (ms)";
  stdout << std::right << std::setw(headerWidths[3]) << "  Avg Time (ms)";
  stdout << std::right << std::setw(headerWidths[4]) << "  Max Time (ms)";
  stdout << std::right << std::setw(headerWidths[5]) << "  Total Time (ms)";
  stdout << "\n";

  for (it=entries.begin(); it!=entries.end(); ++it)
  {
    stdout << std::left  << std::setw(headerWidths[0]) << it->first;
    stdout << "  " << std::right << std::setw(headerWidths[1]) << it->second.count;
    stdout << "  " << std::right << std::setw(headerWidths[2]) << it->second.min;
    stdout << "  " << std::right << std::setw(headerWidths[3]) << it->second.avg;
    stdout << "  " << std::right << std::setw(headerWidths[4]) << it->second.max;
    stdout << "  " << std::right << std::setw(headerWidths[5]) << it->second.total;
    stdout << "\n";
  }

  // Output
  std::cout << stdout.rdbuf();
}
