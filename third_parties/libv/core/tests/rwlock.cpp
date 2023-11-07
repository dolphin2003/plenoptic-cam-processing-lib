
#include <stdexcept>
#include <thread>
#include <chrono>
#include <functional>
#include <iostream>

#include <libv/core/rwlock.hpp>

using namespace v::core;

namespace
{
  typedef std::chrono::system_clock Clock;
  typedef std::chrono::time_point<Clock> TimePoint;

  TimePoint now()
  {
    return Clock::now();
  }

  const TimePoint start = now();

  unsigned elapsed_ms()
  {
    return std::chrono::duration_cast<std::chrono::milliseconds>(now() - start).count();
  }

  void sleep_ms(unsigned ms)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
  }

  RwLock rwlock;
  volatile int shared = 0;

  int read(unsigned ms)
  {
    RwLock::Reader r(rwlock);
    sleep_ms(ms);
    return shared;
  }

  int write(int x, unsigned ms)
  {
    RwLock::Writer w(rwlock);