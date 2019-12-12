//---------------------------------------------------------------------------------------------------------------------
//  CORNER DETECTOR https://github.com/uzh-rpg/rpg_corner_events
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018
//---------------------------------------------------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
//  and associated documentation files (the "Software"), to deal in the Software without restriction,
//  including without limitation the rights to use, copy, modify, merge, publish, distribute,
//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all copies or substantial
//  portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
//  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//---------------------------------------------------------------------------------------------------------------------


#ifndef UTILS_TIMER_H_
#define UTILS_TIMER_H_

// Source: https://gist.github.com/artivis/53a621d65a676723c0e87b6faadaeda8

#include <chrono>

namespace utils
{
namespace time
{
using std::chrono::hours;
using std::chrono::minutes;
using std::chrono::seconds;
using std::chrono::milliseconds;
using std::chrono::microseconds;
using std::chrono::nanoseconds;

namespace details
{
template<typename _Clock, typename _Dur>
using time_point = std::chrono::time_point<_Clock, _Dur>;

using default_clock_t = std::chrono::_V2::high_resolution_clock;
} // namespace details

/**
 * @brief Timer. A tic-toc timer.
 *
 * Mesure the elapsed time between construction - or tic() -
 * and toc(). The elapsed time is expressed in unit.
 *
 * @param unit. The time unit.
 * @see unit
 */
template <typename unit>
class Timer
{
public:

  /**
   * @brief Timer. Launch the timer.
   */
  Timer(): start_(now()) { }

  /**
   * @brief ~Timer. Default desctructor.
   */
  ~Timer() = default;

  /**
   * @brief tic. Reset the timer.
   */
  void tic()
  {
    start_ = now();
  }

  /**
   * @brief toc. Return this elapsed time since construction or last tic().
   * @return double. The elapsed time.
   * @see tic()
   */
   template <typename T = int64_t>
   T toc()
   {
     return static_cast<T>(cast_d(now() - start_).count());
   }

protected:

  details::time_point<details::default_clock_t, unit> start_;

  template <typename... Args>
  auto cast_d(Args&&... args) ->
    decltype(std::chrono::duration_cast<unit>(std::forward<Args>(args)...))
  {
    return std::chrono::duration_cast<unit>(std::forward<Args>(args)...);
  }

  template <typename... Args>
  auto cast(Args&&... args) ->
    decltype(std::chrono::time_point_cast<unit>(std::forward<Args>(args)...))
  {
    return std::chrono::time_point_cast<unit>(std::forward<Args>(args)...);
  }

  auto now() ->
  decltype(std::declval<Timer<unit>>().cast(details::default_clock_t::now()))
  {
    return cast(std::chrono::system_clock::now());
  }
};

using timer_secs  = Timer<seconds>;
using timer_msecs = Timer<milliseconds>;
using timer_usecs = Timer<microseconds>;
using timer_nsecs = Timer<nanoseconds>;

} // namespace time
} // namespace utils
#endif // UTILS_TIMER_H_

/*

Usage :

utils::timer_usecs timer;

// do stuff

auto elapsed_time_usecs = timer.toc();

 */
