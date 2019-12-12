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

#ifndef MICO_DVS_CORNERDETECTORS_FIXED_DISTINCT_QUEUE_H_
#define MICO_DVS_CORNERDETECTORS_FIXED_DISTINCT_QUEUE_H_

#include <deque>
#include <vector>
#include <Eigen/Dense>

namespace mico
{

class FixedDistinctQueue{
  
  public:
    FixedDistinctQueue(int window, int queue);

    bool isFull() const;

    void addNew(int x, int y);
    Eigen::MatrixXi getWindow() const;

  private:
    // contains index of queue element if occupied, negative value otherwise
    Eigen::MatrixXi window_;
    // contains one event
    struct QueueEvent
    {
      int prev, next;
      int x, y;
    };
    std::vector<QueueEvent> queue_;
    int first_, last_;
    unsigned int queueMax_;
  };


} // namespace

#endif