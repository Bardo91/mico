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

#ifndef MICO_DVS_CORNERDETECTORS_LOCAL_EVENT_QUEUES_H_
#define MICO_DVS_CORNERDETECTORS_LOCAL_EVENT_QUEUES_H_

#include <Eigen/Dense>

namespace mico{

  class LocalEventQueues{
    public:
      LocalEventQueues(int window_size, int queue_size)
      : window_size_(window_size), queue_size_(queue_size) {}

      virtual void newEvent(int x, int y, bool pol) = 0;
      virtual bool isFull(int x, int y, bool pol) const = 0;
      virtual Eigen::MatrixXi getPatch(int x, int y, bool pol) = 0;

    protected:
      int window_size_;
      int queue_size_;
  };

} // namespace

#endif