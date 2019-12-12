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

#include <mico/dvs/corner_detectors/distinct_queue.h>

namespace mico{

  DistinctQueue::DistinctQueue(int window_size, int queue_size, bool use_polarity) :
    LocalEventQueues(window_size, queue_size)
  {
    // create queues
    const int polarities = use_polarity ? 2 : 1;
    const int num_queues = sensorWidth_*sensorHeight_ * polarities;

    queues_ = std::vector<FixedDistinctQueue>
              (num_queues, FixedDistinctQueue(2*window_size+1, queue_size));
  }

  DistinctQueue::~DistinctQueue(){
  }

  bool DistinctQueue::isFull(int x, int y, bool pol) const{
    return queues_[getIndex(x, y, pol)].isFull();
  }

  void DistinctQueue::newEvent(int x, int y, bool pol)
  {
    // update neighboring pixels
    for (int dx=-window_size_; dx<=window_size_; dx++)
    {
      for (int dy=-window_size_; dy<=window_size_; dy++)
      {
        // in limits?
        if (x+dx<0 or x+dx>=sensorWidth_ or y+dy<0 or y+dy>=sensorHeight_)
        {
          continue;
        }

        // update pixel's queue
        queues_[getIndex(x+dx, y+dy, pol)].addNew(window_size_+dx,
                                                  window_size_+dy);
      }
    }
  }

  Eigen::MatrixXi DistinctQueue::getPatch(int x, int y, bool pol)
  {
    return queues_[getIndex(x, y, pol)].getWindow();
  }

  int DistinctQueue::getIndex(int x, int y, bool polarity) const
  {
    int polarity_offset = polarity ? sensorHeight_*sensorWidth_ : 0;
    return y*sensorWidth_ + x + polarity_offset;
  }

} // namespace
