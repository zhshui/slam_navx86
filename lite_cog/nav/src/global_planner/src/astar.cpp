/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include<global_planner/astar.h>
#include<costmap_2d/cost_values.h>

namespace global_planner {

AStarExpansion::AStarExpansion(PotentialCalculator* p_calc, int xs, int ys) :
        Expander(p_calc, xs, ys) {
}

bool AStarExpansion::calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y,
                                        int cycles, float* potential) {
    queue_.clear();
    parents_.assign(ns_, -1);
    int start_i = toIndex(start_x, start_y);
    if (start_x < 0 || start_y < 0 || start_x >= nx_ || start_y >= ny_ ||
        end_x < 0 || end_y < 0 || end_x >= nx_ || end_y >= ny_) return false;
    parents_[start_i] = start_i;
    queue_.push_back(Index(start_i, 0));

    std::fill(potential, potential + ns_, POT_HIGH);
    potential[start_i] = 0;

    int goal_i = toIndex(end_x, end_y);
    int cycle = 0;

    while (queue_.size() > 0 && cycle < cycles) {
        Index top = queue_[0];
        std::pop_heap(queue_.begin(), queue_.end(), greater1());
        queue_.pop_back();

        int i = top.i;
        if (i == goal_i)
            return true;

        if (i % nx_ + 1 < nx_) add(costs, potential, potential[i], i + 1, end_x, end_y, i);
        if (i % nx_ > 0) add(costs, potential, potential[i], i - 1, end_x, end_y, i);
        add(costs, potential, potential[i], i + nx_, end_x, end_y, i);
        add(costs, potential, potential[i], i - nx_, end_x, end_y, i);

        cycle++;
    }

    return false;
}

void AStarExpansion::add(unsigned char* costs, float* potential, float prev_potential, int next_i, int end_x,
                         int end_y, int parent) {
    if (next_i < 0 || next_i >= ns_)
        return;

    if (potential[next_i] < POT_HIGH)
        return;

    if(costs[next_i]>=lethal_cost_ && !(unknown_ && costs[next_i]==costmap_2d::NO_INFORMATION))
        return;

    potential[next_i] = p_calc_->calculatePotential(potential, costs[next_i] + neutral_cost_, next_i, prev_potential);
    parents_[next_i] = parent;
    int x = next_i % nx_, y = next_i / nx_;
    float distance = abs(end_x - x) + abs(end_y - y);

    queue_.push_back(Index(next_i, potential[next_i] + distance * neutral_cost_));
    std::push_heap(queue_.begin(), queue_.end(), greater1());
}

bool AStarExpansion::getParentPath(double start_x, double start_y, double end_x, double end_y,
                                  std::vector<std::pair<float, float>>& path) const {
    path.clear();
    if (start_x < 0 || start_y < 0 || start_x >= nx_ || start_y >= ny_ ||
        end_x < 0 || end_y < 0 || end_x >= nx_ || end_y >= ny_) return false;
    const int start = static_cast<int>(start_y) * nx_ + static_cast<int>(start_x);
    int current = static_cast<int>(end_y) * nx_ + static_cast<int>(end_x);
    if (parents_.size() != static_cast<size_t>(ns_)) return false;
    path.emplace_back(end_x, end_y);
    for (int count = 0; count < ns_; ++count) {
        if (current == start) {
            path.emplace_back(start_x, start_y);
            return true;
        }
        const int parent = parents_[current];
        if (parent < 0 || parent >= ns_ || parent == current) break;
        current = parent;
        path.emplace_back(current % nx_, current / nx_);
    }
    path.clear();
    return false;
}

} //end namespace global_planner
