#pragma once
#include <array>
#include <cmath>
#include <cstddef>
#include <deque>
#include <stdexcept>

// Stores immutable, already transformed odom-frame clouds. Used only by the mapping thread.
template <typename CloudPtr>
class LocalFrameBuffer
{
public:
    struct Frame
    {
        CloudPtr cloud;
        std::array<double, 3> position;
        double stamp;
    };

    void configure(std::size_t max_frames, double min_distance)
    {
        if (max_frames == 0 || !std::isfinite(min_distance) || min_distance < 0)
            throw std::invalid_argument("Invalid local frame buffer limits");
        max_frames_ = max_frames;
        min_distance_ = min_distance;
        clear();
    }

    void clear()
    {
        frames_.clear();
        has_stamp_ = false;
    }

    bool add(CloudPtr cloud, const std::array<double, 3>& position, double stamp)
    {
        if (!cloud || !std::isfinite(stamp)) return false;
        for (double value : position) if (!std::isfinite(value)) return false;
        if (has_stamp_ && stamp < last_stamp_) clear();
        if (has_stamp_ && stamp == last_stamp_) return false;
        last_stamp_ = stamp;
        has_stamp_ = true;
        if (!frames_.empty())
        {
            double distance_squared = 0;
            for (std::size_t axis = 0; axis < 3; ++axis)
            {
                const double delta = position[axis] - frames_.back().position[axis];
                distance_squared += delta * delta;
            }
            if (distance_squared < min_distance_ * min_distance_) return false;
        }
        frames_.push_back(Frame{cloud, position, stamp});
        while (frames_.size() > max_frames_) frames_.pop_front();
        return true;
    }

    const std::deque<Frame>& frames() const { return frames_; }

private:
    std::deque<Frame> frames_;
    std::size_t max_frames_ = 15;
    double min_distance_ = 0.03;
    bool has_stamp_ = false;
    double last_stamp_ = 0;
};
