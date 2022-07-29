#include <vector>

#include <kdl/frames.hpp>


class LowPassFIRFilter {
public:
    explicit LowPassFIRFilter(size_t window_size) {
        if (window_size < 2) {
            throw std::invalid_argument("Low pass filter with a window size < 2 does not make sense.");
        }
        window_size_ = window_size;

        ring_buffer_ = std::vector<KDL::Wrench>(window_size, KDL::Wrench::Zero());
        ring_buffer_index_ = 0;
        measurements_sum_ = KDL::Wrench::Zero();
    }

    void push_measurement(const KDL::Wrench& measurement) {
        // delete old value in ring buffer
        measurements_sum_ -= ring_buffer_[ring_buffer_index_];

        // add new measurement to ring buffer
        measurements_sum_ += measurement;
        ring_buffer_[ring_buffer_index_] = measurement;

        // move index around ring buffer
        ring_buffer_index_++;
        ring_buffer_index_ = ring_buffer_index_ % ring_buffer_.size();

        // TODO: Decide, if measurements_sum_ should be recalculated from time to time
        // In case of values with orders of magnitude difference, the sum might diverge
        // from 0, due to numerical issues.
    }

    KDL::Wrench get_current_mean() const {
        return measurements_sum_ / ring_buffer_.size();
    }

private:
    size_t window_size_;
    std::vector<KDL::Wrench> ring_buffer_;
    size_t ring_buffer_index_;
    KDL::Wrench measurements_sum_;
};
