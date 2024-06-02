#pragma once

#include <cmath>
#include <cstdint>
#include <vector>
#include <stdexcept>
#include <algorithm>

class SomeGaussianFilter {
private:
    std::vector<float> kernel_;
    int kernel_size_;
    int half_kernel_size_;

public:
    SomeGaussianFilter(const std::vector<float>& kernel) : kernel_(kernel) {
        const int length = kernel_.size();
        kernel_size_ = std::sqrt(length);
        if (kernel_size_ * kernel_size_ != length) {
            throw std::invalid_argument("Invalid kernel size: Kernel must form a square matrix");
        }

        half_kernel_size_ = kernel_size_ / 2;
    }

    void apply(std::vector<int8_t, std::allocator<int8_t>> &image, int width, int height) const {
        // Create an output vector to store the result
        std::vector<uint8_t, std::allocator<int8_t>> buffer(width * height);

        // Apply convolution
        for (int x = half_kernel_size_; x < width - half_kernel_size_; ++x) {
            for (int y = half_kernel_size_; y < height - half_kernel_size_; ++y) {
                float newValue = 0;
                for (int i = -half_kernel_size_; i <= half_kernel_size_; ++i) {
                    for (int j = -half_kernel_size_; j <= half_kernel_size_; ++j) {
                        float kernelValue = kernel_[(j + half_kernel_size_) * kernel_size_ + (i + half_kernel_size_)];
                        newValue += image[(y + j) * width + (x + i)] * kernelValue;
                    }
                }
                buffer[y * width + x] = std::round(newValue);
            }
        }

        std::move(buffer.begin(), buffer.end(), image.begin());
    }
};
