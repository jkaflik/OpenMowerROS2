#pragma once

#include <cmath>
#include <cstdint>
#include <vector>
#include <stdexcept>
#include <algorithm>
#include <numeric>

class SomeGaussianFilter
{
private:
  std::vector<float> kernel_;
  int kernel_size_;
  int half_kernel_size_;
  float kernel_sum_;

public:
  SomeGaussianFilter(const std::vector<float>& kernel) : kernel_(kernel)
  {
    const int length = kernel_.size();
    kernel_size_ = std::sqrt(length);
    if (kernel_size_ * kernel_size_ != length)
    {
      throw std::invalid_argument("Invalid kernel size: Kernel must form a square matrix");
    }

    half_kernel_size_ = kernel_size_ / 2;

    // Calculate kernel sum for normalization
    kernel_sum_ = std::accumulate(kernel_.begin(), kernel_.end(), 0.0f);
    if (kernel_sum_ <= 0.0f)
    {
      kernel_sum_ = 1.0f;  // Prevent division by zero
    }
  }

  void apply(std::vector<int8_t>& image, int width, int height) const
  {
    // Create an output vector to store the result
    std::vector<int8_t> buffer(image.size(), -1);  // Initialize with unknown values

    // Apply convolution
    for (int y = 0; y < height; ++y)
    {
      for (int x = 0; x < width; ++x)
      {
        // Skip unknown cells
        if (image[y * width + x] == -1)
        {
          buffer[y * width + x] = -1;
          continue;
        }

        float newValue = 0;
        float weightSum = 0;

        // Apply kernel
        for (int j = -half_kernel_size_; j <= half_kernel_size_; ++j)
        {
          int ny = y + j;
          if (ny < 0 || ny >= height)
            continue;

          for (int i = -half_kernel_size_; i <= half_kernel_size_; ++i)
          {
            int nx = x + i;
            if (nx < 0 || nx >= width)
              continue;

            int index = ny * width + nx;
            if (index < 0 || index >= static_cast<int>(image.size()))
              continue;

            // Treat unknown (-1) values as 100 (occupied)
            // so navigable corners are smoothed.
            int pixelValue = (image[index] == -1) ? 100 : image[index];

            float kernelValue = kernel_[(j + half_kernel_size_) * kernel_size_ + (i + half_kernel_size_)];
            newValue += pixelValue * kernelValue;
            weightSum += kernelValue;
          }
        }

        // Normalize
        if (weightSum > 0)
        {
          // For occupancy grid, values should be between 0 and 100
          newValue = std::round(newValue / weightSum);
          newValue = std::min(std::max(newValue, 0.0f), 100.0f);
          buffer[y * width + x] = static_cast<int8_t>(newValue);
        }
        else
        {
          buffer[y * width + x] = image[y * width + x];  // Keep original if no valid neighbors
        }
      }
    }

    // Copy back to the original image
    std::copy(buffer.begin(), buffer.end(), image.begin());
  }
};
