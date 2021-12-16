#ifndef MARCH_MATH_UTILITIES
#define MARCH_MATH_UTILITIES

#include <math.h>
#include <pcl/point_types.h>

using Point = pcl::PointXYZ;

/**
 * Applies a kernel convolution to a matrix and stores it in another matrix.
 * Note that the kernel should have odd dimensions.
 *
 * @param kernel the kernel to convolve on the matrix
 * @param source reference of the matrix to apply the convolution to
 * @param destination reference of array where the result is stored
 */
template <int K, int R>
void convolve2D(
    double kernel[K][K], double (&source)[R][R], double (&destination)[R][R])
{
    for (int i = K / 2; i < R - K / 2; i++) {
        for (int j = K / 2; j < R - K / 2; j++) {
            double sum = 0;
            for (int a = 0; a < K; a++)
                for (int b = 0; b < K; b++)
                    sum += kernel[a][b] * source[i + (a - 1)][j + (b - 1)];
            destination[i][j] = sum;
        }
    }
}

/**
 * Compute the average point of a vector of points.
 *
 * @param points a list of pcl points
 * @return pcl::PointXYZ the average point
 */
inline Point computeAveragePoint(const std::vector<Point>& points)
{
    Point avg(0, 0, 0);

    for (const Point& p : points) {
        avg.x += p.x;
        avg.y += p.y;
        avg.z += p.z;
    }

    avg.x /= points.size();
    avg.y /= points.size();
    avg.z /= points.size();

    return avg;
}

#endif // MARCH_MATH_UTILITIES