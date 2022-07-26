/**
 * @author Tuhin Das - MARCH 7
 */

#ifndef MARCH_MATH_UTILITIES
#define MARCH_MATH_UTILITIES

#include <array>
#include <cmath>
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
template <std::size_t K, std::size_t W, std::size_t H>
void convolve2D(std::array<std::array<double, K>, K>& kernel,
    std::array<std::array<double, W>, H>& source,
    std::array<std::array<double, W>, H>& destination)
{
    for (int i = K / 2; i < H - K / 2; i++) {
        for (int j = K / 2; j < W - K / 2; j++) {
            double sum = 0;
            for (int a = 0; a < K; a++) {
                for (int b = 0; b < K; b++) {
                    sum += kernel[a][b] * source[i + (a - 1)][j + (b - 1)];
                }
            }
            destination[i][j] = sum;
        }
    }
}

/**
 * Smoothen a matrix by convolving a Gaussian kernel.
 *
 * @param source reference of the matrix to apply the convolution to
 * @param destination reference of array where the result is stored
 */
template <std::size_t W, std::size_t H>
void convolveGaussianKernel(std::array<std::array<double, W>, H>& source,
    std::array<std::array<double, W>, H>& destination)
{
    std::array<std::array<double, 3>, 3> gaussian = {
        { { 1.0 / 16, 2.0 / 16, 1.0 / 16 }, { 2.0 / 16, 4.0 / 16, 2.0 / 16 },
            { 1.0 / 16, 2.0 / 16, 1.0 / 16 } }
    };

    convolve2D(gaussian, source, destination);
}

/**
 * Compute the second derivatives of a matrix with a Laplacian kernel.
 *
 * @param source reference of the matrix to apply the convolution to
 * @param destination reference of array where the result is stored
 */
template <std::size_t W, std::size_t H>
void convolveLaplacianKernel(std::array<std::array<double, W>, H>& source,
    std::array<std::array<double, W>, H>& destination)
{
    std::array<std::array<double, 3>, 3> laplacian
        = { { { 1 / 6.0, 4 / 6.0, 1 / 6.0 }, { 4 / 6.0, -20 / 6.0, 4 / 6.0 },
            { 1 / 6.0, 4 / 6.0, 1 / 6.0 } } };

    convolve2D(laplacian, source, destination);
}

/**
 * Compute the average point of a vector of points.
 *
 * @param points a list of pcl points
 * @return pcl::PointXYZ the average point
 */
inline Point computeAveragePoint(const std::vector<Point>& points)
{
    Point avg(/*_x=*/0, /*_y=*/0, /*_z=*/0);

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

/**
 * Create a domain with n 3D points linearly spaced between a start and end
 * point.
 *
 * @param Point a first point of the domain
 * @param Point b last point of the domain
 * @param int n number of points in the domain
 * @return a vector of evenly spaced 3D points
 */
inline std::vector<Point> linearDomain(Point a, Point b, int n)
{
    n--;
    std::vector<Point> points;
    for (int i = 0; i <= n; i++) {
        auto x = (float)((b.x - a.x) / (double)n * i + a.x);
        auto y = (float)((b.y - a.y) / (double)n * i + a.y);
        auto z = (float)((b.z - a.z) / (double)n * i + a.z);
        points.emplace_back(Point(x, y, z));
    }
    return points;
}

/**
 * Rotate a point clockwise.
 *
 * @param Point point to rotate
 * @return Point new point rotated clockwise
 */
inline Point rotateRight(const Point& p)
{
    Point p_rotated(p.y, -p.x, p.z);
    return p_rotated;
}

/**
 * Rotate a point counter-clockwise.
 *
 * @param Point point to rotate
 * @return Point new point rotated counter-clockwise
 */
inline Point rotateLeft(const Point& p)
{
    Point p_rotated(-p.y, p.x, p.z);
    return p_rotated;
}

/**
 * Add two points together.
 *
 * @param Point p1 first point
 * @param Point p2 second point
 * @return Point new point that is equivalent to p1 + p2
 */
inline Point addPoints(const Point p1, const Point p2)
{
    Point sum(p1.x + p2.x, p1.y + p2.y, p1.z + p2.z);
    return sum;
}

/**
 * Take the difference between two points.
 *
 * @param Point p1 first point
 * @param Point p2 second point
 * @return Point new point that is equivalent to p1 - p2
 */
inline Point subtractPoints(const Point p1, const Point p2)
{
    Point difference(p1.x - p2.x, p1.y - p2.y, p1.z - p2.z);
    return difference;
}

/**
 * Check if a point contains realistic values.
 *
 * @param Point p point to check
 * @return bool whether the point can be published
 */
inline bool validatePoint(const Point p)
{
    return (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z)
        && std::abs(p.x) < 1.0 && std::abs(p.y) < 0.7 && std::abs(p.z) < 0.5);
}

#endif // MARCH_MATH_UTILITIES
