
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <iostream>

inline bool isEven(int n)
{
    return not(n % 2);
}

inline bool isOdd(int n)
{
    return (n % 2);
}

inline double wrapToPi(double angle)
{
    double ret = angle;
    while (ret > M_PI) {
        ret -= 2 * M_PI;
    }

    while (ret <= -M_PI) {
        ret += 2 * M_PI;
    }

    return ret;
}

inline double angDiff(double thetaD, double theta)
{
    double alpha = 0;
    Eigen::Vector2d nD, n;

    nD << cos(thetaD), sin(thetaD);
    n << cos(theta), sin(theta);

    double alphaAbs = acos(nD.transpose() * n);

    Eigen::Vector3d n3, nD3;

    n3 << n(0), n(1), 0;
    nD3 << nD(0), nD(1), 0;

    Eigen::Vector3d nC3;

    nC3 = n3.cross(nD3);

    if (nC3(2) > 0) {
        alpha = alphaAbs;
    } else {
        alpha = -alphaAbs;
    }

    return alpha;
}

inline Eigen::MatrixXd matrixPower(Eigen::MatrixXd& A, int exp)
{

    Eigen::MatrixXd result = Eigen::MatrixXd::Identity(A.rows(), A.cols());

    for (int i = 0; i < exp; ++i)
        result *= A;

    return result;
}

inline double sign(double x)
{
    if (x > 0)
        return +1;
    if (x < 0)
        return -1;
    return -1;
}

inline Eigen::Matrix3d rotx(double ax)
{

    Eigen::Matrix3d rx;
    rx.setZero();

    rx(0, 0) = 1;
    rx(0, 1) = 0;
    rx(0, 2) = 0;
    rx(1, 0) = 0;
    rx(1, 1) = cos(ax);
    rx(1, 2) = -sin(ax);
    rx(2, 0) = 0;
    rx(2, 1) = sin(ax);
    rx(2, 2) = cos(ax);

    return rx;
}

inline Eigen::Matrix3d roty(double ay)
{

    Eigen::Matrix3d ry;
    ry.setZero();

    ry(0, 0) = cos(ay);
    ry(0, 1) = 0;
    ry(0, 2) = sin(ay);
    ry(1, 0) = 0;
    ry(1, 1) = 1;
    ry(1, 2) = 0;
    ry(2, 0) = -sin(ay);
    ry(2, 1) = 0;
    ry(2, 2) = cos(ay);

    return ry;
}

inline Eigen::Matrix3d rotz(double az)
{

    Eigen::Matrix3d rz;
    rz.setZero();

    rz(0, 0) = cos(az);
    rz(0, 1) = -sin(az);
    rz(0, 2) = 0;
    rz(1, 0) = sin(az);
    rz(1, 1) = cos(az);
    rz(1, 2) = 0;
    rz(2, 0) = 0;
    rz(2, 1) = 0;
    rz(2, 2) = 1;

    return rz;
}

inline Eigen::Matrix3d rot(Eigen::Vector3d eul)
{

    Eigen::Matrix3d r;

    Eigen::Matrix3d rx = rotx(eul(0));
    Eigen::Matrix3d ry = roty(eul(1));
    Eigen::Matrix3d rz = rotz(eul(2));

    r = rx * ry * rz;

    return r;
}

inline Eigen::Matrix4d v2t(Eigen::VectorXd v)
{

    Eigen::Matrix4d m = Eigen::Matrix4d::Identity();

    Eigen::Vector3d eul = v.head(3);
    Eigen::Matrix3d r = rot(eul);

    m.block<3, 3>(0, 0) = r;
    m.block<3, 1>(0, 3) = v.tail(3);

    return m;
}

inline Eigen::VectorXd t2v(Eigen::Matrix4d m)
{
    Eigen::VectorXd v(6);

    float beta = atan2(m(0, 2), sqrt(pow(m(0, 0), 2) + pow(m(0, 1), 2)));
    float alpha = atan2(-m(1, 2) / cos(beta), m(2, 2) / cos(beta));
    float gamma = atan2(-m(0, 1) / cos(beta), m(0, 0) / cos(beta));

    v(0) = alpha;
    v(1) = beta;
    v(2) = gamma;
    v(3) = m(0, 3);
    v(4) = m(1, 3);
    v(5) = m(2, 3);

    return v;
}

// Express v2 in the frame of v1
inline Eigen::VectorXd vvRel(Eigen::VectorXd v2, Eigen::VectorXd v1)
{
    return t2v(v2t(v1).inverse() * v2t(v2));
}

struct Logger {
    std::ofstream file;
    Eigen::Vector3d* value;

    Logger(std::string name, Eigen::Vector3d* _value)
    {
        std::cout << "created LOG in " << realpath("../data/", NULL) + name << std::endl;
        file = std::ofstream(realpath("../data/", NULL) + ("/" + name), std::ofstream::out); //, std::ofstream::trunc);
        value = _value;
    }

    void log()
    {
        file << value->transpose() << std::endl;
    }
};

inline double angleSignedDistance(double a, double b)
{
    // float d = fabs(a - b) % 2.0*M_PI;
    double d = fabs(a - b);
    while (d > 2.0 * M_PI)
        d = d - 2.0 * M_PI;

    double r = 0.0;
    if (d > M_PI)
        r = 2.0 * M_PI - d;
    else
        r = d;
    double sign = 0.0;
    if ((a - b >= 0.0 && a - b <= M_PI) || (a - b <= -M_PI && a - b >= -2.0 * M_PI))
        sign = +1.0;
    else
        sign = -1.0;

    r = sign * r;
    return r;
}

inline Eigen::Vector3d angleSignedDistance(Eigen::Vector3d a, Eigen::Vector3d b)
{
    Eigen::Matrix3d Ra = rot(a);
    Eigen::Matrix3d Rb = rot(b);

    Eigen::Matrix3d Rdiff = Rb.transpose() * Ra;
    auto aa = Eigen::AngleAxisd(Rdiff);
    return aa.angle() * Ra * aa.axis();
}

inline Eigen::Vector3d getRPY(Eigen::MatrixXd rotMatrix)
{
    Eigen::Vector3d RPY;
    RPY << atan2(rotMatrix(2, 1), rotMatrix(2, 2)),
        atan2(-rotMatrix(2, 0), sqrt(rotMatrix(2, 1) * rotMatrix(2, 1) + rotMatrix(2, 2) * rotMatrix(2, 2))),
        atan2(rotMatrix(1, 0), rotMatrix(0, 0));

    return RPY;
}