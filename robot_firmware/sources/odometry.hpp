/**
* @file odometry.hpp
* @brief Odometry class
*/
#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP


typedef struct
{
    float x, y, dir;
} OdometryPosition_t;


class Odometry
{
public:
    static void Init();
    static void Reset();
    static OdometryPosition_t* GetPosition();
};


#endif /* ODOMETRY_HPP */
