#pragma once

#include <vector>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

struct Point2D {
  double x, y;
  Point2D operator+(const Point2D& other) const;
  Point2D operator*(double scalar) const;
};

class CatmullRomSpline {
public:
  CatmullRomSpline(const Point2D& p0, const Point2D& p1,
                   const Point2D& p2, const Point2D& p3);
  std::vector<Point2D> computeSpline(double resolution = 0.1);

private:
  Point2D p0_, p1_, p2_, p3_;
  double distance(const Point2D& a, const Point2D& b) const;
};
