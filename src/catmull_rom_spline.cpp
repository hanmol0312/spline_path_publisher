#include "spline_generator/catmull_rom_spline.hpp"
#include <cmath>  // For pow() and sqrt

// ==========================
// Operator Overloading for Point2D
// ==========================

// This lets me directly add two Point2D structs using the + operator
Point2D Point2D::operator+(const Point2D& other) const {
  return {x + other.x, y + other.y};  // Add x and y components separately
}

// This lets me scale a Point2D using * operator with a scalar
Point2D Point2D::operator*(double scalar) const {
  return {x * scalar, y * scalar};  // Multiply both x and y by the scalar
}

// ==========================
// Catmull-Rom Spline Constructor
// ==========================

// Constructor: Just saves the control points for later use in computeSpline()
CatmullRomSpline::CatmullRomSpline(const Point2D& p0, const Point2D& p1,
                                   const Point2D& p2, const Point2D& p3)
    : p0_(p0), p1_(p1), p2_(p2), p3_(p3) {}

// ==========================
// Helper Function: Euclidean Distance
// ==========================

// Calculates straight-line (Euclidean) distance between two points
double CatmullRomSpline::distance(const Point2D& a, const Point2D& b) const {
  return std::pow(std::pow(b.x - a.x, 2) + std::pow(b.y - a.y, 2), 0.5);  // √((x2 - x1)^2 + (y2 - y1)^2)
}

// ==========================
// Main Function: Generate Spline Points
// ==========================

// Actually computes the points along the spline based on resolution
std::vector<Point2D> CatmullRomSpline::computeSpline(double resolution) {
  std::vector<Point2D> result;
  double alpha = 0.5; // Using centripetal version (avoids self-intersections)

  // Calculate t values based on distances (used for parameterizing points)
  double t0 = 0.0;
  double t1 = t0 + std::pow(distance(p0_, p1_), alpha);
  double t2 = t1 + std::pow(distance(p1_, p2_), alpha);
  double t3 = t2 + std::pow(distance(p2_, p3_), alpha);

  // Loop to interpolate points between p1 and p2
  for (double t = t1; t < t2; t += resolution) {
    // First level of interpolation
    Point2D A1 = p0_ * ((t1 - t) / (t1 - t0)) + p1_ * ((t - t0) / (t1 - t0));  // Effective use of operator overloading
    Point2D A2 = p1_ * ((t2 - t) / (t2 - t1)) + p2_ * ((t - t1) / (t2 - t1));
    Point2D A3 = p2_ * ((t3 - t) / (t3 - t2)) + p3_ * ((t - t2) / (t3 - t2));

    // Second level of interpolation
    Point2D B1 = A1 * ((t2 - t) / (t2 - t0)) + A2 * ((t - t0) / (t2 - t0));
    Point2D B2 = A2 * ((t3 - t) / (t3 - t1)) + A3 * ((t - t1) / (t3 - t1));

    // Final level of interpolation: this is the actual spline point
    Point2D C = B1 * ((t2 - t) / (t2 - t1)) + B2 * ((t - t1) / (t2 - t1));

    // Add it to the final path
    result.push_back(C);
  }

  return result;  // Return the full list of interpolated points
}
