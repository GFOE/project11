/**
@file utils.h

Utilities used extensively in project11

*/

#ifndef P11_UTILS_H
#define P11_UTILS_H

#include <tf2/utils.h>
#include "gz4d_geo.h"

namespace project11
{
  /** @brief Uses the tf2::getYaw() utils function and converts to degrees, NED.  
   * Does not appear that this is used anywhere in Project11.
   * @param a An orientation messages, e.g., http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Quaternion.html
   * @see tf2:getYaw()
   * @return Heading in degrees, NED.
  */
  template <typename A> double quaternionToHeadingDegrees(const A& a)
  {
    return 90.0-180.0*tf2::getYaw(a)/M_PI;
  }

  inline double speedOverGround(const geometry_msgs::Vector3 & v)
  {
    tf2::Vector3 v3;
    tf2::fromMsg(v, v3);
    return v3.length();
  }

  /** @brief Passthrough typdef to gz4d 
  * @see gz4d_geo.h
  */
  typedef gz4d::GeoPointLatLongDegrees LatLongDegrees;
  
  typedef gz4d::GeoPointECEF ECEF;

  /** @brief Passthrough typeded to assign utils::Point as gz4d::Point with template datatype = double.
  * Really makes it hard to follow the code
  */
  typedef gz4d::Point<double> Point;
  typedef gz4d::LocalENU ENUFrame;
  
  // angle types that do not wrap
  typedef gz4d::Angle<double, gz4d::pu::Degree, gz4d::rt::Unclamped> AngleDegrees;
  typedef gz4d::Angle<double, gz4d::pu::Radian, gz4d::rt::Unclamped> AngleRadians;
  
  // angle types that wrap at +/- half a circle
  typedef gz4d::Angle<double, gz4d::pu::Degree, gz4d::rt::ZeroCenteredPeriod> AngleDegreesZeroCentered;
  typedef gz4d::Angle<double, gz4d::pu::Radian, gz4d::rt::ZeroCenteredPeriod> AngleRadiansZeroCentered;
  
  // angle types that wrap at 0 and full circle
  typedef gz4d::Angle<double, gz4d::pu::Degree, gz4d::rt::PositivePeriod> AngleDegreesPositive;
  typedef gz4d::Angle<double, gz4d::pu::Radian, gz4d::rt::PositivePeriod> AngleRadiansPositive;
  
  typedef gz4d::geo::WGS84::Ellipsoid WGS84;

  /** @brief Assigns LatLongDegrees object lat/lon/alt fields from NavSatFix ROS message 
  * @param a The ROS message containing latitude, longitude and altitude attributes.
  * @param b utils::LatLongDegrees object with data from the ROS message.
  */
  template <typename A> void fromMsg(const A& a, LatLongDegrees &b)
  {
    b.latitude() = a.latitude;
    b.longitude() = a.longitude;
    b.altitude() = a.altitude;
  }

  template <typename B> void toMsg(const LatLongDegrees &a, B &b)
  {
    b.latitude = a.latitude();
    b.longitude = a.longitude();
    b.altitude = a.altitude();
  }

  /** @brief Assigns utils::Point object x/y/z fields from ROS Point message.
  * @param a The source ROS geometry_msgs::Point (or other message with .x, .y and .y attributes).
  * @param b The destination utils::Point object containing the .x, .y and .z data as elements of the Point vector.
  */
  template <typename A> void fromMsg(const A& a, Point &b)
  {
    b = Point(a.x, a.y, a.z);
  }
  
  template <typename B> void toMsg(const Point& a, B &b)
  {
    b.x = a[0];
    b.y = a[1];
    b.z = a[2];
  }
}

std::ostream& operator<< (std::ostream &out, const project11::LatLongDegrees &p);
std::ostream& operator<< (std::ostream &out, const project11::ECEF &p);
std::ostream& operator<< (std::ostream &out, const project11::AngleDegrees &p);
std::ostream& operator<< (std::ostream &out, const project11::AngleRadians &p);

#endif
