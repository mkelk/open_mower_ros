#ifndef _ROS_grid_map_msgs_GridMapInfo_h
#define _ROS_grid_map_msgs_GridMapInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"

namespace grid_map_msgs
{

  class GridMapInfo : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef double _resolution_type;
      _resolution_type resolution;
      typedef double _length_x_type;
      _length_x_type length_x;
      typedef double _length_y_type;
      _length_y_type length_y;
      typedef geometry_msgs::Pose _pose_type;
      _pose_type pose;

    GridMapInfo():
      header(),
      resolution(0),
      length_x(0),
      length_y(0),
      pose()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_resolution;
      u_resolution.real = this->resolution;
      *(outbuffer + offset + 0) = (u_resolution.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_resolution.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_resolution.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_resolution.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_resolution.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_resolution.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_resolution.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_resolution.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->resolution);
      union {
        double real;
        uint64_t base;
      } u_length_x;
      u_length_x.real = this->length_x;
      *(outbuffer + offset + 0) = (u_length_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_length_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_length_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_length_x.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_length_x.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_length_x.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_length_x.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_length_x.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->length_x);
      union {
        double real;
        uint64_t base;
      } u_length_y;
      u_length_y.real = this->length_y;
      *(outbuffer + offset + 0) = (u_length_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_length_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_length_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_length_y.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_length_y.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_length_y.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_length_y.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_length_y.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->length_y);
      offset += this->pose.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_resolution;
      u_resolution.base = 0;
      u_resolution.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_resolution.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_resolution.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_resolution.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_resolution.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_resolution.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_resolution.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_resolution.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->resolution = u_resolution.real;
      offset += sizeof(this->resolution);
      union {
        double real;
        uint64_t base;
      } u_length_x;
      u_length_x.base = 0;
      u_length_x.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_length_x.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_length_x.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_length_x.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_length_x.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_length_x.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_length_x.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_length_x.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->length_x = u_length_x.real;
      offset += sizeof(this->length_x);
      union {
        double real;
        uint64_t base;
      } u_length_y;
      u_length_y.base = 0;
      u_length_y.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_length_y.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_length_y.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_length_y.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_length_y.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_length_y.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_length_y.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_length_y.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->length_y = u_length_y.real;
      offset += sizeof(this->length_y);
      offset += this->pose.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "grid_map_msgs/GridMapInfo"; };
    virtual const char * getMD5() override { return "43ee5430e1c253682111cb6bedac0ef9"; };

  };

}
#endif
