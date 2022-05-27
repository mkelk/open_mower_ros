#ifndef _ROS_SERVICE_PlanPath_h
#define _ROS_SERVICE_PlanPath_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Polygon.h"
#include "nav_msgs/Path.h"

namespace slic3r_coverage_planner
{

static const char PLANPATH[] = "slic3r_coverage_planner/PlanPath";

  class PlanPathRequest : public ros::Msg
  {
    public:
      typedef uint8_t _fill_type_type;
      _fill_type_type fill_type;
      typedef double _angle_type;
      _angle_type angle;
      typedef double _distance_type;
      _distance_type distance;
      typedef uint8_t _outline_count_type;
      _outline_count_type outline_count;
      typedef geometry_msgs::Polygon _outline_type;
      _outline_type outline;
      uint32_t holes_length;
      typedef geometry_msgs::Polygon _holes_type;
      _holes_type st_holes;
      _holes_type * holes;
      enum { FILL_LINEAR = 0 };
      enum { FILL_CONCENTRIC = 1 };

    PlanPathRequest():
      fill_type(0),
      angle(0),
      distance(0),
      outline_count(0),
      outline(),
      holes_length(0), st_holes(), holes(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->fill_type >> (8 * 0)) & 0xFF;
      offset += sizeof(this->fill_type);
      union {
        double real;
        uint64_t base;
      } u_angle;
      u_angle.real = this->angle;
      *(outbuffer + offset + 0) = (u_angle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angle.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_angle.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_angle.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_angle.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_angle.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->angle);
      union {
        double real;
        uint64_t base;
      } u_distance;
      u_distance.real = this->distance;
      *(outbuffer + offset + 0) = (u_distance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_distance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_distance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_distance.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_distance.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_distance.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_distance.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_distance.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->distance);
      *(outbuffer + offset + 0) = (this->outline_count >> (8 * 0)) & 0xFF;
      offset += sizeof(this->outline_count);
      offset += this->outline.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->holes_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->holes_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->holes_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->holes_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->holes_length);
      for( uint32_t i = 0; i < holes_length; i++){
      offset += this->holes[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->fill_type =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->fill_type);
      union {
        double real;
        uint64_t base;
      } u_angle;
      u_angle.base = 0;
      u_angle.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angle.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_angle.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_angle.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_angle.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_angle.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_angle.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_angle.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->angle = u_angle.real;
      offset += sizeof(this->angle);
      union {
        double real;
        uint64_t base;
      } u_distance;
      u_distance.base = 0;
      u_distance.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_distance.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_distance.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_distance.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_distance.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_distance.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_distance.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_distance.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->distance = u_distance.real;
      offset += sizeof(this->distance);
      this->outline_count =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->outline_count);
      offset += this->outline.deserialize(inbuffer + offset);
      uint32_t holes_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      holes_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      holes_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      holes_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->holes_length);
      if(holes_lengthT > holes_length)
        this->holes = (geometry_msgs::Polygon*)realloc(this->holes, holes_lengthT * sizeof(geometry_msgs::Polygon));
      holes_length = holes_lengthT;
      for( uint32_t i = 0; i < holes_length; i++){
      offset += this->st_holes.deserialize(inbuffer + offset);
        memcpy( &(this->holes[i]), &(this->st_holes), sizeof(geometry_msgs::Polygon));
      }
     return offset;
    }

    virtual const char * getType() override { return PLANPATH; };
    virtual const char * getMD5() override { return "0b4d7f95ba027cf2d2a92ce9a41d5d10"; };

  };

  class PlanPathResponse : public ros::Msg
  {
    public:
      uint32_t paths_length;
      typedef nav_msgs::Path _paths_type;
      _paths_type st_paths;
      _paths_type * paths;

    PlanPathResponse():
      paths_length(0), st_paths(), paths(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->paths_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->paths_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->paths_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->paths_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->paths_length);
      for( uint32_t i = 0; i < paths_length; i++){
      offset += this->paths[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t paths_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      paths_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      paths_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      paths_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->paths_length);
      if(paths_lengthT > paths_length)
        this->paths = (nav_msgs::Path*)realloc(this->paths, paths_lengthT * sizeof(nav_msgs::Path));
      paths_length = paths_lengthT;
      for( uint32_t i = 0; i < paths_length; i++){
      offset += this->st_paths.deserialize(inbuffer + offset);
        memcpy( &(this->paths[i]), &(this->st_paths), sizeof(nav_msgs::Path));
      }
     return offset;
    }

    virtual const char * getType() override { return PLANPATH; };
    virtual const char * getMD5() override { return "4ac7ec67d8c4222b3a71671051b1fa81"; };

  };

  class PlanPath {
    public:
    typedef PlanPathRequest Request;
    typedef PlanPathResponse Response;
  };

}
#endif
