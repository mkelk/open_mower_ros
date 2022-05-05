#ifndef _ROS_mower_map_MapAreas_h
#define _ROS_mower_map_MapAreas_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "mower_map/MapArea.h"

namespace mower_map
{

  class MapAreas : public ros::Msg
  {
    public:
      typedef double _mapWidth_type;
      _mapWidth_type mapWidth;
      typedef double _mapHeight_type;
      _mapHeight_type mapHeight;
      typedef double _mapCenterX_type;
      _mapCenterX_type mapCenterX;
      typedef double _mapCenterY_type;
      _mapCenterY_type mapCenterY;
      uint32_t navigationAreas_length;
      typedef mower_map::MapArea _navigationAreas_type;
      _navigationAreas_type st_navigationAreas;
      _navigationAreas_type * navigationAreas;
      uint32_t mowingAreas_length;
      typedef mower_map::MapArea _mowingAreas_type;
      _mowingAreas_type st_mowingAreas;
      _mowingAreas_type * mowingAreas;

    MapAreas():
      mapWidth(0),
      mapHeight(0),
      mapCenterX(0),
      mapCenterY(0),
      navigationAreas_length(0), st_navigationAreas(), navigationAreas(nullptr),
      mowingAreas_length(0), st_mowingAreas(), mowingAreas(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_mapWidth;
      u_mapWidth.real = this->mapWidth;
      *(outbuffer + offset + 0) = (u_mapWidth.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mapWidth.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mapWidth.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mapWidth.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_mapWidth.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_mapWidth.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_mapWidth.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_mapWidth.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->mapWidth);
      union {
        double real;
        uint64_t base;
      } u_mapHeight;
      u_mapHeight.real = this->mapHeight;
      *(outbuffer + offset + 0) = (u_mapHeight.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mapHeight.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mapHeight.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mapHeight.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_mapHeight.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_mapHeight.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_mapHeight.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_mapHeight.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->mapHeight);
      union {
        double real;
        uint64_t base;
      } u_mapCenterX;
      u_mapCenterX.real = this->mapCenterX;
      *(outbuffer + offset + 0) = (u_mapCenterX.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mapCenterX.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mapCenterX.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mapCenterX.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_mapCenterX.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_mapCenterX.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_mapCenterX.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_mapCenterX.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->mapCenterX);
      union {
        double real;
        uint64_t base;
      } u_mapCenterY;
      u_mapCenterY.real = this->mapCenterY;
      *(outbuffer + offset + 0) = (u_mapCenterY.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mapCenterY.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mapCenterY.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mapCenterY.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_mapCenterY.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_mapCenterY.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_mapCenterY.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_mapCenterY.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->mapCenterY);
      *(outbuffer + offset + 0) = (this->navigationAreas_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->navigationAreas_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->navigationAreas_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->navigationAreas_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->navigationAreas_length);
      for( uint32_t i = 0; i < navigationAreas_length; i++){
      offset += this->navigationAreas[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->mowingAreas_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->mowingAreas_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->mowingAreas_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->mowingAreas_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mowingAreas_length);
      for( uint32_t i = 0; i < mowingAreas_length; i++){
      offset += this->mowingAreas[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_mapWidth;
      u_mapWidth.base = 0;
      u_mapWidth.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mapWidth.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mapWidth.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mapWidth.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_mapWidth.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_mapWidth.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_mapWidth.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_mapWidth.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->mapWidth = u_mapWidth.real;
      offset += sizeof(this->mapWidth);
      union {
        double real;
        uint64_t base;
      } u_mapHeight;
      u_mapHeight.base = 0;
      u_mapHeight.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mapHeight.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mapHeight.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mapHeight.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_mapHeight.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_mapHeight.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_mapHeight.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_mapHeight.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->mapHeight = u_mapHeight.real;
      offset += sizeof(this->mapHeight);
      union {
        double real;
        uint64_t base;
      } u_mapCenterX;
      u_mapCenterX.base = 0;
      u_mapCenterX.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mapCenterX.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mapCenterX.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mapCenterX.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_mapCenterX.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_mapCenterX.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_mapCenterX.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_mapCenterX.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->mapCenterX = u_mapCenterX.real;
      offset += sizeof(this->mapCenterX);
      union {
        double real;
        uint64_t base;
      } u_mapCenterY;
      u_mapCenterY.base = 0;
      u_mapCenterY.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mapCenterY.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mapCenterY.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mapCenterY.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_mapCenterY.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_mapCenterY.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_mapCenterY.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_mapCenterY.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->mapCenterY = u_mapCenterY.real;
      offset += sizeof(this->mapCenterY);
      uint32_t navigationAreas_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      navigationAreas_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      navigationAreas_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      navigationAreas_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->navigationAreas_length);
      if(navigationAreas_lengthT > navigationAreas_length)
        this->navigationAreas = (mower_map::MapArea*)realloc(this->navigationAreas, navigationAreas_lengthT * sizeof(mower_map::MapArea));
      navigationAreas_length = navigationAreas_lengthT;
      for( uint32_t i = 0; i < navigationAreas_length; i++){
      offset += this->st_navigationAreas.deserialize(inbuffer + offset);
        memcpy( &(this->navigationAreas[i]), &(this->st_navigationAreas), sizeof(mower_map::MapArea));
      }
      uint32_t mowingAreas_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      mowingAreas_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      mowingAreas_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      mowingAreas_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->mowingAreas_length);
      if(mowingAreas_lengthT > mowingAreas_length)
        this->mowingAreas = (mower_map::MapArea*)realloc(this->mowingAreas, mowingAreas_lengthT * sizeof(mower_map::MapArea));
      mowingAreas_length = mowingAreas_lengthT;
      for( uint32_t i = 0; i < mowingAreas_length; i++){
      offset += this->st_mowingAreas.deserialize(inbuffer + offset);
        memcpy( &(this->mowingAreas[i]), &(this->st_mowingAreas), sizeof(mower_map::MapArea));
      }
     return offset;
    }

    virtual const char * getType() override { return "mower_map/MapAreas"; };
    virtual const char * getMD5() override { return "d3c611e824aac319a11694c960ef24a8"; };

  };

}
#endif
