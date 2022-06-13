#ifndef _ROS_SERVICE_DeleteMowingAreaSrv_h
#define _ROS_SERVICE_DeleteMowingAreaSrv_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mower_map
{

static const char DELETEMOWINGAREASRV[] = "mower_map/DeleteMowingAreaSrv";

  class DeleteMowingAreaSrvRequest : public ros::Msg
  {
    public:
      typedef uint32_t _index_type;
      _index_type index;

    DeleteMowingAreaSrvRequest():
      index(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->index >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->index >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->index >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->index >> (8 * 3)) & 0xFF;
      offset += sizeof(this->index);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->index =  ((uint32_t) (*(inbuffer + offset)));
      this->index |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->index |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->index |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->index);
     return offset;
    }

    virtual const char * getType() override { return DELETEMOWINGAREASRV; };
    virtual const char * getMD5() override { return "ad7b979103dbd563a352ef5270716728"; };

  };

  class DeleteMowingAreaSrvResponse : public ros::Msg
  {
    public:

    DeleteMowingAreaSrvResponse()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
     return offset;
    }

    virtual const char * getType() override { return DELETEMOWINGAREASRV; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class DeleteMowingAreaSrv {
    public:
    typedef DeleteMowingAreaSrvRequest Request;
    typedef DeleteMowingAreaSrvResponse Response;
  };

}
#endif
