#ifndef _ROS_SERVICE_AppendMapSrv_h
#define _ROS_SERVICE_AppendMapSrv_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mower_map
{

static const char APPENDMAPSRV[] = "mower_map/AppendMapSrv";

  class AppendMapSrvRequest : public ros::Msg
  {
    public:
      typedef const char* _bagfile_type;
      _bagfile_type bagfile;

    AppendMapSrvRequest():
      bagfile("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_bagfile = strlen(this->bagfile);
      varToArr(outbuffer + offset, length_bagfile);
      offset += 4;
      memcpy(outbuffer + offset, this->bagfile, length_bagfile);
      offset += length_bagfile;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_bagfile;
      arrToVar(length_bagfile, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_bagfile; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_bagfile-1]=0;
      this->bagfile = (char *)(inbuffer + offset-1);
      offset += length_bagfile;
     return offset;
    }

    virtual const char * getType() override { return APPENDMAPSRV; };
    virtual const char * getMD5() override { return "a935fd4be1d8c2e1777172fa15665bc9"; };

  };

  class AppendMapSrvResponse : public ros::Msg
  {
    public:

    AppendMapSrvResponse()
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

    virtual const char * getType() override { return APPENDMAPSRV; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class AppendMapSrv {
    public:
    typedef AppendMapSrvRequest Request;
    typedef AppendMapSrvResponse Response;
  };

}
#endif
