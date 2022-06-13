#ifndef _ROS_cherokey_msgs_Ticks_h
#define _ROS_cherokey_msgs_Ticks_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace cherokey_msgs
{

  class Ticks : public ros::Msg
  {
    public:
      typedef uint16_t _ticksLeft_type;
      _ticksLeft_type ticksLeft;
      typedef uint16_t _ticksRight_type;
      _ticksRight_type ticksRight;

    Ticks():
      ticksLeft(0),
      ticksRight(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->ticksLeft >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->ticksLeft >> (8 * 1)) & 0xFF;
      offset += sizeof(this->ticksLeft);
      *(outbuffer + offset + 0) = (this->ticksRight >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->ticksRight >> (8 * 1)) & 0xFF;
      offset += sizeof(this->ticksRight);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->ticksLeft =  ((uint16_t) (*(inbuffer + offset)));
      this->ticksLeft |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->ticksLeft);
      this->ticksRight =  ((uint16_t) (*(inbuffer + offset)));
      this->ticksRight |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->ticksRight);
     return offset;
    }

    virtual const char * getType() override { return "cherokey_msgs/Ticks"; };
    virtual const char * getMD5() override { return "1029027dd8f5b97b851a97557020f503"; };

  };

}
#endif
