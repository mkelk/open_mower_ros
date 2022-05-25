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
      typedef int16_t _ticksLeft_type;
      _ticksLeft_type ticksLeft;
      typedef int16_t _ticksRight_type;
      _ticksRight_type ticksRight;

    Ticks():
      ticksLeft(0),
      ticksRight(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_ticksLeft;
      u_ticksLeft.real = this->ticksLeft;
      *(outbuffer + offset + 0) = (u_ticksLeft.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ticksLeft.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->ticksLeft);
      union {
        int16_t real;
        uint16_t base;
      } u_ticksRight;
      u_ticksRight.real = this->ticksRight;
      *(outbuffer + offset + 0) = (u_ticksRight.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ticksRight.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->ticksRight);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_ticksLeft;
      u_ticksLeft.base = 0;
      u_ticksLeft.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ticksLeft.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->ticksLeft = u_ticksLeft.real;
      offset += sizeof(this->ticksLeft);
      union {
        int16_t real;
        uint16_t base;
      } u_ticksRight;
      u_ticksRight.base = 0;
      u_ticksRight.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ticksRight.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->ticksRight = u_ticksRight.real;
      offset += sizeof(this->ticksRight);
     return offset;
    }

    virtual const char * getType() override { return "cherokey_msgs/Ticks"; };
    virtual const char * getMD5() override { return "c5d21d47a2e767cbaa216ebabe2683aa"; };

  };

}
#endif
