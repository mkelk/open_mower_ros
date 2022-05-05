#ifndef _ROS_cherokey_msgs_WheelState_h
#define _ROS_cherokey_msgs_WheelState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace cherokey_msgs
{

  class WheelState : public ros::Msg
  {
    public:
      typedef uint16_t _speedLeft_type;
      _speedLeft_type speedLeft;
      typedef int8_t _dirLeft_type;
      _dirLeft_type dirLeft;
      typedef uint16_t _speedRight_type;
      _speedRight_type speedRight;
      typedef int8_t _dirRight_type;
      _dirRight_type dirRight;

    WheelState():
      speedLeft(0),
      dirLeft(0),
      speedRight(0),
      dirRight(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->speedLeft >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->speedLeft >> (8 * 1)) & 0xFF;
      offset += sizeof(this->speedLeft);
      union {
        int8_t real;
        uint8_t base;
      } u_dirLeft;
      u_dirLeft.real = this->dirLeft;
      *(outbuffer + offset + 0) = (u_dirLeft.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->dirLeft);
      *(outbuffer + offset + 0) = (this->speedRight >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->speedRight >> (8 * 1)) & 0xFF;
      offset += sizeof(this->speedRight);
      union {
        int8_t real;
        uint8_t base;
      } u_dirRight;
      u_dirRight.real = this->dirRight;
      *(outbuffer + offset + 0) = (u_dirRight.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->dirRight);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->speedLeft =  ((uint16_t) (*(inbuffer + offset)));
      this->speedLeft |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->speedLeft);
      union {
        int8_t real;
        uint8_t base;
      } u_dirLeft;
      u_dirLeft.base = 0;
      u_dirLeft.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->dirLeft = u_dirLeft.real;
      offset += sizeof(this->dirLeft);
      this->speedRight =  ((uint16_t) (*(inbuffer + offset)));
      this->speedRight |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->speedRight);
      union {
        int8_t real;
        uint8_t base;
      } u_dirRight;
      u_dirRight.base = 0;
      u_dirRight.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->dirRight = u_dirRight.real;
      offset += sizeof(this->dirRight);
     return offset;
    }

    virtual const char * getType() override { return "cherokey_msgs/WheelState"; };
    virtual const char * getMD5() override { return "3504f04ace09a3b09ec8a6b7d0833aba"; };

  };

}
#endif
