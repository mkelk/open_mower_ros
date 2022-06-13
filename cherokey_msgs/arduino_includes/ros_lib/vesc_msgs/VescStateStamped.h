#ifndef _ROS_vesc_msgs_VescStateStamped_h
#define _ROS_vesc_msgs_VescStateStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "vesc_msgs/VescState.h"

namespace vesc_msgs
{

  class VescStateStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef vesc_msgs::VescState _state_type;
      _state_type state;

    VescStateStamped():
      header(),
      state()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->state.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->state.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "vesc_msgs/VescStateStamped"; };
    virtual const char * getMD5() override { return "31b68d13b845c7f1f9cb66a9bdf89a55"; };

  };

}
#endif
