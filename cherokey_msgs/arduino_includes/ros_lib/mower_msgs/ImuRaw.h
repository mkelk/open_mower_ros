#ifndef _ROS_mower_msgs_ImuRaw_h
#define _ROS_mower_msgs_ImuRaw_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mower_msgs
{

  class ImuRaw : public ros::Msg
  {
    public:
      typedef uint16_t _dt_type;
      _dt_type dt;
      typedef double _ax_type;
      _ax_type ax;
      typedef double _ay_type;
      _ay_type ay;
      typedef double _az_type;
      _az_type az;
      typedef double _gx_type;
      _gx_type gx;
      typedef double _gy_type;
      _gy_type gy;
      typedef double _gz_type;
      _gz_type gz;
      typedef double _mx_type;
      _mx_type mx;
      typedef double _my_type;
      _my_type my;
      typedef double _mz_type;
      _mz_type mz;

    ImuRaw():
      dt(0),
      ax(0),
      ay(0),
      az(0),
      gx(0),
      gy(0),
      gz(0),
      mx(0),
      my(0),
      mz(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->dt >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->dt >> (8 * 1)) & 0xFF;
      offset += sizeof(this->dt);
      union {
        double real;
        uint64_t base;
      } u_ax;
      u_ax.real = this->ax;
      *(outbuffer + offset + 0) = (u_ax.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ax.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ax.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ax.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_ax.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_ax.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_ax.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_ax.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->ax);
      union {
        double real;
        uint64_t base;
      } u_ay;
      u_ay.real = this->ay;
      *(outbuffer + offset + 0) = (u_ay.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ay.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ay.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ay.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_ay.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_ay.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_ay.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_ay.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->ay);
      union {
        double real;
        uint64_t base;
      } u_az;
      u_az.real = this->az;
      *(outbuffer + offset + 0) = (u_az.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_az.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_az.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_az.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_az.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_az.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_az.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_az.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->az);
      union {
        double real;
        uint64_t base;
      } u_gx;
      u_gx.real = this->gx;
      *(outbuffer + offset + 0) = (u_gx.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gx.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gx.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gx.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_gx.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_gx.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_gx.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_gx.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->gx);
      union {
        double real;
        uint64_t base;
      } u_gy;
      u_gy.real = this->gy;
      *(outbuffer + offset + 0) = (u_gy.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gy.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gy.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gy.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_gy.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_gy.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_gy.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_gy.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->gy);
      union {
        double real;
        uint64_t base;
      } u_gz;
      u_gz.real = this->gz;
      *(outbuffer + offset + 0) = (u_gz.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gz.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gz.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gz.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_gz.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_gz.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_gz.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_gz.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->gz);
      union {
        double real;
        uint64_t base;
      } u_mx;
      u_mx.real = this->mx;
      *(outbuffer + offset + 0) = (u_mx.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mx.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mx.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mx.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_mx.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_mx.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_mx.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_mx.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->mx);
      union {
        double real;
        uint64_t base;
      } u_my;
      u_my.real = this->my;
      *(outbuffer + offset + 0) = (u_my.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_my.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_my.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_my.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_my.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_my.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_my.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_my.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->my);
      union {
        double real;
        uint64_t base;
      } u_mz;
      u_mz.real = this->mz;
      *(outbuffer + offset + 0) = (u_mz.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mz.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mz.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mz.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_mz.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_mz.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_mz.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_mz.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->mz);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->dt =  ((uint16_t) (*(inbuffer + offset)));
      this->dt |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->dt);
      union {
        double real;
        uint64_t base;
      } u_ax;
      u_ax.base = 0;
      u_ax.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ax.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ax.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ax.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_ax.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_ax.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_ax.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_ax.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->ax = u_ax.real;
      offset += sizeof(this->ax);
      union {
        double real;
        uint64_t base;
      } u_ay;
      u_ay.base = 0;
      u_ay.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ay.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ay.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ay.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_ay.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_ay.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_ay.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_ay.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->ay = u_ay.real;
      offset += sizeof(this->ay);
      union {
        double real;
        uint64_t base;
      } u_az;
      u_az.base = 0;
      u_az.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_az.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_az.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_az.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_az.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_az.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_az.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_az.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->az = u_az.real;
      offset += sizeof(this->az);
      union {
        double real;
        uint64_t base;
      } u_gx;
      u_gx.base = 0;
      u_gx.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gx.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_gx.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_gx.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_gx.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_gx.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_gx.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_gx.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->gx = u_gx.real;
      offset += sizeof(this->gx);
      union {
        double real;
        uint64_t base;
      } u_gy;
      u_gy.base = 0;
      u_gy.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gy.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_gy.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_gy.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_gy.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_gy.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_gy.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_gy.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->gy = u_gy.real;
      offset += sizeof(this->gy);
      union {
        double real;
        uint64_t base;
      } u_gz;
      u_gz.base = 0;
      u_gz.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gz.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_gz.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_gz.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_gz.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_gz.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_gz.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_gz.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->gz = u_gz.real;
      offset += sizeof(this->gz);
      union {
        double real;
        uint64_t base;
      } u_mx;
      u_mx.base = 0;
      u_mx.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mx.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mx.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mx.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_mx.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_mx.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_mx.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_mx.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->mx = u_mx.real;
      offset += sizeof(this->mx);
      union {
        double real;
        uint64_t base;
      } u_my;
      u_my.base = 0;
      u_my.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_my.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_my.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_my.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_my.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_my.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_my.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_my.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->my = u_my.real;
      offset += sizeof(this->my);
      union {
        double real;
        uint64_t base;
      } u_mz;
      u_mz.base = 0;
      u_mz.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mz.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mz.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mz.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_mz.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_mz.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_mz.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_mz.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->mz = u_mz.real;
      offset += sizeof(this->mz);
     return offset;
    }

    virtual const char * getType() override { return "mower_msgs/ImuRaw"; };
    virtual const char * getMD5() override { return "0352b07cbe7d2a21d2573179fd87547d"; };

  };

}
#endif
