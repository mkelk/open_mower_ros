#ifndef _ROS_ublox_msgs_NavHPPOSECEF_h
#define _ROS_ublox_msgs_NavHPPOSECEF_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class NavHPPOSECEF : public ros::Msg
  {
    public:
      typedef uint8_t _version_type;
      _version_type version;
      uint8_t reserved0[3];
      typedef uint32_t _iTOW_type;
      _iTOW_type iTOW;
      typedef int32_t _ecefX_type;
      _ecefX_type ecefX;
      typedef int32_t _ecefY_type;
      _ecefY_type ecefY;
      typedef int32_t _ecefZ_type;
      _ecefZ_type ecefZ;
      typedef int8_t _ecefXHp_type;
      _ecefXHp_type ecefXHp;
      typedef int8_t _ecefYHp_type;
      _ecefYHp_type ecefYHp;
      typedef int8_t _ecefZHp_type;
      _ecefZHp_type ecefZHp;
      typedef uint8_t _flags_type;
      _flags_type flags;
      typedef uint32_t _pAcc_type;
      _pAcc_type pAcc;
      enum { CLASS_ID =  1 };
      enum { MESSAGE_ID =  19 };

    NavHPPOSECEF():
      version(0),
      reserved0(),
      iTOW(0),
      ecefX(0),
      ecefY(0),
      ecefZ(0),
      ecefXHp(0),
      ecefYHp(0),
      ecefZHp(0),
      flags(0),
      pAcc(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->version >> (8 * 0)) & 0xFF;
      offset += sizeof(this->version);
      for( uint32_t i = 0; i < 3; i++){
      *(outbuffer + offset + 0) = (this->reserved0[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved0[i]);
      }
      *(outbuffer + offset + 0) = (this->iTOW >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->iTOW >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->iTOW >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->iTOW >> (8 * 3)) & 0xFF;
      offset += sizeof(this->iTOW);
      union {
        int32_t real;
        uint32_t base;
      } u_ecefX;
      u_ecefX.real = this->ecefX;
      *(outbuffer + offset + 0) = (u_ecefX.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ecefX.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ecefX.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ecefX.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ecefX);
      union {
        int32_t real;
        uint32_t base;
      } u_ecefY;
      u_ecefY.real = this->ecefY;
      *(outbuffer + offset + 0) = (u_ecefY.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ecefY.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ecefY.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ecefY.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ecefY);
      union {
        int32_t real;
        uint32_t base;
      } u_ecefZ;
      u_ecefZ.real = this->ecefZ;
      *(outbuffer + offset + 0) = (u_ecefZ.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ecefZ.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ecefZ.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ecefZ.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ecefZ);
      union {
        int8_t real;
        uint8_t base;
      } u_ecefXHp;
      u_ecefXHp.real = this->ecefXHp;
      *(outbuffer + offset + 0) = (u_ecefXHp.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ecefXHp);
      union {
        int8_t real;
        uint8_t base;
      } u_ecefYHp;
      u_ecefYHp.real = this->ecefYHp;
      *(outbuffer + offset + 0) = (u_ecefYHp.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ecefYHp);
      union {
        int8_t real;
        uint8_t base;
      } u_ecefZHp;
      u_ecefZHp.real = this->ecefZHp;
      *(outbuffer + offset + 0) = (u_ecefZHp.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ecefZHp);
      *(outbuffer + offset + 0) = (this->flags >> (8 * 0)) & 0xFF;
      offset += sizeof(this->flags);
      *(outbuffer + offset + 0) = (this->pAcc >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pAcc >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->pAcc >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->pAcc >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pAcc);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->version =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->version);
      for( uint32_t i = 0; i < 3; i++){
      this->reserved0[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved0[i]);
      }
      this->iTOW =  ((uint32_t) (*(inbuffer + offset)));
      this->iTOW |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->iTOW |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->iTOW |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->iTOW);
      union {
        int32_t real;
        uint32_t base;
      } u_ecefX;
      u_ecefX.base = 0;
      u_ecefX.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ecefX.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ecefX.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ecefX.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ecefX = u_ecefX.real;
      offset += sizeof(this->ecefX);
      union {
        int32_t real;
        uint32_t base;
      } u_ecefY;
      u_ecefY.base = 0;
      u_ecefY.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ecefY.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ecefY.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ecefY.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ecefY = u_ecefY.real;
      offset += sizeof(this->ecefY);
      union {
        int32_t real;
        uint32_t base;
      } u_ecefZ;
      u_ecefZ.base = 0;
      u_ecefZ.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ecefZ.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ecefZ.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ecefZ.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ecefZ = u_ecefZ.real;
      offset += sizeof(this->ecefZ);
      union {
        int8_t real;
        uint8_t base;
      } u_ecefXHp;
      u_ecefXHp.base = 0;
      u_ecefXHp.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ecefXHp = u_ecefXHp.real;
      offset += sizeof(this->ecefXHp);
      union {
        int8_t real;
        uint8_t base;
      } u_ecefYHp;
      u_ecefYHp.base = 0;
      u_ecefYHp.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ecefYHp = u_ecefYHp.real;
      offset += sizeof(this->ecefYHp);
      union {
        int8_t real;
        uint8_t base;
      } u_ecefZHp;
      u_ecefZHp.base = 0;
      u_ecefZHp.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ecefZHp = u_ecefZHp.real;
      offset += sizeof(this->ecefZHp);
      this->flags =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->flags);
      this->pAcc =  ((uint32_t) (*(inbuffer + offset)));
      this->pAcc |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->pAcc |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->pAcc |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->pAcc);
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/NavHPPOSECEF"; };
    virtual const char * getMD5() override { return "41fbf0937e53f84ca89afe3287f85e50"; };

  };

}
#endif
