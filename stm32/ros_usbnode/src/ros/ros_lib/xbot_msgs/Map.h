#ifndef _ROS_xbot_msgs_Map_h
#define _ROS_xbot_msgs_Map_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "xbot_msgs/MapArea.h"

namespace xbot_msgs
{

  class Map : public ros::Msg
  {
    public:
      typedef float _mapWidth_type;
      _mapWidth_type mapWidth;
      typedef float _mapHeight_type;
      _mapHeight_type mapHeight;
      typedef float _mapCenterX_type;
      _mapCenterX_type mapCenterX;
      typedef float _mapCenterY_type;
      _mapCenterY_type mapCenterY;
      uint32_t navigationAreas_length;
      typedef xbot_msgs::MapArea _navigationAreas_type;
      _navigationAreas_type st_navigationAreas;
      _navigationAreas_type * navigationAreas;
      uint32_t workingArea_length;
      typedef xbot_msgs::MapArea _workingArea_type;
      _workingArea_type st_workingArea;
      _workingArea_type * workingArea;
      typedef float _dockX_type;
      _dockX_type dockX;
      typedef float _dockY_type;
      _dockY_type dockY;
      typedef float _dockHeading_type;
      _dockHeading_type dockHeading;

    Map():
      mapWidth(0),
      mapHeight(0),
      mapCenterX(0),
      mapCenterY(0),
      navigationAreas_length(0), st_navigationAreas(), navigationAreas(nullptr),
      workingArea_length(0), st_workingArea(), workingArea(nullptr),
      dockX(0),
      dockY(0),
      dockHeading(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->mapWidth);
      offset += serializeAvrFloat64(outbuffer + offset, this->mapHeight);
      offset += serializeAvrFloat64(outbuffer + offset, this->mapCenterX);
      offset += serializeAvrFloat64(outbuffer + offset, this->mapCenterY);
      *(outbuffer + offset + 0) = (this->navigationAreas_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->navigationAreas_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->navigationAreas_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->navigationAreas_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->navigationAreas_length);
      for( uint32_t i = 0; i < navigationAreas_length; i++){
      offset += this->navigationAreas[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->workingArea_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->workingArea_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->workingArea_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->workingArea_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->workingArea_length);
      for( uint32_t i = 0; i < workingArea_length; i++){
      offset += this->workingArea[i].serialize(outbuffer + offset);
      }
      offset += serializeAvrFloat64(outbuffer + offset, this->dockX);
      offset += serializeAvrFloat64(outbuffer + offset, this->dockY);
      offset += serializeAvrFloat64(outbuffer + offset, this->dockHeading);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->mapWidth));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->mapHeight));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->mapCenterX));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->mapCenterY));
      uint32_t navigationAreas_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      navigationAreas_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      navigationAreas_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      navigationAreas_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->navigationAreas_length);
      if(navigationAreas_lengthT > navigationAreas_length)
        this->navigationAreas = (xbot_msgs::MapArea*)realloc(this->navigationAreas, navigationAreas_lengthT * sizeof(xbot_msgs::MapArea));
      navigationAreas_length = navigationAreas_lengthT;
      for( uint32_t i = 0; i < navigationAreas_length; i++){
      offset += this->st_navigationAreas.deserialize(inbuffer + offset);
        memcpy( &(this->navigationAreas[i]), &(this->st_navigationAreas), sizeof(xbot_msgs::MapArea));
      }
      uint32_t workingArea_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      workingArea_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      workingArea_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      workingArea_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->workingArea_length);
      if(workingArea_lengthT > workingArea_length)
        this->workingArea = (xbot_msgs::MapArea*)realloc(this->workingArea, workingArea_lengthT * sizeof(xbot_msgs::MapArea));
      workingArea_length = workingArea_lengthT;
      for( uint32_t i = 0; i < workingArea_length; i++){
      offset += this->st_workingArea.deserialize(inbuffer + offset);
        memcpy( &(this->workingArea[i]), &(this->st_workingArea), sizeof(xbot_msgs::MapArea));
      }
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->dockX));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->dockY));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->dockHeading));
     return offset;
    }

    virtual const char * getType() override { return "xbot_msgs/Map"; };
    virtual const char * getMD5() override { return "0defc0a41c9dfac41eff99d1ae21f375"; };

  };

}
#endif
