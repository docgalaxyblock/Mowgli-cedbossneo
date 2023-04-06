/*
 * cpp_main.h
 * 
 */
#ifndef CPP_MAIN_H_
#define CPP_MAIN_H_

#ifdef __cplusplus
extern "C" {
#endif


void init_ROS();
void spinOnce();
void chatter_handler();
void motors_handler();
void panel_handler();
void broadcast_handler();
void wheelTicks_handler(uint8_t p_u8LeftDirection,uint8_t p_u8RightDirection, uint32_t p_u16LeftTicks, uint32_t p_u16RightTicks, uint32_t p_s16LeftSpeed, uint32_t p_s16RightSpeed);

uint8_t CDC_DataReceivedHandler(const uint8_t *Buf, uint32_t len);
#ifdef __cplusplus
}
#endif

#endif /* CPP_MAIN_H_ */
