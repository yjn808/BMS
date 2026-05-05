#ifndef __MAVLINK_APP_H
#define __MAVLINK_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"


#define MAX_WAYPOINTS 100
#define UART_RX_BUFFER_SIZE 512U


typedef struct {
  float lat;
  float lon;
  float alt;
} Waypoint_t;



// 航点数据
extern Waypoint_t waypoints[MAX_WAYPOINTS];
extern uint16_t waypoint_count;


extern UART_HandleTypeDef *huart_mavlink;
extern uint8_t uart_rx_byte;
extern volatile uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
extern volatile uint16_t uart_rx_head;
extern volatile uint16_t uart_rx_tail;
extern volatile uint32_t uart_rx_overflow_count;

/* ========== 函数声明 ========== */
void MAVLink_Init(UART_HandleTypeDef *huart_mav, UART_HandleTypeDef *huart_dbg);
void MAVLink_Loop(void);
void MAVLink_SetMPU6050Ready(uint8_t ready);
void MAVLink_SetHomePosition(int32_t lat_e7, int32_t lon_e7, int32_t alt_mm);

#ifdef __cplusplus
}
#endif

#endif /* __MAVLINK_APP_H */


