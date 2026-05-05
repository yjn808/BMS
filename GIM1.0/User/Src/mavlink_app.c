#include "mavlink_app.h"


// 禁止编译器警告
#if defined(__CC_ARM)
#pragma push
#pragma diag_suppress 66        
#elif defined(__ARMCC_VERSION)
#pragma diag_push
#pragma diag_suppress 66
#endif
#include "common/common.h"
#if defined(__CC_ARM)
#pragma pop
#elif defined(__ARMCC_VERSION)
#pragma diag_pop
#endif



#define MAV_PARAM_MAX 128U

// GPS定位类型
#ifndef MAV_GPS_FIX_TYPE_DGPS
#define MAV_GPS_FIX_TYPE_DGPS 4
#endif
#ifndef MAV_GPS_FIX_TYPE_RTK_FIXED
#define MAV_GPS_FIX_TYPE_RTK_FIXED 6
#endif
#ifndef MAV_GPS_FIX_TYPE_RTK_FLOAT
#define MAV_GPS_FIX_TYPE_RTK_FLOAT 5
#endif
#ifndef MAV_GPS_FIX_TYPE_3D_FIX
#define MAV_GPS_FIX_TYPE_3D_FIX 3
#endif
#ifndef MAV_GPS_FIX_TYPE_NO_FIX
#define MAV_GPS_FIX_TYPE_NO_FIX 1
#endif



typedef struct {
  char name[16];
  float value;
  uint8_t type;
} FakeParam_t;

typedef struct {
  int32_t lat_e7;
  int32_t lon_e7;
  int32_t alt_mm;
  int32_t rel_alt_mm;
  uint16_t eph_cm;
  uint16_t epv_cm;
  uint16_t vel_cm_s;
  uint16_t cog_cdeg;
  int16_t vx_cm_s;
  int16_t vy_cm_s;
  int16_t vz_cm_s;
  uint8_t fix_type;
  uint8_t satellites;
  uint8_t valid;
} PositionData_t;



static const uint8_t MAV_SYS_ID = 1;
static const uint8_t MAV_COMP_ID = MAV_COMP_ID_AUTOPILOT1;
static const float FAKE_HOME_Q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
static const uint8_t MAV_VERSION_EMPTY[8] = {0};
static const uint8_t MAV_UID2_EMPTY[18] = {0};
static const int16_t FAKE_IMU_XACC = 0;
static const int16_t FAKE_IMU_YACC = 0;
static const int16_t FAKE_IMU_ZACC = 1000;
static const int16_t FAKE_IMU_XGYRO = 0;
static const int16_t FAKE_IMU_YGYRO = 0;
static const int16_t FAKE_IMU_ZGYRO = 0;
static const int16_t FAKE_IMU_XMAG = 200;
static const int16_t FAKE_IMU_YMAG = 0;
static const int16_t FAKE_IMU_ZMAG = -200;
static const int16_t FAKE_IMU_TEMP_C = 2500;
static const float FAKE_ABS_PRESS = 1013.25f;
static const float FAKE_DIFF_PRESS = 0.0f;
static const int16_t FAKE_PRESS_TEMP = 2500;
static const int16_t FAKE_PRESS_DIFF_TEMP = 2500;
static const uint32_t HEARTBEAT_PERIOD_MS = 1000U;
static const uint32_t GPS_PERIOD_MS = 200U;
static const uint32_t GLOBAL_POSITION_PERIOD_MS = 200U;
static const uint32_t SYS_STATUS_PERIOD_MS = 1000U;
static const uint32_t SENSOR_PERIOD_MS = 200U;
static const uint32_t MISSION_REQUEST_TIMEOUT_MS = 1000U;


// UART句柄
UART_HandleTypeDef *huart_mavlink = &huart4;
static UART_HandleTypeDef *huart_debug = &huart7;

// 航点数据
Waypoint_t waypoints[MAX_WAYPOINTS];
uint16_t waypoint_count = 0U;

// UART接收缓冲区变量（供usart.c使用）
uint8_t uart_rx_byte = 0U;
volatile uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
volatile uint16_t uart_rx_head = 0U;
volatile uint16_t uart_rx_tail = 0U;
volatile uint32_t uart_rx_overflow_count = 0U;


static uint32_t last_heartbeat_ms = 0U;
static uint32_t last_gps_ms = 0U;
static uint32_t last_global_position_ms = 0U;
static uint32_t last_sys_status_ms = 0U;
static uint32_t last_sensor_ms = 0U;
static uint8_t mpu6050_ready = 0U;
static uint16_t mission_expected_count = 0U;
static uint16_t mission_request_seq = 0U;
static uint8_t mission_upload_in_progress = 0U;
static uint8_t mission_partner_sysid = 0U;
static uint8_t mission_partner_compid = 0U;
static uint32_t last_mission_request_ms = 0U;

static FakeParam_t mavlink_params[MAV_PARAM_MAX] = {
  { "SYSID_THISMAV", 1.0f, MAV_PARAM_TYPE_INT32 },
  { "MAV_SYS_ID", 1.0f, MAV_PARAM_TYPE_INT32 },
  { "MAV_TYPE", (float)MAV_TYPE_QUADROTOR, MAV_PARAM_TYPE_INT32 },
  { "SYS_AUTOSTART", 4010.0f, MAV_PARAM_TYPE_INT32 },
  { "SYS_AUTOCONFIG", 0.0f, MAV_PARAM_TYPE_INT32 },
  { "CAL_GYRO0_ID", 229376.0f, MAV_PARAM_TYPE_INT32 },
  { "CAL_ACC0_ID", 137626.0f, MAV_PARAM_TYPE_INT32 },
  { "CAL_MAG0_ID", 197136.0f, MAV_PARAM_TYPE_INT32 },
  { "CAL_MAG1_ID", 197137.0f, MAV_PARAM_TYPE_INT32 },
  { "CAL_MAG2_ID", 0.0f, MAV_PARAM_TYPE_INT32 },
  { "CAL_GYRO0_XOFF", 0.0f, MAV_PARAM_TYPE_REAL32 },
  { "CAL_GYRO0_YOFF", 0.0f, MAV_PARAM_TYPE_REAL32 },
  { "CAL_GYRO0_ZOFF", 0.0f, MAV_PARAM_TYPE_REAL32 },
  { "CAL_ACC0_XOFF", 0.0f, MAV_PARAM_TYPE_REAL32 },
  { "CAL_ACC0_YOFF", 0.0f, MAV_PARAM_TYPE_REAL32 },
  { "CAL_ACC0_ZOFF", 0.0f, MAV_PARAM_TYPE_REAL32 },
  { "CAL_ACC0_XSCALE", 1.0f, MAV_PARAM_TYPE_REAL32 },
  { "CAL_ACC0_YSCALE", 1.0f, MAV_PARAM_TYPE_REAL32 },
  { "CAL_ACC0_ZSCALE", 1.0f, MAV_PARAM_TYPE_REAL32 },
  { "CAL_MAG0_XOFF", 0.0f, MAV_PARAM_TYPE_REAL32 },
  { "CAL_MAG0_YOFF", 0.0f, MAV_PARAM_TYPE_REAL32 },
  { "CAL_MAG0_ZOFF", 0.0f, MAV_PARAM_TYPE_REAL32 },
  { "CAL_MAG1_XOFF", 0.0f, MAV_PARAM_TYPE_REAL32 },
  { "CAL_MAG1_YOFF", 0.0f, MAV_PARAM_TYPE_REAL32 },
  { "CAL_MAG1_ZOFF", 0.0f, MAV_PARAM_TYPE_REAL32 },
  { "CAL_MAG0_ROT", 0.0f, MAV_PARAM_TYPE_INT32 },
  { "CAL_MAG1_ROT", 0.0f, MAV_PARAM_TYPE_INT32 },
  { "CAL_MAG2_ROT", 0.0f, MAV_PARAM_TYPE_INT32 },
  { "SENS_BOARD_ROT", 0.0f, MAV_PARAM_TYPE_INT32 },
  { "SENS_DPRES_OFF", 0.0f, MAV_PARAM_TYPE_REAL32 },
  { "COM_DL_LOSS_T", 5.0f, MAV_PARAM_TYPE_REAL32 },
  { "COM_RC_IN_MODE", 0.0f, MAV_PARAM_TYPE_INT32 },
  { "RC_MAP_ROLL", 1.0f, MAV_PARAM_TYPE_INT32 },
  { "RC_MAP_PITCH", 2.0f, MAV_PARAM_TYPE_INT32 },
  { "RC_MAP_THROTTLE", 3.0f, MAV_PARAM_TYPE_INT32 },
  { "RC_MAP_YAW", 4.0f, MAV_PARAM_TYPE_INT32 },
  { "RC_MAP_AUX1", 5.0f, MAV_PARAM_TYPE_INT32 },
  { "RC_MAP_AUX2", 6.0f, MAV_PARAM_TYPE_INT32 },
  { "RC_MAP_FLAPS", 0.0f, MAV_PARAM_TYPE_INT32 },
  { "COM_FLTMODE1", 0.0f, MAV_PARAM_TYPE_INT32 },
  { "COM_FLTMODE2", 0.0f, MAV_PARAM_TYPE_INT32 },
  { "COM_FLTMODE3", 0.0f, MAV_PARAM_TYPE_INT32 },
  { "COM_FLTMODE4", 0.0f, MAV_PARAM_TYPE_INT32 },
  { "COM_FLTMODE5", 0.0f, MAV_PARAM_TYPE_INT32 },
  { "COM_FLTMODE6", 0.0f, MAV_PARAM_TYPE_INT32 },
  { "MIS_TAKEOFF_ALT", 15.0f, MAV_PARAM_TYPE_REAL32 },
  { "RTL_RETURN_ALT", 30.0f, MAV_PARAM_TYPE_REAL32 },
  { "RTL_DESCEND_ALT", 10.0f, MAV_PARAM_TYPE_REAL32 },
  { "RTL_LAND_DELAY", 0.0f, MAV_PARAM_TYPE_REAL32 },
  { "NAV_DLL_ACT", 0.0f, MAV_PARAM_TYPE_INT32 },
  { "NAV_RCL_ACT", 0.0f, MAV_PARAM_TYPE_INT32 },
  { "COM_RC_LOSS_T", 5.0f, MAV_PARAM_TYPE_REAL32 },
  { "COM_LOW_BAT_ACT", 0.0f, MAV_PARAM_TYPE_INT32 },
  { "BAT1_SOURCE", 0.0f, MAV_PARAM_TYPE_INT32 },
  { "BAT_CAPACITY", 2200.0f, MAV_PARAM_TYPE_REAL32 }
};

static uint16_t mavlink_param_count = 0U;
static char debug_buf[128];
static int32_t home_lat_e7 = 0;
static int32_t home_lon_e7 = 0;
static int32_t home_alt_mm = 0;
static uint8_t home_position_valid = 0U;
static uint8_t home_origin_sent = 0U;
static uint8_t home_position_sent = 0U;

// MAVLink解析器状态（仅内部使用）
static mavlink_message_t uart_rx_msg;
static mavlink_status_t uart_rx_status;

/* ========== 函数声明 ========== */

static void debug_log(const char *msg);
static void debug_log_waypoint(uint16_t seq, float lat_deg, float lon_deg, float alt_m);
static void mavlink_send_heartbeat(void);
static void mavlink_send_gps_raw_int(uint32_t time_ms);
static void mavlink_send_global_position_int(uint32_t time_ms);
static void mavlink_send_set_gps_global_origin(uint32_t time_ms);
static void mavlink_send_home_position(uint32_t time_ms);
static void mavlink_send_mission_count(uint8_t target_sys, uint8_t target_comp, uint16_t count);
static void mavlink_send_mission_ack(uint8_t target_sys, uint8_t target_comp, uint8_t type);
static void mavlink_send_mission_request(uint8_t target_sys, uint8_t target_comp, uint16_t seq);
static void mavlink_send_mission_request_basic(uint8_t target_sys, uint8_t target_comp, uint16_t seq);
static void mavlink_request_mission_sequence(uint16_t seq);
static void mavlink_send_command_ack(uint16_t command, uint8_t result, uint8_t target_sys, uint8_t target_comp);
static void mavlink_send_sys_status(void);
static void mavlink_send_autopilot_version(void);
static void mavlink_send_scaled_imu(uint32_t time_ms);
static void mavlink_send_scaled_pressure(uint32_t time_ms);
static void mavlink_send_param_value(const char *name, float value, uint16_t index, uint16_t count, uint8_t type);
static int16_t mavlink_param_find_index(const char *name);
static int16_t mavlink_param_add(const char *name, float value, uint8_t type);
static void mavlink_params_init(void);
static void mavlink_prepare_position(PositionData_t *pos);
static uint8_t mavlink_map_fix_type(uint8_t quality, uint8_t has_fix);
static void mavlink_handle_message(const mavlink_message_t *msg);



static void debug_log(const char *msg)
{
  if ((msg == NULL) || (msg[0] == '\0') || (huart_debug == NULL))
  {
    return;
  }
  size_t len = strlen(msg);
  if (len > 0U)
  {
    HAL_UART_Transmit(huart_debug, (uint8_t *)msg, (uint16_t)len, HAL_MAX_DELAY);
  }
}

static void debug_log_waypoint(uint16_t seq, float lat_deg, float lon_deg, float alt_m)
{
  int written = snprintf(debug_buf, sizeof(debug_buf),
                         "WP %u lat=%.7f lon=%.7f alt=%.2f\r\n",
                         (unsigned int)seq, lat_deg, lon_deg, alt_m);
  if (written < 0) return;
  size_t len = (size_t)written;
  if (len >= sizeof(debug_buf))
  {
    len = sizeof(debug_buf) - 1U;
    debug_buf[len] = '\0';
  }
  debug_log(debug_buf);
}

static void mavlink_params_init(void)
{
  if (mavlink_param_count == 0U)
  {
    while ((mavlink_param_count < MAV_PARAM_MAX) && 
           (mavlink_params[mavlink_param_count].name[0] != '\0'))
    {
      mavlink_param_count++;
    }
  }
}

static int16_t mavlink_param_find_index(const char *name)
{
  if (name == NULL) return -1;
  mavlink_params_init();
  for (uint16_t i = 0; i < mavlink_param_count; ++i)
  {
    if (strncmp(mavlink_params[i].name, name, sizeof(mavlink_params[i].name)) == 0)
    {
      return (int16_t)i;
    }
  }
  return -1;
}

static int16_t mavlink_param_add(const char *name, float value, uint8_t type)
{
  if ((name == NULL) || (name[0] == '\0'))
  {
    return -1;
  }
  
  mavlink_params_init();
  
  if (mavlink_param_count >= MAV_PARAM_MAX)
  {
    return -1;
  }
  
  FakeParam_t *param = &mavlink_params[mavlink_param_count];
  strncpy(param->name, name, sizeof(param->name));
  param->name[sizeof(param->name) - 1U] = '\0';
  param->value = value;
  param->type = type;
  
  mavlink_param_count++;
  return (int16_t)(mavlink_param_count - 1U);
}

static uint8_t mavlink_map_fix_type(uint8_t quality, uint8_t has_fix)
{
  switch (quality)
  {
    case 2: return MAV_GPS_FIX_TYPE_DGPS;
    case 4: return MAV_GPS_FIX_TYPE_RTK_FIXED;
    case 5: return MAV_GPS_FIX_TYPE_RTK_FLOAT;
    case 1: return MAV_GPS_FIX_TYPE_3D_FIX;
    default: return (has_fix != 0U) ? MAV_GPS_FIX_TYPE_3D_FIX : MAV_GPS_FIX_TYPE_NO_FIX;
  }
}



static void mavlink_prepare_position(PositionData_t *pos)
{
  if (pos == NULL) return;
  
  memset(pos, 0, sizeof(*pos));
  pos->fix_type = MAV_GPS_FIX_TYPE_NO_FIX;
  
  extern GNSS_Data_t gnss_data;
  
  if (gnss_data.position_status != 'A')
  {
    return;
  }
  
  pos->lat_e7 = (int32_t)(gnss_data.lat_degree * 1e7);
  pos->lon_e7 = (int32_t)(gnss_data.lon_degree * 1e7);
  
  if (strlen(gnss_data.altitude) > 0)
  {
    float alt_m = atof(gnss_data.altitude);
    pos->alt_mm = (int32_t)(alt_m * 1000.0f);
  }
  
  if (strlen(gnss_data.hdop) > 0)
  {
    float hdop = atof(gnss_data.hdop);
    pos->eph_cm = (uint16_t)(hdop * 100.0f);
    pos->epv_cm = pos->eph_cm;
  }
  
  if (strlen(gnss_data.speed) > 0)
  {
    float speed_kmh = atof(gnss_data.speed);
    float speed_m_s = speed_kmh / 3.6f;
    pos->vel_cm_s = (uint16_t)(speed_m_s * 100.0f);
  }
  
  if (strlen(gnss_data.course) > 0)
  {
    float course_deg = atof(gnss_data.course);
    pos->cog_cdeg = (uint16_t)(course_deg * 100.0f);
  }
  
  uint8_t quality = 0;
  if (strlen(gnss_data.quality) > 0)
  {
    quality = (uint8_t)atoi(gnss_data.quality);
  }
  pos->fix_type = mavlink_map_fix_type(quality, 1U);
  
  if (strlen(gnss_data.satellites) > 0)
  {
    pos->satellites = (uint8_t)atoi(gnss_data.satellites);
  }
  
  pos->valid = 1U;
  
  if (!home_position_valid && pos->valid)
  {
    home_lat_e7 = pos->lat_e7;
    home_lon_e7 = pos->lon_e7;
    home_alt_mm = pos->alt_mm;
    home_position_valid = 1U;
    home_origin_sent = 0U;
    home_position_sent = 0U;
  }
  
  if (home_position_valid)
  {
    pos->rel_alt_mm = pos->alt_mm - home_alt_mm;
  }
}



static void mavlink_send_heartbeat(void)
{
  if (huart_mavlink == NULL) return;
  
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  const uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                            MAV_MODE_FLAG_GUIDED_ENABLED |
                            MAV_MODE_FLAG_AUTO_ENABLED;
  
  mavlink_msg_heartbeat_pack(MAV_SYS_ID, MAV_COMP_ID, &msg,
                             MAV_TYPE_QUADROTOR,
                             MAV_AUTOPILOT_PX4,
                             base_mode, 0, MAV_STATE_ACTIVE);
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  HAL_UART_Transmit(huart_mavlink, buf, len, 100);
}

static void mavlink_send_gps_raw_int(uint32_t time_ms)
{
  if (huart_mavlink == NULL) return;
  
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint64_t time_us = (uint64_t)time_ms * 1000ULL;
  PositionData_t pos = {0};
  
  mavlink_prepare_position(&pos);
  
  int32_t alt_ellipsoid_mm = pos.alt_mm;
  uint32_t h_acc_mm = (uint32_t)pos.eph_cm * 10U;
  uint32_t v_acc_mm = (uint32_t)pos.epv_cm * 10U;
  
  mavlink_msg_gps_raw_int_pack(MAV_SYS_ID, MAV_COMP_ID, &msg,
                                time_us, pos.fix_type,
                                pos.lat_e7, pos.lon_e7, pos.alt_mm,
                                pos.eph_cm, pos.epv_cm,
                                pos.vel_cm_s, pos.cog_cdeg,
                                pos.satellites,
                                alt_ellipsoid_mm, h_acc_mm, v_acc_mm,
                                0, 0, pos.cog_cdeg);
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  HAL_UART_Transmit(huart_mavlink, buf, len, 100);
}

static void mavlink_send_global_position_int(uint32_t time_ms)
{
  if (huart_mavlink == NULL) return;
  
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  PositionData_t pos = {0};
  
  mavlink_prepare_position(&pos);
  
  mavlink_msg_global_position_int_pack(MAV_SYS_ID, MAV_COMP_ID, &msg,
                                       time_ms, pos.lat_e7, pos.lon_e7,
                                       pos.alt_mm, pos.rel_alt_mm,
                                       pos.vx_cm_s, pos.vy_cm_s, pos.vz_cm_s,
                                       pos.cog_cdeg);
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  HAL_UART_Transmit(huart_mavlink, buf, len, 100);
}

static void mavlink_send_set_gps_global_origin(uint32_t time_ms)
{
  if (huart_mavlink == NULL) return;
  
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint64_t time_us = (uint64_t)time_ms * 1000ULL;
  
  mavlink_msg_set_gps_global_origin_pack(MAV_SYS_ID, MAV_COMP_ID, &msg,
                                         MAV_SYS_ID, home_lat_e7, home_lon_e7,
                                         home_alt_mm, time_us);
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  HAL_UART_Transmit(huart_mavlink, buf, len, 100);
}

static void mavlink_send_home_position(uint32_t time_ms)
{
  if (huart_mavlink == NULL) return;
  
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint64_t time_us = (uint64_t)time_ms * 1000ULL;
  
  mavlink_msg_home_position_pack(MAV_SYS_ID, MAV_COMP_ID, &msg,
                                 home_lat_e7, home_lon_e7, home_alt_mm,
                                 0.0f, 0.0f, 0.0f, FAKE_HOME_Q,
                                 0.0f, 0.0f, 0.0f, time_us);
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  HAL_UART_Transmit(huart_mavlink, buf, len, 100);
}

static void mavlink_send_sys_status(void)
{
  if (huart_mavlink == NULL) return;
  
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  const uint32_t sensors = MAV_SYS_STATUS_SENSOR_3D_GYRO |
                           MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                           MAV_SYS_STATUS_SENSOR_3D_MAG |
                           MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                           MAV_SYS_STATUS_SENSOR_GPS;
  
  mavlink_msg_sys_status_pack(MAV_SYS_ID, MAV_COMP_ID, &msg,
                              sensors, sensors, sensors,
                              0, 0, -1, -1,
                              0, 0, 0, 0, 0, 0,
                              sensors, sensors, sensors);
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  HAL_UART_Transmit(huart_mavlink, buf, len, 100);
}

static void mavlink_send_autopilot_version(void)
{
  if (huart_mavlink == NULL) return;
  
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  const uint64_t capabilities = MAV_PROTOCOL_CAPABILITY_MISSION_INT |
                                MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                                MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT;
  
  const uint32_t flight_sw_version = (1U << 24) | (0U << 16) | 
                                      (0U << 8) | FIRMWARE_VERSION_TYPE_DEV;
  
  mavlink_msg_autopilot_version_pack(MAV_SYS_ID, MAV_COMP_ID, &msg,
                                     capabilities, flight_sw_version,
                                     0, 0, 0,
                                     MAV_VERSION_EMPTY, MAV_VERSION_EMPTY, MAV_VERSION_EMPTY,
                                     0, 0, 0, MAV_UID2_EMPTY);
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  HAL_UART_Transmit(huart_mavlink, buf, len, 100);
}

/* ========== IMU数据发送 - 直接读取AccX/Y/Z全局变量 ========== */

static void mavlink_send_scaled_imu(uint32_t time_ms)
{
  if (huart_mavlink == NULL) return;
  
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  int16_t xacc = FAKE_IMU_XACC;
  int16_t yacc = FAKE_IMU_YACC;
  int16_t zacc = FAKE_IMU_ZACC;
  int16_t xgyro = FAKE_IMU_XGYRO;
  int16_t ygyro = FAKE_IMU_YGYRO;
  int16_t zgyro = FAKE_IMU_ZGYRO;
  int16_t temp = FAKE_IMU_TEMP_C;
  
  // 如果MPU6050已就绪，读取真实数据
  if (mpu6050_ready != 0U)
  {
    // 声明外部全局变量 (MPU6050.c中定义的)
    extern int16_t AccX, AccY, AccZ;
    extern int16_t GyroX, GyroY, GyroZ;
    
    // 加速度转换: LSB → mG (毫重力)
    // ±8g量程: 4096 LSB/g
    // 公式: mG = (LSB * 1000) / 4096
    xacc = (int16_t)((int32_t)AccX * 1000 / 4096);
    yacc = (int16_t)((int32_t)AccY * 1000 / 4096);
    zacc = (int16_t)((int32_t)AccZ * 1000 / 4096);
    
    // 角速度转换: LSB → m°/s (毫度/秒)
    // ±2000°/s量程: 16.4 LSB/(°/s)
    // 公式: m°/s = (LSB / 16.4) * 1000
    xgyro = (int16_t)((float)GyroX / 16.4f * 1000.0f);
    ygyro = (int16_t)((float)GyroY / 16.4f * 1000.0f);
    zgyro = (int16_t)((float)GyroZ / 16.4f * 1000.0f);
    
    // 温度 (°C*100)
    temp = (int16_t)MPU_Get_Temperature();
  }
  
  mavlink_msg_scaled_imu_pack(MAV_SYS_ID, MAV_COMP_ID, &msg,
                              time_ms,
                              xacc, yacc, zacc,
                              xgyro, ygyro, zgyro,
                              FAKE_IMU_XMAG, FAKE_IMU_YMAG, FAKE_IMU_ZMAG,
                              temp);
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  HAL_UART_Transmit(huart_mavlink, buf, len, 100);
}

static void mavlink_send_scaled_pressure(uint32_t time_ms)
{
  if (huart_mavlink == NULL) return;
  
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_scaled_pressure_pack(MAV_SYS_ID, MAV_COMP_ID, &msg,
                                   time_ms, FAKE_ABS_PRESS, FAKE_DIFF_PRESS,
                                   FAKE_PRESS_TEMP, FAKE_PRESS_DIFF_TEMP);
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  HAL_UART_Transmit(huart_mavlink, buf, len, 100);
}

static void mavlink_send_param_value(const char *name, float value,
                                      uint16_t index, uint16_t count, uint8_t type)
{
  if (huart_mavlink == NULL) return;
  
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_param_value_pack(MAV_SYS_ID, MAV_COMP_ID, &msg,
                               name, value, type, count, index);
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  HAL_UART_Transmit(huart_mavlink, buf, len, 100);
}

/* ========== 任务(Mission)管理函数 ========== */

static void mavlink_send_mission_count(uint8_t target_sys, uint8_t target_comp, uint16_t count)
{
  if (huart_mavlink == NULL) return;
  
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_mission_count_pack(MAV_SYS_ID, MAV_COMP_ID, &msg,
                                 target_sys, target_comp, count,
                                 MAV_MISSION_TYPE_MISSION, 0);
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  HAL_UART_Transmit(huart_mavlink, buf, len, 100);
}

static void mavlink_send_mission_ack(uint8_t target_sys, uint8_t target_comp, uint8_t type)
{
  if (huart_mavlink == NULL) return;
  
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_mission_ack_pack(MAV_SYS_ID, MAV_COMP_ID, &msg,
                               target_sys, target_comp, type,
                               MAV_MISSION_TYPE_MISSION, 0);
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  HAL_UART_Transmit(huart_mavlink, buf, len, 100);
}

static void mavlink_send_mission_request(uint8_t target_sys, uint8_t target_comp, uint16_t seq)
{
  if (huart_mavlink == NULL) return;
  
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_mission_request_int_pack(MAV_SYS_ID, MAV_COMP_ID, &msg,
                                       target_sys, target_comp, seq,
                                       MAV_MISSION_TYPE_MISSION);
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  HAL_UART_Transmit(huart_mavlink, buf, len, 100);
}

static void mavlink_send_mission_request_basic(uint8_t target_sys, uint8_t target_comp, uint16_t seq)
{
  if (huart_mavlink == NULL) return;
  
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_mission_request_pack(MAV_SYS_ID, MAV_COMP_ID, &msg,
                                   target_sys, target_comp, seq,
                                   MAV_MISSION_TYPE_MISSION);
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  HAL_UART_Transmit(huart_mavlink, buf, len, 100);
}

static void mavlink_request_mission_sequence(uint16_t seq)
{
  if (mission_partner_sysid == 0U) return;
  
  mavlink_send_mission_request(mission_partner_sysid, mission_partner_compid, seq);
  mavlink_send_mission_request_basic(mission_partner_sysid, mission_partner_compid, seq);
  last_mission_request_ms = HAL_GetTick();
}

static void mavlink_send_command_ack(uint16_t command, uint8_t result,
                                      uint8_t target_sys, uint8_t target_comp)
{
  if (huart_mavlink == NULL) return;
  
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_command_ack_pack(MAV_SYS_ID, MAV_COMP_ID, &msg,
                               command, result, 0, 0,
                               target_sys, target_comp);
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  HAL_UART_Transmit(huart_mavlink, buf, len, 100);
}

/* ========== 消息处理函数 ========== */

static void mavlink_handle_message(const mavlink_message_t *msg)
{
  if (msg == NULL) return;
  
  switch (msg->msgid)
  {
    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
    {
      mavlink_param_request_list_t req;
      mavlink_msg_param_request_list_decode(msg, &req);
      
      if ((req.target_system == 0U || req.target_system == MAV_SYS_ID) &&
          (req.target_component == 0U || req.target_component == MAV_COMP_ID))
      {
        mavlink_params_init();
        for (uint16_t i = 0; i < mavlink_param_count; ++i)
        {
          mavlink_send_param_value(mavlink_params[i].name,
                                   mavlink_params[i].value,
                                   i, mavlink_param_count,
                                   mavlink_params[i].type);
        }
      }
      break;
    }
    
    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
    {
      mavlink_param_request_read_t read;
      mavlink_msg_param_request_read_decode(msg, &read);
      
      if ((read.target_system == 0U || read.target_system == MAV_SYS_ID) &&
          (read.target_component == 0U || read.target_component == MAV_COMP_ID))
      {
        mavlink_params_init();
        
        if (read.param_index >= 0 && (uint16_t)read.param_index < mavlink_param_count)
        {
          uint16_t idx = (uint16_t)read.param_index;
          mavlink_send_param_value(mavlink_params[idx].name,
                                   mavlink_params[idx].value,
                                   idx, mavlink_param_count,
                                   mavlink_params[idx].type);
        }
        else
        {
          char param_name[17];
          memcpy(param_name, read.param_id, sizeof(read.param_id));
          param_name[16] = '\0';
          
          int16_t found = mavlink_param_find_index(param_name);
          if (found < 0)
          {
            found = mavlink_param_add(param_name, 0.0f, MAV_PARAM_TYPE_REAL32);
          }
          
          if (found >= 0)
          {
            uint16_t idx = (uint16_t)found;
            mavlink_send_param_value(mavlink_params[idx].name,
                                     mavlink_params[idx].value,
                                     idx, mavlink_param_count,
                                     mavlink_params[idx].type);
          }
        }
      }
      break;
    }
    
    case MAVLINK_MSG_ID_PARAM_SET:
    {
      mavlink_param_set_t set;
      mavlink_msg_param_set_decode(msg, &set);
      
      if ((set.target_system == 0U || set.target_system == MAV_SYS_ID) &&
          (set.target_component == 0U || set.target_component == MAV_COMP_ID))
      {
        char param_name[17];
        memcpy(param_name, set.param_id, sizeof(set.param_id));
        param_name[16] = '\0';
        
        int16_t found = mavlink_param_find_index(param_name);
        if (found < 0)
        {
          found = mavlink_param_add(param_name, set.param_value, (uint8_t)set.param_type);
        }
        
        if (found >= 0)
        {
          uint16_t idx = (uint16_t)found;
          mavlink_params[idx].value = set.param_value;
          mavlink_params[idx].type = (uint8_t)set.param_type;
          mavlink_send_param_value(mavlink_params[idx].name,
                                   mavlink_params[idx].value,
                                   idx, mavlink_param_count,
                                   mavlink_params[idx].type);
        }
      }
      break;
    }
    
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
    {
      mavlink_mission_request_list_t req;
      mavlink_msg_mission_request_list_decode(msg, &req);
      mavlink_send_mission_count(msg->sysid, msg->compid, waypoint_count);
      break;
    }
    
    case MAVLINK_MSG_ID_MISSION_COUNT:
    {
      mavlink_mission_count_t count;
      mavlink_msg_mission_count_decode(msg, &count);
      
      mission_partner_sysid = msg->sysid;
      mission_partner_compid = msg->compid;
      mission_expected_count = count.count;
      mission_request_seq = 0U;
      waypoint_count = 0U;
      
      if (mission_expected_count == 0U)
      {
        mission_upload_in_progress = 0U;
        last_mission_request_ms = 0U;
        mavlink_send_mission_ack(mission_partner_sysid, mission_partner_compid,
                                  MAV_MISSION_ACCEPTED);
      }
      else if (mission_expected_count > MAX_WAYPOINTS)
      {
        mission_upload_in_progress = 0U;
        mission_expected_count = 0U;
        last_mission_request_ms = 0U;
        mavlink_send_mission_ack(mission_partner_sysid, mission_partner_compid,
                                  MAV_MISSION_NO_SPACE);
      }
      else
      {
        mission_upload_in_progress = 1U;
        mavlink_request_mission_sequence(mission_request_seq);
      }
      break;
    }

    case MAVLINK_MSG_ID_MISSION_ITEM_INT:
    {
      mavlink_mission_item_int_t wp;
      mavlink_msg_mission_item_int_decode(msg, &wp);
      
      if (wp.seq < MAX_WAYPOINTS)
      {
        waypoints[wp.seq].lat = wp.x / 1e7f;
        waypoints[wp.seq].lon = wp.y / 1e7f;
        waypoints[wp.seq].alt = wp.z;
        debug_log_waypoint(wp.seq, waypoints[wp.seq].lat,
                           waypoints[wp.seq].lon, waypoints[wp.seq].alt);
        
        if ((uint16_t)(wp.seq + 1U) > waypoint_count)
        {
          waypoint_count = (uint16_t)(wp.seq + 1U);
        }
        
        if (mission_upload_in_progress && (wp.seq == mission_request_seq))
        {
          if ((uint16_t)(wp.seq + 1U) >= mission_expected_count)
          {
            mission_upload_in_progress = 0U;
            mission_expected_count = waypoint_count;
            mavlink_send_mission_ack(mission_partner_sysid, mission_partner_compid,
                                      MAV_MISSION_ACCEPTED);
            last_mission_request_ms = 0U;
          }
          else
          {
            mission_request_seq = (uint16_t)(wp.seq + 1U);
            mavlink_request_mission_sequence(mission_request_seq);
          }
        }
      }
      else
      {
        mavlink_send_mission_ack(msg->sysid, msg->compid, MAV_MISSION_NO_SPACE);
        mission_upload_in_progress = 0U;
        last_mission_request_ms = 0U;
      }
      break;
    }
    
    case MAVLINK_MSG_ID_MISSION_ITEM:
    {
      mavlink_mission_item_t wp;
      mavlink_msg_mission_item_decode(msg, &wp);
      
      uint16_t seq = wp.seq;
      if (seq < MAX_WAYPOINTS)
      {
        waypoints[seq].lat = wp.x;
        waypoints[seq].lon = wp.y;
        waypoints[seq].alt = wp.z;
        debug_log_waypoint(seq, waypoints[seq].lat,
                           waypoints[seq].lon, waypoints[seq].alt);
        
        if ((uint16_t)(seq + 1U) > waypoint_count)
        {
          waypoint_count = (uint16_t)(seq + 1U);
        }
        
        if (mission_upload_in_progress && seq == mission_request_seq)
        {
          if ((uint16_t)(seq + 1U) >= mission_expected_count)
          {
            mission_upload_in_progress = 0U;
            mission_expected_count = waypoint_count;
            mavlink_send_mission_ack(mission_partner_sysid, mission_partner_compid,
                                      MAV_MISSION_ACCEPTED);
            last_mission_request_ms = 0U;
          }
          else
          {
            mission_request_seq = (uint16_t)(seq + 1U);
            mavlink_request_mission_sequence(mission_request_seq);
          }
        }
      }
      else
      {
        mavlink_send_mission_ack(msg->sysid, msg->compid, MAV_MISSION_NO_SPACE);
        mission_upload_in_progress = 0U;
        last_mission_request_ms = 0U;
      }
      break;
    }
    
    case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
    {
      mavlink_mission_clear_all_t clear;
      mavlink_msg_mission_clear_all_decode(msg, &clear);
      
      waypoint_count = 0U;
      mission_expected_count = 0U;
      mission_request_seq = 0U;
      mission_upload_in_progress = 0U;
      last_mission_request_ms = 0U;
      
      mavlink_send_mission_ack(msg->sysid, msg->compid, MAV_MISSION_ACCEPTED);
      break;
    }
    
    case MAVLINK_MSG_ID_COMMAND_LONG:
    {
      mavlink_command_long_t cmd;
      mavlink_msg_command_long_decode(msg, &cmd);
      uint16_t command = cmd.command;
      
      if (command == MAV_CMD_REQUEST_MESSAGE)
      {
        uint32_t requested_id = (uint32_t)cmd.param1;
        if (requested_id == MAVLINK_MSG_ID_AUTOPILOT_VERSION)
        {
          mavlink_send_autopilot_version();
          mavlink_send_command_ack(command, MAV_RESULT_ACCEPTED, msg->sysid, msg->compid);
        }
        else if (requested_id == MAVLINK_MSG_ID_SYS_STATUS)
        {
          mavlink_send_sys_status();
          mavlink_send_command_ack(command, MAV_RESULT_ACCEPTED, msg->sysid, msg->compid);
        }
        else if (requested_id == MAVLINK_MSG_ID_HOME_POSITION)
        {
          mavlink_send_home_position(HAL_GetTick());
          mavlink_send_command_ack(command, MAV_RESULT_ACCEPTED, msg->sysid, msg->compid);
        }
        else if (requested_id == MAVLINK_MSG_ID_GLOBAL_POSITION_INT)
        {
          mavlink_send_global_position_int(HAL_GetTick());
          mavlink_send_command_ack(command, MAV_RESULT_ACCEPTED, msg->sysid, msg->compid);
        }
        else
        {
          mavlink_send_command_ack(command, MAV_RESULT_UNSUPPORTED, msg->sysid, msg->compid);
        }
      }
      else if (command == MAV_CMD_COMPONENT_ARM_DISARM)
      {
        mavlink_send_command_ack(command, MAV_RESULT_ACCEPTED, msg->sysid, msg->compid);
      }
      else if (command == MAV_CMD_DO_SET_MODE)
      {
        mavlink_send_command_ack(command, MAV_RESULT_ACCEPTED, msg->sysid, msg->compid);
      }
      else
      {
        mavlink_send_command_ack(command, MAV_RESULT_UNSUPPORTED, msg->sysid, msg->compid);
      }
      break;
    }
    
    default:
      break;
  }
}

/* ========== 公共API函数 ========== */

void MAVLink_Init(UART_HandleTypeDef *huart1_ptr, UART_HandleTypeDef *huart2_ptr)
{
  huart_mavlink = huart1_ptr;
  huart_debug = huart2_ptr;
  
  if (huart_mavlink != NULL)
  {
    HAL_UART_Receive_IT(huart_mavlink, &uart_rx_byte, 1);
  }
  
  mavlink_params_init();
  
  uint32_t now = HAL_GetTick();
  last_heartbeat_ms = now;
  last_gps_ms = now;
  last_global_position_ms = now;
  last_sys_status_ms = now;
  last_sensor_ms = now;
  
  debug_log("MAVLink App initialized\r\n");
}

void MAVLink_Loop(void)
{
  // 处理Home位置更新
  if (home_position_valid)
  {
    if (home_origin_sent == 0U)
    {
      mavlink_send_set_gps_global_origin(HAL_GetTick());
      home_origin_sent = 1U;
    }
    if (home_position_sent == 0U)
    {
      mavlink_send_home_position(HAL_GetTick());
      home_position_sent = 1U;
    }
  }
  
  // 处理UART接收缓冲区
  while (uart_rx_head != uart_rx_tail)
  {
    uint8_t byte = uart_rx_buffer[uart_rx_tail];
    uart_rx_tail = (uint16_t)((uart_rx_tail + 1U) % UART_RX_BUFFER_SIZE);
    
    if (mavlink_parse_char(MAVLINK_COMM_0, byte, &uart_rx_msg, &uart_rx_status))
    {
      mavlink_handle_message(&uart_rx_msg);
    }
  }
  
  uint32_t tick = HAL_GetTick();
  
  // 心跳包 (1Hz)
  if (tick - last_heartbeat_ms >= HEARTBEAT_PERIOD_MS)
  {
    mavlink_send_heartbeat();
    last_heartbeat_ms = tick;
  }
  
  // GPS (5Hz)
  if (tick - last_gps_ms >= GPS_PERIOD_MS)
  {
    mavlink_send_gps_raw_int(tick);
    last_gps_ms = tick;
  }
  
  // 全局位置 (5Hz)
  if (tick - last_global_position_ms >= GLOBAL_POSITION_PERIOD_MS)
  {
    mavlink_send_global_position_int(tick);
    last_global_position_ms = tick;
  }
  
  // 系统状态 (1Hz)
  if (tick - last_sys_status_ms >= SYS_STATUS_PERIOD_MS)
  {
    mavlink_send_sys_status();
    last_sys_status_ms = tick;
  }
  
  // 传感器数据 (5Hz)
  if (tick - last_sensor_ms >= SENSOR_PERIOD_MS)
  {
    mavlink_send_scaled_imu(tick);
    mavlink_send_scaled_pressure(tick);
    last_sensor_ms = tick;
  }
  
  // 任务请求超时
  if (mission_upload_in_progress && last_mission_request_ms != 0U &&
      (tick - last_mission_request_ms) >= MISSION_REQUEST_TIMEOUT_MS)
  {
    mavlink_request_mission_sequence(mission_request_seq);
  }
}

void MAVLink_SetMPU6050Ready(uint8_t ready)
{
  mpu6050_ready = ready;
}

void MAVLink_SetHomePosition(int32_t lat_e7, int32_t lon_e7, int32_t alt_mm)
{
  home_lat_e7 = lat_e7;
  home_lon_e7 = lon_e7;
  home_alt_mm = alt_mm;
  home_position_valid = 1U;
  home_origin_sent = 0U;
  home_position_sent = 0U;
}

