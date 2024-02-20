// Module for interfacing with adc hardware

#include "module.h"
#include "lauxlib.h"
#include "platform.h"

#include "driver/gpio.h"
#include "driver/twai.h"
#include "hal/twai_types.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_task.h"
#include "esp_log.h"

#include <string.h>

#include "task/task.h"

/** \brief CAN Node Bus speed */
typedef enum  {
  CAN_SPEED_100KBPS=100, 				/**< \brief CAN Node runs at 100kBit/s. */
  CAN_SPEED_125KBPS=125, 				/**< \brief CAN Node runs at 125kBit/s. */
  CAN_SPEED_250KBPS=250, 				/**< \brief CAN Node runs at 250kBit/s. */
  CAN_SPEED_500KBPS=500, 				/**< \brief CAN Node runs at 500kBit/s. */
  CAN_SPEED_800KBPS=800, 				/**< \brief CAN Node runs at 800kBit/s. */
  CAN_SPEED_1000KBPS=1000				/**< \brief CAN Node runs at 1000kBit/s. */
}CAN_speed_t;

typedef struct  {
  CAN_speed_t			speed;			/**< \brief CAN speed. */
  gpio_num_t 			tx_pin_id;		/**< \brief TX pin. */
  gpio_num_t 			rx_pin_id;		/**< \brief RX pin. */
  QueueHandle_t 		rx_queue;		/**< \brief Handler to FreeRTOS RX queue. */
  uint32_t			code;
  uint32_t			mask;
  bool				dual_filter;
}CAN_device_t;

CAN_device_t CAN_cfg = {
  .speed = CAN_SPEED_1000KBPS,    // CAN Node baudrade
  .tx_pin_id = -1,    // CAN TX pin
  .rx_pin_id = -1,    // CAN RX pin
  .code = 0,
  .mask = 0xffffffff,
  .dual_filter = false
};

static task_handle_t can_data_task_id;
static int can_on_received = LUA_NOREF;

static TaskHandle_t  xCanTaskHandle = NULL;

// LUA
static void can_data_task( task_param_t param, task_prio_t prio ) {
  twai_message_t *message = (twai_message_t *)param;

  if(can_on_received == LUA_NOREF) {
    free( message );
    return;
  }
  lua_State *L = lua_getstate();

  lua_rawgeti(L, LUA_REGISTRYINDEX, can_on_received);
  lua_pushinteger(L, message->extd ? 1 : 0);
  lua_pushinteger(L, message->identifier);
  lua_pushlstring(L, (char *)message->data, message->data_length_code);
  free( message );
  luaL_pcallx(L, 3, 0);
}

static bool get_timing_config(CAN_speed_t speed, twai_timing_config_t *t_config) {
  switch (speed) {
    case CAN_SPEED_100KBPS:
      *t_config = (twai_timing_config_t) TWAI_TIMING_CONFIG_100KBITS();
      return true;
    case CAN_SPEED_125KBPS:
      *t_config = (twai_timing_config_t) TWAI_TIMING_CONFIG_125KBITS();
      return true;
    case CAN_SPEED_250KBPS:
      *t_config = (twai_timing_config_t) TWAI_TIMING_CONFIG_250KBITS();
      return true;
    case CAN_SPEED_500KBPS:
      *t_config = (twai_timing_config_t) TWAI_TIMING_CONFIG_500KBITS();
      return true;
    case CAN_SPEED_800KBPS:
      *t_config = (twai_timing_config_t) TWAI_TIMING_CONFIG_800KBITS();
      return true;
    case CAN_SPEED_1000KBPS:
      *t_config = (twai_timing_config_t) TWAI_TIMING_CONFIG_1MBITS();
      return true;
  }
  return false;
}

// RTOS
static void task_CAN( void *pvParameters ){
  (void)pvParameters;
  
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_cfg.tx_pin_id, CAN_cfg.rx_pin_id, TWAI_MODE_NORMAL);

  twai_timing_config_t t_config;
  get_timing_config(CAN_cfg.speed, &t_config);

  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  lua_State *L = lua_getstate();

  //Install TWAI driver
  if (ESP_OK != twai_driver_install(&g_config, &t_config, &f_config)) {
      luaL_error( L, "failed to install CAN driver" );
      return;
  }

  //Start TWAI driver
  if (ESP_OK != twai_start()) {
      luaL_error( L, "failed to start CAN driver" );
      return;
  }

  twai_message_t message;
  for (;;){
      //receive next CAN message from queue
      if (twai_receive(&message, pdMS_TO_TICKS(10000)) == ESP_OK) {
          twai_message_t *postMessage = (twai_message_t *)malloc( sizeof( twai_message_t ) );
          memcpy(postMessage, &message, sizeof( twai_message_t ));
          task_post_medium( can_data_task_id, (task_param_t)postMessage );
      }
  }
}

// Lua: setup( {}, callback )
static int can_setup( lua_State *L )
{
  if(xCanTaskHandle != NULL)
    luaL_error( L, "stop CAN before setup" );
  luaL_checktable (L, 1);

  luaL_checkfunction (L, 2);
  lua_settop (L, 2);
  if(can_on_received != LUA_NOREF) luaL_unref(L, LUA_REGISTRYINDEX, can_on_received);
  can_on_received = luaL_ref(L, LUA_REGISTRYINDEX);

  lua_getfield (L, 1, "speed");
  CAN_cfg.speed = luaL_checkint(L, -1);
  lua_getfield (L, 1, "tx");
  CAN_cfg.tx_pin_id = luaL_checkint(L, -1);
  lua_getfield (L, 1, "rx");
  CAN_cfg.rx_pin_id = luaL_checkint(L, -1);
  lua_getfield (L, 1, "dual_filter");
  CAN_cfg.dual_filter = lua_toboolean(L, 0);
  lua_getfield (L, 1, "code");
  CAN_cfg.code = (uint32_t)luaL_optnumber(L, -1, 0);
  lua_getfield (L, 1, "mask");
  CAN_cfg.mask = (uint32_t)luaL_optnumber(L, -1, 0x0ffffffff);
  return 0;
}

static int can_start( lua_State *L )
{
  if(xCanTaskHandle != NULL)
    luaL_error( L, "CAN started" );
  xTaskCreate(task_CAN, "CAN", 4096, NULL, ESP_TASK_MAIN_PRIO + 1, &xCanTaskHandle);
  return 0;
}

static int can_stop( lua_State *L )
{
  if(xCanTaskHandle) {
    vTaskDelete(xCanTaskHandle);
    xCanTaskHandle = NULL;
  }
  //Stop TWAI driver
  if (ESP_OK != twai_stop()) {
      luaL_error( L, "failed to stop CAN driver" );
      return 0;
  }
  /*Uninstall TWAI driver
  if (ESP_OK != twai_driver_uninstall()) {
      luaL_error( L, "Failed to uninstall CAN driver" );
      return;
  }
  //*/
  return 0;
}

static int can_send( lua_State *L )
{
  uint32_t format = (uint8_t)luaL_checkinteger( L, 1 );
  uint32_t msg_id = luaL_checkinteger( L, 2 );
  size_t len;
  const char *data = luaL_checklstring( L, 3, &len );
  
  if(xCanTaskHandle == NULL)
    luaL_error( L, "CAN not started" );
  
  if(len > 8)
    luaL_error( L, "CAN can not send more than 8 bytes" );

  //Configure message to transmit
  twai_message_t message;
  message.identifier = msg_id;
  message.extd = format? 1 : 0;
  message.data_length_code = len;
  for (uint8_t i = 0; i < len; i++) {
      message.data[i] = data[i];
  }

  //Queue message for transmission
  if (ESP_OK != twai_transmit(&message, pdMS_TO_TICKS(1000))) {
      luaL_error( L, "failed to queue message for transmission" );
  }

  return 0;
}

// Module function map
LROT_BEGIN(can, NULL, 0)
  LROT_FUNCENTRY( setup,          can_setup )
  LROT_FUNCENTRY( start,          can_start )
  LROT_FUNCENTRY( stop,           can_stop )
  LROT_FUNCENTRY( send,           can_send )
  LROT_NUMENTRY ( STANDARD_FRAME, 0 )
  LROT_NUMENTRY ( EXTENDED_FRAME, 1 )
LROT_END(can, NULL, 0)

int luaopen_can( lua_State *L ) {
  can_data_task_id = task_get_id( can_data_task ); // reset CAN after sw reset
  twai_stop();
  return 0;
}

NODEMCU_MODULE(CAN, "can", can, luaopen_can);
