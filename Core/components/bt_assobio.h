/**
 * @file bt_assobio.h
 * @author Jaime Albuquerque (jaime.albq@gmail.com)
 * @brief HAL for bluetooth network
 * @version 0.1
 * @date 2023-01-29
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#if !defined(__BT_ASSOBIO_H__)
#define __BT_ASSOBIO_H__

#include <inttypes.h>
#include <string.h>

#include "stm32l0xx.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "queue.h"

#include "assobio_error_handler.h"
#include "nrf52832_api.h"
#include "bmx160.h"

/* Default values */
#define BT_NUMBER_OF_LOCATION_SAFE_POINT        8u
#define BT_DEFAULT_SAFE_AREA_POINT_X1           -8.063724
#define BT_DEFAULT_SAFE_AREA_POINT_Y1           -34.874457
#define BT_DEFAULT_SAFE_AREA_POINT_X2           -8.063626
#define BT_DEFAULT_SAFE_AREA_POINT_Y2           -34.873666
#define BT_DEFAULT_SAFE_AREA_POINT_X3           -8.063257
#define BT_DEFAULT_SAFE_AREA_POINT_Y3           -34.873765
#define BT_DEFAULT_SAFE_AREA_POINT_X4           -8.063341
#define BT_DEFAULT_SAFE_AREA_POINT_Y4           -34.874406

#define BT_UART_TIMEOUT                         15000u

/* WiFi */
#define BT_NUMBER_OF_SAFE_WIFI                  4u
#define BT_DEFAULT_WIFI_DEVICE_ID               0x00E5FC84u
#define BT_DEFAULT_WIFI_SSID                    "ASSOBIO\0"
#define BT_DEFAULT_WIFI_SSID_LENGTH             32u
#define BT_DEFAULT_WIFI_PASSWORD                "123456789\0"
#define BT_DEFAULT_WIFI_PASSWORD_LENGTH         32u

/* Bluetooth */
#define BT_NUMBER_OF_SAFE_BLUETOOTH             4u
#define BT_DEFAULT_BLUETOOTH_SSID               "iPhone de João\0"
#define BT_DEFAULT_BLUETOOTH_SSID_LENGTH        32u
#define BT_DEFAULT_DEVICE_BLUETOOTH_ID          "A01\0"
#define BT_DEFAULT_DEVICE_BLUETOOTH_ID_LENGTH   32u

/* MQTT */
#define BT_NUMBER_OF_MQTT_BROKER                2u

#define BT_MQTT_USERNAME_LENGTH                 32u
#define BT_MQTT_PASSWORD_LENGTH                 32u
#define BT_MQTT_DOMAIN_LENGTH                   55u
#define BT_MQTT_LABELING_TOPIC_LENGTH           55u
#define BT_MQTT_CONFIG_TOPIC_LENGTH             55u


#define BT_DEFAULT_MQTT_USERNAME                "emqx\0"
#define BT_DEFAULT_MQTT_PASSWORD                "public\0"
#define BT_DEFAULT_MQTT_DOMAIN                  "broker.emqx.io\0"
#define BT_DEFAULT_MQTT_PORT                    1883U
#define BT_DEFAULT_MQTT_LABELING_TOPIC          "labeling-entry\0"
#define BT_DEFAULT_MQTT_CONFIG_TOPIC            "config\0"

/* Operation */
#define BT_DEFAULT_OPERATION_MODE               0U

/* MPU */
#define BT_NUMBER_OF_MPU_SETTINGS               2u
#define BT_DEFAULT_MPU_ACCEL                    2u
#define BT_DEFAULT_MPU_GYROS                    1u
#define BT_DEFAULT_MPU_BANDWIDTH                6u
#define BT_DEFAULT_MPU_SAMPLES                  100u

/* Queue */
#define BT_QUEUE_SIZE                           10u
#define BT_QUEUE_ITEM_SIZE                      sizeof(uint8_t) * 6
extern osMessageQueueId_t bt_receive_queue_handle;
extern uint8_t assobio_mode;

/* Tasks */

/* Enum */
typedef enum __attribute__((__packed__)) {
    BT_OP_SAFE = 0,
    BT_OP_ALERT,
    BT_OP_CONFIG
} bt_assobio_op_mode_t;

typedef enum __attribute__((__packed__)) {
    BT_ACCEL_2G = 0,
    BT_ACCEL_4G,
    BT_ACCEL_8G,
    BT_ACCEL_16G
} bt_assobio_accel_range_t;

typedef enum __attribute__((__packed__)) {
    BT_GYRO_250DEG = 0,
    BT_GYRO_500DEG,
    BT_GYRO_1000DEG,
    BT_GYRO_2000DEG
} bt_assobio_gyro_range_t;

typedef enum __attribute__((__packed__)) {
    BT_FB_260HZ = 0,
    BT_FB_184HZ,
    BT_FB_94HZ,
    BT_FB_44HZ,
    BT_FB_21HZ,
    BT_FB_10HZ,
    BT_FB_5HZ
} bt_assobio_filter_bandwidth_t;

typedef enum __attribute__((__packed__)) {
    BT_NOT_SAFE = 0,
    BT_SAFE
} bt_safe_area_t;

/* Strutures */
typedef struct __attribute__((__packed__)) {
    char   ssid[BT_DEFAULT_WIFI_SSID_LENGTH];
    char   password[BT_DEFAULT_WIFI_PASSWORD_LENGTH];
} bt_assobio_wifi_parameters_s;

typedef struct __attribute__((__packed__)) {
    char   ssid[BT_DEFAULT_BLUETOOTH_SSID_LENGTH];
} bt_assobio_bt_parameters_s;

typedef struct __attribute__((__packed__)) {
    double  x1;
    double  y1;
    double  x2;
    double  y2;
} bt_assobio_location_safe_point_parameters_s;

typedef struct __attribute__((__packed__)) {
    char        username[BT_MQTT_USERNAME_LENGTH];
    char        password[BT_MQTT_PASSWORD_LENGTH];
    char        domain[BT_MQTT_DOMAIN_LENGTH];
    uint16_t    port;
    char        labling_topic[BT_MQTT_LABELING_TOPIC_LENGTH];
    char        config_topic[BT_MQTT_CONFIG_TOPIC_LENGTH];
} bt_assobio_mqtt_parameters_s;

typedef struct __attribute__((__packed__)) {
    bt_assobio_accel_range_t        accel;
    bt_assobio_gyro_range_t         gyro;
    bt_assobio_filter_bandwidth_t   filter_bandwidth;
    uint16_t                        sample_interval;
} bt_assobio_mpu_parameters_s;


typedef struct __attribute__((__packed__)) {
    bt_assobio_wifi_parameters_s                wifi[BT_NUMBER_OF_SAFE_WIFI];
    uint32_t                                    wifi_device_id;
    bt_assobio_bt_parameters_s                  bt[BT_NUMBER_OF_SAFE_BLUETOOTH];
    char                                        bt_device_id[BT_DEFAULT_DEVICE_BLUETOOTH_ID_LENGTH];
    bt_assobio_location_safe_point_parameters_s location_safe_point[BT_NUMBER_OF_LOCATION_SAFE_POINT];
    bt_assobio_mqtt_parameters_s                mqtt[BT_NUMBER_OF_MQTT_BROKER];
    bt_assobio_op_mode_t                        op;
    bt_assobio_mpu_parameters_s                 mpu[BT_NUMBER_OF_MPU_SETTINGS];
} bt_assobio_config_param_s;

typedef struct __attribute__((__packed__)) {
    sBmx160SensorData_t                         magn;
    sBmx160SensorData_t                         gyro;
    sBmx160SensorData_t                         accel;
} bt_assobio_bmx160_data_s;

typedef struct __attribute__((__packed__)) {
    bt_assobio_config_param_s                   parameters;
    assobio_enabling_t                          en;
    UART_HandleTypeDef                          *uart;
    // uint8_t                                     *buff;
    // xQueueHandle                                queue;
} bt_assobio_stm32_parameters_s;

/* Functions */

assobio_error_t bt_assobio_init(bt_assobio_stm32_parameters_s *parameter);
assobio_error_t bt_assobio_denit(); // Change name later

assobio_error_t bt_assobio_enable();
assobio_error_t bt_assobio_disable();

assobio_error_t bt_assobio_default_parameters();

assobio_error_t bt_assobio_pair_mode();
assobio_error_t bt_assobio_parameters_transmit();
assobio_error_t bt_assobio_parameters_recieve();
bt_safe_area_t  bt_assobio_check_safe_area();
assobio_error_t bt_assobio_send_cmd(uint8_t cmd);
assobio_error_t bt_send_uart(uint8_t *data, uint16_t len);
void bt_receive_uart(void *argument);

uint8_t gencrc(const uint8_t *data, uint16_t length);

#endif // __BT_ASSOBIO_H__
