/**
 * @file bt_assobio.c
 * @author Jaime Albuquerque (jaime.albq@gmail.com)
 * @brief HAL for bluetooth network
 * @version 0.1
 * @date 2023-01-29
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "bt_assobio.h"

static bt_assobio_stm32_parameters_s *_bt_parameters;
uint8_t assobio_mode = 0;

osMessageQueueId_t bt_receive_queue_handle;
osMessageQueueId_t bt_intern_queue_handle;

extern CRC_HandleTypeDef hcrc;

assobio_error_t bt_assobio_init(bt_assobio_stm32_parameters_s *parameter)
{
    _bt_parameters = parameter;

    bt_receive_queue_handle = osMessageQueueNew(BT_QUEUE_SIZE, BT_QUEUE_ITEM_SIZE, NULL);

    if (_bt_parameters == NULL || bt_receive_queue_handle == NULL) {
        return ASSOBIO_NO_PARAMETER;
    }

    bt_assobio_enable();

    /* initiate Bluetooth module */

    /* set default parameters */
    bt_assobio_default_parameters();

    return ASSOBIO_OK;
}

assobio_error_t bt_assobio_denit()
{
    /* deinit bluetooth module */

    bt_assobio_disable();
    _bt_parameters = NULL;

    return ASSOBIO_OK;
}

assobio_error_t bt_assobio_enable()
{
    _bt_parameters->en = ASSOBIO_ENABLE;
    bt_assobio_send_cmd(NRF52_CMD_ENABLE);
    return ASSOBIO_OK;
}
assobio_error_t bt_assobio_disable()
{
    _bt_parameters->en = ASSOBIO_DISABLE;
    bt_assobio_send_cmd(NRF52_CMD_DISABLE);
    return ASSOBIO_OK;
}

assobio_error_t bt_assobio_default_parameters()
{
    if (_bt_parameters == NULL) {
        return ASSOBIO_NO_PARAMETER;
    }

    // if (_bt_parameters->en != ASSOBIO_ENABLE) {
    //     return ASSOBIO_NOT_ENABLED;
    // }

    /* Default WiFi */
    strcpy(_bt_parameters->parameters.wifi[0].ssid, BT_DEFAULT_WIFI_SSID);
    strcpy(_bt_parameters->parameters.wifi[0].password, BT_DEFAULT_WIFI_PASSWORD);
    _bt_parameters->parameters.wifi_device_id = BT_DEFAULT_WIFI_DEVICE_ID;

    /* Default Bluetooth */
    strcpy(_bt_parameters->parameters.bt[0].ssid, BT_DEFAULT_BLUETOOTH_SSID);
    strcpy(_bt_parameters->parameters.bt_device_id, BT_DEFAULT_DEVICE_BLUETOOTH_ID);

    /* Default Location Safe Point */
    _bt_parameters->parameters.location_safe_point[0].x1 = BT_DEFAULT_SAFE_AREA_POINT_X1;
    _bt_parameters->parameters.location_safe_point[0].y1 = BT_DEFAULT_SAFE_AREA_POINT_Y1;
    _bt_parameters->parameters.location_safe_point[0].x2 = BT_DEFAULT_SAFE_AREA_POINT_X2;
    _bt_parameters->parameters.location_safe_point[0].y2 = BT_DEFAULT_SAFE_AREA_POINT_Y2;
    _bt_parameters->parameters.location_safe_point[1].x1 = BT_DEFAULT_SAFE_AREA_POINT_X3;
    _bt_parameters->parameters.location_safe_point[1].y1 = BT_DEFAULT_SAFE_AREA_POINT_Y3;
    _bt_parameters->parameters.location_safe_point[1].x2 = BT_DEFAULT_SAFE_AREA_POINT_X4;
    _bt_parameters->parameters.location_safe_point[1].y2 = BT_DEFAULT_SAFE_AREA_POINT_Y4;

    /* Default MQTT */
    strcpy(_bt_parameters->parameters.mqtt[0].username, BT_DEFAULT_MQTT_USERNAME);
    strcpy(_bt_parameters->parameters.mqtt[0].password, BT_DEFAULT_MQTT_PASSWORD);
    strcpy(_bt_parameters->parameters.mqtt[0].domain, BT_DEFAULT_MQTT_DOMAIN);
    _bt_parameters->parameters.mqtt[0].port = BT_DEFAULT_MQTT_PORT;
    strcpy(_bt_parameters->parameters.mqtt[0].labling_topic, BT_DEFAULT_MQTT_LABELING_TOPIC);
    strcpy(_bt_parameters->parameters.mqtt[0].config_topic, BT_DEFAULT_MQTT_CONFIG_TOPIC);

    /* Default Operation */
    _bt_parameters->parameters.op = BT_OP_SAFE;

    /* Default MPU */
    _bt_parameters->parameters.mpu[0].accel = BT_ACCEL_8G;
    _bt_parameters->parameters.mpu[0].gyro = BT_GYRO_500DEG;
    _bt_parameters->parameters.mpu[0].filter_bandwidth = BT_FB_5HZ;
    _bt_parameters->parameters.mpu[0].sample_interval = 100;

    return ASSOBIO_OK;
}

/* SSID - Ainda não entendi o que foi requerido */

assobio_error_t bt_assobio_pair_mode()
{
    // if (_bt_parameters->en != ASSOBIO_ENABLE) {
    //     return ASSOBIO_NOT_ENABLED;
    // }

    /* Request to pair the device */
    return bt_assobio_send_cmd(NRF52_CMD_PAIR_MODE_ON);
}

assobio_error_t bt_assobio_parameters_transmit()
{
    // if (_bt_parameters->en != ASSOBIO_ENABLE) {
    //     return ASSOBIO_NOT_ENABLED;
    // }

    bt_assobio_send_cmd(NRF52_CMD_SET_PARAMETERS);
    return bt_send_uart((uint8_t *) &_bt_parameters->parameters, sizeof(bt_assobio_config_param_s));
}

assobio_error_t bt_assobio_parameters_recieve()
{
    // if (_bt_parameters->en != ASSOBIO_ENABLE) {
    //     return ASSOBIO_NOT_ENABLED;
    // }

    /* Checking by sending command */

    bt_assobio_send_cmd(NRF52_CMD_GET_PARAMETERS);
    osStatus_t status = osMessageQueueGet(bt_intern_queue_handle, &_bt_parameters->parameters, NULL, 1000);

    if (status != osOK) {
    	return ASSOBIO_UNKOWN_ERROR;
    }

    return ASSOBIO_OK;
}

bt_safe_area_t bt_assobio_check_safe_area()
{
    // if (_bt_parameters->en != ASSOBIO_ENABLE) {
    //     return ASSOBIO_NOT_ENABLED;
    // }

    /* Checking by sending command */
    bt_safe_area_t safe = 0;
    bt_assobio_send_cmd(NRF52_CMD_CHECK_SAFE_AREA);
    osMessageQueueGet(bt_intern_queue_handle, &safe, NULL, 1000);
    
    return safe;
}

assobio_error_t bt_assobio_send_cmd(uint8_t cmd)
{
    // if (_bt_parameters->en != ASSOBIO_ENABLE) {
    //     return ASSOBIO_NOT_ENABLED;
    // }

    return bt_send_uart(&cmd, 1);
}

assobio_error_t bt_send_uart(uint8_t *data, uint16_t len)
{
    // if (_bt_parameters->en != ASSOBIO_ENABLE) {
    //     return ASSOBIO_NOT_ENABLED;
    // }

    // Packet structure:
    // 0: start byte
    // 1 to 2: length of the payload = n
    // 3 to (n+3): payload
    // n+3: CRC8
    // n+4: end byte

    uint16_t total_len = len + 5;
    uint8_t buff[total_len];
    buff[0] = 0x24;
    memcpy(&buff[1], &len, sizeof(uint16_t));
    memcpy(&buff[3], data, len);
    buff[total_len - 2] = HAL_CRC_Calculate(&hcrc, (uint32_t *) data, len);
    buff[total_len - 1] = 0x04;

    HAL_StatusTypeDef ret = HAL_UART_Transmit(_bt_parameters->uart, buff, total_len, HAL_MAX_DELAY);
    if (ret == HAL_OK) {
        return ASSOBIO_OK;
    } else {
        return ASSOBIO_COMMUNICATION_ERROR;
    }
}

void bt_receive_uart(void *argument)
{
    // if (_bt_parameters->en != ASSOBIO_ENABLE) {
    //     return ASSOBIO_NOT_ENABLED;
    // }

    // Packet structure:
    // 0: start byte
    // 1 to 2: length of the payload = n
    // 3 to (n+3): payload
    // n+3: CRC8
    // n+4: end byte

//	char print_buff[64];
//	sprintf(print_buff, "Started Received UART BT\r\n");
//	print_debug(print_buff);

    uint16_t total_len = 6;//sizeof(bt_assobio_config_param_s) + 5;
//    uint16_t payload_len = 0;
    uint8_t buff[total_len];
//    uint8_t temp;
//    uint16_t index = 0;
//    bool start = false;
//    bool stop = false;

//	sprintf(print_buff, "Heap size: %d bytes\r\n", xPortGetFreeHeapSize());
//	print_debug(print_buff);

    while (true) {
    	while (osMessageQueueGet(bt_receive_queue_handle, buff, NULL, BT_UART_TIMEOUT) == osOK) {
    		// if (!(start || stop)) {
//				sprintf(print_buff, "buff: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\r\n", buff[0], buff[1], buff[2], buff[3], buff[4], buff[5]);
//				print_debug(print_buff);
			// 	osMessageQueuePut(bt_intern_queue_handle, &buff[3], 0, 0);
			// 	stop = true;
			// 	start = true;
			// 	payload_len = 1;
    		// }

            // if (start && stop) {
                if (/* payload_len == 1 &&  */buff[3] == NRF52_CMD_MODE_ROTULACAO_START) {
                    assobio_mode = 1;
                    // payload_len = 0;
                }

                if (/* payload_len == 1 &&  */buff[3] == NRF52_CMD_MODE_ROTULACAO_STOP) {
                    assobio_mode = 0;
                    // payload_len = 0;
                }
            // }
        }
    }
}

