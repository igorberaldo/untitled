# Bluetooth
Implementation of bluetooth library

## Communication packet structure
`$<command length><command><crc8><end byte>`
<br>
`$<payload length><payload><crc8><end byte>`
<br>
Where:
- `$` is the starting byte [0x24]
- `<command or payload length>` is the length of the content only
- `[<command>]` is the command transferred
- `[<payload>]` is the content (optional)
- `<crc>` is the CRC8 of the payload (dependent of payload)
- `<end byte>` is the end of transmission byte [0x04]

## CRC
Please, use the following CRC algorithm:
```
uint8_t gencrc(const uint8_t *data, uint16_t length)
{
    uint8_t crc = 0x00;
    uint8_t extract;
    uint8_t sum;
    for (int i = 0; i < length; i++) {
        extract = *data;
        for (uint8_t tempI = 8; tempI; tempI--) {
            sum = (crc ^ extract) & 0x01;
            crc >>= 1;
            if (sum)
                crc ^= 0x8C;
            extract >>= 1;
        }
        data++;
    }
    return crc;
}
```

## How to perform the test
Peform a step over test using the STM32CubeIDE.

At the [line 92 in the main.c](Core/Src/main.c#L92), the function will copy the bluetooth structure and use it to do operations. It will also start the parameters in the default mode.
```
bt_assobio_init(&bluetooth);
```

When the function `bt_assobio_pair_mode()` is called at [line 93](Core/Src/main.c#L93) it will send the command `NRF52_CMD_PAIR_MODE_ON`. The list of commands can be found in [nrf52832_api.h](Core/Inc/nrf52832_api.h). In the UART line you are going to see the following packet: `0x24 0x85 0xB3 0x04`.
```
bt_assobio_pair_mode();
```

Then in the [line 94](Core/Src/main.c#L94) all the default parameters are going to be transmitted, and it is going to be large.
```
bt_assobio_parameters_transmit();
```

**ATTENTION TO THE STRUCTURE ALIGNMENT**: the data stracture is being compressed by the attribute packed, so the compiler will make it smaller as possible.

And the last test is a loop requesting time to time to check if we are in a safe area. [Line 101](Core/Src/main.c#L101).
```
bt_safe_area_t safe = bt_assobio_check_safe_area();
uint16_t delay_time;
if (safe) {
    delay_time = 15000;
} else {
    delay_time = 1;
}

HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
HAL_Delay(delay_time);
```

If all the test pass, so the implementation is good.

## Modo Rotulação
Para entrar em modo Rotulção é necessário que o nRF52 consiga se conectar com o dispositivo e que ele retorne o comando [`NRF52_CMD_MODE_ROTULACAO_START`](Core/components/nrf52832_api.h#13) (0x91) para o STM32.

Assim que o STM32 recebe o comando, ele inicia o modo de Rotução enviando constantemente o os dados das componentes de cada sensor, que estão estruturada da seguinte forma: [`bt_assobio_bmx160_data_s`](Core/components/bt_assobio.h#175)
```
typedef struct __attribute__((__packed__)) {
    sBmx160SensorData_t                         magn;
    sBmx160SensorData_t                         gyro;
    sBmx160SensorData_t                         accel;
} bt_assobio_bmx160_data_s;
```
Onde [`sBmx160SensorData_t`](Core/components/bmx160.h#700):
```
typedef struct __attribute__((__packed__)) {
    float x;           /**< X-axis sensor data */
    float y;           /**< Y-axis sensor data */
    float z;           /**< Z-axis sensor data */
    uint32_t sensortime; /**< sensor time */
}sBmx160SensorData_t;
```
Assim que o nRF52 envia o comando [`NRF52_CMD_MODE_ROTULACAO_STOP`](Core/components/nrf52832_api.h#14), o envio é encerrado.

## TODO
- [ ] **Implement Mode Manager**: create the task or delete it depending on the events.
  - [*] Rotulação: Comando de início e fim
- [ ] Change the implementation of UART5 from HAL to LL, so it can be faster
- [ ] Implement EEPROM feature
- [*] Flash nRF52832
- [*] Make a structure which nRF52832 and STM32 can exchange data
- [*] Make a list of commands for communication
