#if !defined(__NRF52832_H__)
#define __NRF52832_H__

/* Commands to nRF52832 */
#define NRF52_CMD_ENABLE                0x81
#define NRF52_CMD_DISABLE               0x82
#define NRF52_CMD_GET_PARAMETERS        0x83
#define NRF52_CMD_SET_PARAMETERS        0x84
#define NRF52_CMD_PAIR_MODE_ON          0x85
#define NRF52_CMD_PAIR_MODE_OFF         0x86
#define NRF52_CMD_CHECK_SAFE_AREA       0x87

#define NRF52_CMD_MODE_ROTULACAO_START		0x91
#define NRF52_CMD_MODE_ROTULACAO_STOP		0x92
#define NRF52_CMD_MODE_COLETANDO		0x93
#define NRF52_CMD_MODE_ALERTA			0x94
#define NRF52_CMD_MODE_CONFIG			0x95

#endif // __NRF52832_H__
