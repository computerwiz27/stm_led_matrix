#include "stm32f1xx_hal.h"

#define CONFIRM 0x0
#define ERROR   0x1 
#define RECEIVE 0x2 
#define SEND    0x3   
#define SET     0x4 
#define SET_ALL 0x5 
#define RESET   0x6
#define QUIT    0x7

struct _SERIAL_INTERFACE {
    uint8_t connected_flag;
    uint8_t expect_cmd_flag;
    uint8_t expect_conf_flag;
    uint8_t confirm_flag;
    uint8_t last_cmd;
    int command_size;
    uint8_t* rx_command_buf;
    uint8_t* tx_command_buf;
    int data_size;
    uint8_t* rx_data_buf;
};
typedef struct _SERIAL_INTERFACE SERIAL_INTERFACE;

void SerialInterface_Init(void);