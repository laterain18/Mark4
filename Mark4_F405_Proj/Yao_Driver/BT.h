#ifndef BT_H
#define BT_H

#include "main.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

// 状态定义
typedef enum {
    STATE_LOCKED = -1,
    STATE_UNLOCKED = 0, 
    STATE_TAKEOFF = 1,
    STATE_LANDING = 9
} SystemState;

// 调参模式状态机
typedef enum {
    PARAM_IDLE,           // 空闲状态
    PARAM_WAIT_NAME,      // 等待参数名
    PARAM_ADJUSTING       // 调整参数值
} ParamMode;

// 参数数据类型定义
typedef enum {
    PARAM_TYPE_INT8,
    PARAM_TYPE_UINT8,
    PARAM_TYPE_INT16,
    PARAM_TYPE_UINT16,
    PARAM_TYPE_INT32,
    PARAM_TYPE_UINT32,
    PARAM_TYPE_FLOAT
} ParamDataType;

// 参数结构体
typedef struct {
    char name[30];           // 参数名称
    void *value_ptr;         // 参数值指针
    ParamDataType data_type; // 参数类型
    float min_value;         // 最小值
    float max_value;         // 最大值
    float step_size;         // 步长
} ParamEntry;

// 函数声明
void BT_Parser_Init(void);
void BT_Parser_Process(uint8_t *data_buffer, uint16_t data_length);  // 修改：接收指针和长度
void BT_Send_Response(const char *response);
SystemState BT_Get_Current_State(void);
void BT_Save_Parameters_To_Flash(void);
void BT_Load_Parameters_From_Flash(void);

// 外部变量声明
extern SystemState BT_Flag;
extern ParamMode param_mode;

// 移除旧的缓冲区定义
// extern char uart1_rx_buffer[256]; 

// Flash存储地址定义
#define PARAM_FLASH_SECTOR    FLASH_SECTOR_11
#define PARAM_FLASH_ADDRESS   0x080E0000  // 根据实际芯片调整

#endif