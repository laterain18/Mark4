#include "BT.h"

// 全局变量定义
SystemState BT_Flag = STATE_LOCKED;
ParamMode param_mode = PARAM_IDLE;

// 移除旧的缓冲区定义和索引
// char uart1_rx_buffer[256] = {0};
// uint16_t uart_rx_index = 0;

// 示例参数变量定义
int8_t param_int8_example = 10;
uint8_t param_uint8_example = 200;
int16_t param_int16_example = -1000;
uint16_t param_uint16_example = 5000;
int32_t param_int32_example = -50000;
uint32_t param_uint32_example = 100000;
float param_float_example = 1.5f;

// PID参数示例
float param_roll_kp = 1.2f;
float param_roll_ki = 0.05f;
float param_roll_kd = 0.1f;
float param_pitch_kp = 1.1f;
float param_pitch_ki = 0.04f;
float param_pitch_kd = 0.09f;

// 参数数据库
ParamEntry param_table[] = {

    // TEST
    {"speedlf", &speed_LF, PARAM_TYPE_FLOAT, 0.0f, 10.0f, 0.01f},
    {"speedrf", &speed_RF, PARAM_TYPE_FLOAT, 0.0f, 10.0f, 0.01f},
    {"speedlb", &speed_LB, PARAM_TYPE_FLOAT, 0.0f, 10.0f, 0.01f},
    {"speedrb", &speed_RB, PARAM_TYPE_FLOAT, 0.0f, 10.0f, 0.01f},

    // ==================== 角速度环（内环）参数 ====================
    // 横滚角速度环
    {"rgp", &Drone_Pid.roll_gyro.params.Kp, PARAM_TYPE_FLOAT, -0.5f, 0.5f, 0.001f},
    {"rgi", &Drone_Pid.roll_gyro.params.Ki, PARAM_TYPE_FLOAT, 0.0f, 0.0f, 0.001f},
    {"rgd", &Drone_Pid.roll_gyro.params.Kd, PARAM_TYPE_FLOAT, -0.5f, 0.5f, 0.001f},

    // 俯仰角速度环
    {"pgp", &Drone_Pid.pitch_gyro.params.Kp, PARAM_TYPE_FLOAT, -0.5f, 0.5f, 0.001f},
    {"pgi", &Drone_Pid.pitch_gyro.params.Ki, PARAM_TYPE_FLOAT, 0.0f, 0.0f, 0.001f},
    {"pgd", &Drone_Pid.pitch_gyro.params.Kd, PARAM_TYPE_FLOAT, -0.5f, 0.5f, 0.001f},

    // 偏航角速度环
    {"ygp", &Drone_Pid.yaw_gyro.params.Kp, PARAM_TYPE_FLOAT, -0.5f, 0.5f, 0.001f},
    {"ygi", &Drone_Pid.yaw_gyro.params.Ki, PARAM_TYPE_FLOAT, 0.0f, 0.0f, 0.001f},
    {"ygd", &Drone_Pid.yaw_gyro.params.Kd, PARAM_TYPE_FLOAT, -0.5f, 0.5f, 0.001f},

    // ==================== 角度环（外环）参数 ====================
    // 横滚角度环
    {"rap", &Drone_Pid.roll_angle.params.Kp, PARAM_TYPE_FLOAT, -10.0f, 10.0f, 0.01f},
    {"rai", &Drone_Pid.roll_angle.params.Ki, PARAM_TYPE_FLOAT, 0.0f, 0.0f, 0.001f},
    {"rad", &Drone_Pid.roll_angle.params.Kd, PARAM_TYPE_FLOAT, -10.0f, 10.0f, 0.01f},

    // 俯仰角度环
    {"pap", &Drone_Pid.pitch_angle.params.Kp, PARAM_TYPE_FLOAT, -10.0f, 10.0f, 0.01f},
    {"pai", &Drone_Pid.pitch_angle.params.Ki, PARAM_TYPE_FLOAT, 0.0f, 0.0f, 0.001f},
    {"pad", &Drone_Pid.pitch_angle.params.Kd, PARAM_TYPE_FLOAT, -10.0f, 10.0f, 0.01f},

    // 偏航角度环
    {"yap", &Drone_Pid.yaw_angle.params.Kp, PARAM_TYPE_FLOAT, -10.0f, 10.0f, 0.01f},
    {"yai", &Drone_Pid.yaw_angle.params.Ki, PARAM_TYPE_FLOAT, 0.0f, 0.0f, 0.001f},
    {"yad", &Drone_Pid.yaw_angle.params.Kd, PARAM_TYPE_FLOAT, -10.0f, 10.0f, 0.01f},

    // 结束标记
    {"", NULL, PARAM_TYPE_FLOAT, 0.0f, 0.0f, 0.0f}
};

// 当前调参状态
static char current_param_name[30] = {0};
static void *current_param_ptr = NULL;
static ParamDataType current_param_type;
static float current_step = 0.1f;
static float original_min, original_max;

// 内部函数
static void BT_State_Machine_Handler(const char *command);
static void BT_Param_Adjustment_Handler(const char *command);
static void BT_Adjust_Param_Value(int8_t direction);
static void BT_Display_Param_Value(void);

/**
 * @brief 初始化蓝牙解析器
 */
void BT_Parser_Init(void)
{
    BT_Flag = STATE_LOCKED;
    param_mode = PARAM_IDLE;
    
    // 从Flash加载保存的参数
    BT_Load_Parameters_From_Flash();
    
    BT_Send_Response("BT Ready");
}

/**
 * @brief 处理接收到的蓝牙数据（使用指针传递）
 * @param data_buffer: 接收到的数据缓冲区指针
 * @param data_length: 数据长度
 */
void BT_Parser_Process(uint8_t *data_buffer, uint16_t data_length)
{
    if (data_buffer == NULL || data_length == 0) return;
    
    // 确保字符串以null结尾
    if (data_length > 0) {
        data_buffer[data_length] = '\0';  // 添加字符串结束符
    }
    
    // 清理字符串：移除换行符、回车符等
    char *command = (char*)data_buffer;
    int len = data_length;
    
    // 移除首部空白字符
    while (len > 0 && (*command == '\r' || *command == '\n' || *command == ' ')) {
        command++;
        len--;
    }
    
    // 移除尾部空白字符
    while (len > 0 && (command[len-1] == '\r' || command[len-1] == '\n' || command[len-1] == ' ')) {
        command[--len] = '\0';
    }
    
    if (len == 0) {
        return;
    }
    
    // 回声测试（可选）
    // char echo_msg[150];
    // snprintf(echo_msg, sizeof(echo_msg), "CMD: %s (Len:%d)", command, len);
    // BT_Send_Response(echo_msg);
    
    // 根据模式处理指令
    if (param_mode != PARAM_IDLE) {
        BT_Param_Adjustment_Handler(command);
    } else {
        BT_State_Machine_Handler(command);
    }
}

/**
 * @brief 状态机处理函数
 */
static void BT_State_Machine_Handler(const char *command)
{
    if (strcmp(command, "lock") == 0) {
        if (BT_Flag != STATE_TAKEOFF) {
            BT_Flag = STATE_LOCKED;
            BT_Send_Response("OK: System locked");
        } else {
            BT_Send_Response("Error: Cannot lock while flying");
        }
    } 
    else if (strcmp(command, "unlock") == 0) {
        if (BT_Flag != STATE_TAKEOFF) {
            BT_Flag = STATE_UNLOCKED;
            BT_Send_Response("OK: System unlocked");
        } else {
            BT_Send_Response("Error: Cannot unlock while flying");
        }
    }
    else if (strcmp(command, "start") == 0) {
        if (BT_Flag == STATE_UNLOCKED) {
            BT_Flag = STATE_TAKEOFF;
            BT_Send_Response("OK: Takeoff Soon !!");
        } else {
            BT_Send_Response("Error: Must unlock before takeoff");
        }
    }
    else if (strcmp(command, "stop") == 0) {
        if (BT_Flag == STATE_TAKEOFF) {
            BT_Flag = STATE_LANDING;
            BT_Send_Response("OK: Landing Soon !!");
            
            // 3秒后自动锁定
            // HAL_Delay(3000);
            // BT_Flag = STATE_LOCKED;
            // BT_Send_Response("OK: Landing completed, system locked");
        } else {
            BT_Send_Response("Error: Not in takeoff state");
        }
    }
    else if (strcmp(command, "debug") == 0) {
        if (BT_Flag != STATE_TAKEOFF) {
            BT_Flag = STATE_LOCKED;
            param_mode = PARAM_WAIT_NAME;
            BT_Send_Response("DEBUG: Enter parameter");
        } else {
            BT_Send_Response("Error: Cannot debug while flying");
        }
    }
    else if (strcmp(command, "list") == 0) {
        BT_Send_Response("Available parameters:");
        for (int i = 0; strlen(param_table[i].name) > 0; i++) {
            char list_msg[50];
            snprintf(list_msg, sizeof(list_msg), "- %s", param_table[i].name);
            BT_Send_Response(list_msg);
        }
    }
    else if (strcmp(command, "status") == 0) {
        char status_msg[50];
        snprintf(status_msg, sizeof(status_msg), "STATUS: BT_Flag=%d", BT_Flag);
        BT_Send_Response(status_msg);
    }
    else {
        char error_msg[100];
        snprintf(error_msg, sizeof(error_msg), "Error: Unknown command: %s", command);
        BT_Send_Response(error_msg);
        BT_Send_Response("Valid: lock, unlock, start, stop, debug, list, status");
    }
}

/**
 * @brief 参数调整处理函数（优化版，支持直接切换参数）
 */
static void BT_Param_Adjustment_Handler(const char *command)
{
    // 安全检查：确保不在起飞状态
    if (BT_Flag == STATE_TAKEOFF) {
        BT_Send_Response("Error: Cannot adjust parameters while flying");
        param_mode = PARAM_IDLE;
        return;
    }
    
    // 首先检查是否为有效参数名（无论在 WAIT_NAME 还是 ADJUSTING 状态）
    int param_found = 0;
    for (int i = 0; strlen(param_table[i].name) > 0; i++) {
        if (strcmp(command, param_table[i].name) == 0) {
            // 找到参数，更新当前调参状态
            current_param_ptr = param_table[i].value_ptr;
            strcpy(current_param_name, param_table[i].name);
            current_param_type = param_table[i].data_type;
            current_step = param_table[i].step_size;
            original_min = param_table[i].min_value;
            original_max = param_table[i].max_value;
            param_mode = PARAM_ADJUSTING; // 确保处于调整状态
            param_found = 1;
            
            // 显示当前参数值
            BT_Display_Param_Value();
            BT_Send_Response("Adjust: + - dot+ dot- save exit");
            return; // 找到参数后直接返回
        }
    }
    
    // 如果在 WAIT_NAME 状态，处理特殊命令
    if (param_mode == PARAM_WAIT_NAME) {
        if (strcmp(command, "exit") == 0) {
            param_mode = PARAM_IDLE;
            BT_Send_Response("DEBUG: Exit parameter mode");
            return;
        }
        
        if (strcmp(command, "list") == 0) {
            BT_Send_Response("Available parameters:");
            for (int i = 0; strlen(param_table[i].name) > 0; i++) {
                char list_msg[50];
                snprintf(list_msg, sizeof(list_msg), "- %s", param_table[i].name);
                BT_Send_Response(list_msg);
            }
            return;
        }
        
        if (!param_found) {
            BT_Send_Response("Error: Parameter not found");
        }
    }
    // 如果在 ADJUSTING 状态，处理调整命令
    else if (param_mode == PARAM_ADJUSTING) {
        if (strcmp(command, "+") == 0) {
            BT_Adjust_Param_Value(1); // 增加
            BT_Display_Param_Value();
        }
        else if (strcmp(command, "-") == 0) {
            BT_Adjust_Param_Value(-1); // 减少
            BT_Display_Param_Value();
        }
        else if (strcmp(command, "dot+") == 0) {
            current_step *= 10.0f;
            if (current_step > 1000.0f) current_step = 1000.0f;
            char step_msg[50];
            snprintf(step_msg, sizeof(step_msg), "STEP: %.4f", current_step);
            BT_Send_Response(step_msg);
        }
        else if (strcmp(command, "dot-") == 0) {
            current_step /= 10.0f;
            if (current_step < 0.0001f) current_step = 0.0001f;
            char step_msg[50];
            snprintf(step_msg, sizeof(step_msg), "STEP: %.4f", current_step);
            BT_Send_Response(step_msg);
        }
        else if (strcmp(command, "save") == 0) {
            BT_Save_Parameters_To_Flash();
            BT_Send_Response("SAVED: All parameters stored in flash");
        }
        else if (strcmp(command, "exit") == 0) {
            param_mode = PARAM_IDLE;
            BT_Send_Response("DEBUG: Exit parameter mode");
        }
        else if (!param_found) {
            // 如果不是参数名，也不是有效命令，报错
            BT_Send_Response("Error: Unknown command or parameter");
            BT_Send_Response("  Usage: + - dot+ dot- save exit");
            BT_Send_Response("  Or enter parameter name to switch");
        }
        // 如果 param_found 为 1，说明输入的是有效参数名，已经在上面处理过了
    }
}

/**
 * @brief 调整参数值
 */
static void BT_Adjust_Param_Value(int8_t direction)
{
    switch (current_param_type) {
        case PARAM_TYPE_INT8: {
            int8_t *val = (int8_t *)current_param_ptr;
            int8_t new_val = *val + (int8_t)(direction * current_step);
            if (new_val >= original_min && new_val <= original_max) {
                *val = new_val;
            }
            break;
        }
        case PARAM_TYPE_UINT8: {
            uint8_t *val = (uint8_t *)current_param_ptr;
            int16_t new_val = (int16_t)*val + (int16_t)(direction * current_step);
            if (new_val >= original_min && new_val <= original_max) {
                *val = (uint8_t)new_val;
            }
            break;
        }
        case PARAM_TYPE_INT16: {
            int16_t *val = (int16_t *)current_param_ptr;
            int16_t new_val = *val + (int16_t)(direction * current_step);
            if (new_val >= original_min && new_val <= original_max) {
                *val = new_val;
            }
            break;
        }
        case PARAM_TYPE_UINT16: {
            uint16_t *val = (uint16_t *)current_param_ptr;
            int32_t new_val = (int32_t)*val + (int32_t)(direction * current_step);
            if (new_val >= original_min && new_val <= original_max) {
                *val = (uint16_t)new_val;
            }
            break;
        }
        case PARAM_TYPE_INT32: {
            int32_t *val = (int32_t *)current_param_ptr;
            int32_t new_val = *val + (int32_t)(direction * current_step);
            if (new_val >= original_min && new_val <= original_max) {
                *val = new_val;
            }
            break;
        }
        case PARAM_TYPE_UINT32: {
            uint32_t *val = (uint32_t *)current_param_ptr;
            int64_t new_val = (int64_t)*val + (int64_t)(direction * current_step);
            if (new_val >= original_min && new_val <= original_max) {
                *val = (uint32_t)new_val;
            }
            break;
        }
        case PARAM_TYPE_FLOAT: {
            float *val = (float *)current_param_ptr;
            float new_val = *val + direction * current_step;
            if (new_val >= original_min && new_val <= original_max) {
                *val = new_val;
            }
            break;
        }
    }
}

/**
 * @brief 显示当前参数值
 */
static void BT_Display_Param_Value(void)
{
    char value_str[100];
    
    switch (current_param_type) {
        case PARAM_TYPE_INT8:
            snprintf(value_str, sizeof(value_str), "%s=%d", current_param_name, 
                    *(int8_t *)current_param_ptr);
            break;
        case PARAM_TYPE_UINT8:
            snprintf(value_str, sizeof(value_str), "%s=%u", current_param_name, 
                    *(uint8_t *)current_param_ptr);
            break;
        case PARAM_TYPE_INT16:
            snprintf(value_str, sizeof(value_str), "%s=%d", current_param_name, 
                    *(int16_t *)current_param_ptr);
            break;
        case PARAM_TYPE_UINT16:
            snprintf(value_str, sizeof(value_str), "%s=%u", current_param_name, 
                    *(uint16_t *)current_param_ptr);
            break;
        case PARAM_TYPE_INT32:
            snprintf(value_str, sizeof(value_str), "%s=%ld", current_param_name, 
                    *(int32_t *)current_param_ptr);
            break;
        case PARAM_TYPE_UINT32:
            snprintf(value_str, sizeof(value_str), "%s=%lu", current_param_name, 
                    *(uint32_t *)current_param_ptr);
            break;
        case PARAM_TYPE_FLOAT:
            snprintf(value_str, sizeof(value_str), "%s=%.4f", current_param_name, 
                    *(float *)current_param_ptr);
            break;
        default:
            snprintf(value_str, sizeof(value_str), "%s=UNKNOWN", current_param_name);
            break;
    }
    
    BT_Send_Response(value_str);
}

/**
 * @brief 发送响应到蓝牙
 */
void BT_Send_Response(const char *response)
{
    char full_response[120];
    snprintf(full_response, sizeof(full_response), "%s\r\n", response);
    HAL_UART_Transmit(&huart1, (uint8_t*)full_response, strlen(full_response), HAL_MAX_DELAY);
}
/**
 * @brief 保存所有参数到Flash
 */
void BT_Save_Parameters_To_Flash(void)
{
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError = 0;
    HAL_StatusTypeDef status;
    
    // 解锁Flash
    HAL_FLASH_Unlock();
    
    // 配置擦除
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.Banks = FLASH_BANK_1;
    EraseInitStruct.Sector = PARAM_FLASH_SECTOR;
    EraseInitStruct.NbSectors = 1;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    
    // 擦除Flash扇区
    status = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
    if (status != HAL_OK) {
        HAL_FLASH_Lock();
        BT_Send_Response("Error: Flash erase failed");
        return;
    }
    
    // 计算参数数量
    uint32_t param_count = 0;
    for (; strlen(param_table[param_count].name) > 0; param_count++);
    
    // 写入参数数量作为起始标记
    uint32_t start_marker = 0xDEADBEEF;
    uint32_t address = PARAM_FLASH_ADDRESS;
    
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, start_marker);
    if (status == HAL_OK) address += 4;
    
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, param_count);
    if (status == HAL_OK) address += 4;
    
    // 写入每个参数的值
    for (uint32_t i = 0; i < param_count; i++) {
        uint32_t param_value = 0;
        
        // 根据类型获取参数值的字节表示
        switch (param_table[i].data_type) {
            case PARAM_TYPE_INT8:
            case PARAM_TYPE_UINT8:
                param_value = *(uint8_t *)param_table[i].value_ptr;
                break;
            case PARAM_TYPE_INT16:
            case PARAM_TYPE_UINT16:
                param_value = *(uint16_t *)param_table[i].value_ptr;
                break;
            case PARAM_TYPE_INT32:
            case PARAM_TYPE_UINT32:
                param_value = *(uint32_t *)param_table[i].value_ptr;
                break;
            case PARAM_TYPE_FLOAT:
                param_value = *(uint32_t *)param_table[i].value_ptr; // 直接复制float的字节
                break;
        }
        
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, param_value);
        if (status != HAL_OK) break;
        address += 4;
    }
    
    HAL_FLASH_Lock();
    
    if (status == HAL_OK) {
        BT_Send_Response("OK: Parameters saved to flash");
    } else {
        BT_Send_Response("Error: Flash programming failed");
    }
}

/**
 * @brief 从Flash加载参数
 */
void BT_Load_Parameters_From_Flash(void)
{
    uint32_t address = PARAM_FLASH_ADDRESS;
    
    // 检查起始标记
    uint32_t start_marker = *(__IO uint32_t*)address;
    if (start_marker != 0xDEADBEEF) {
        BT_Send_Response("INFO: No saved parameters found");
        return;
    }
    
    address += 4;
    uint32_t param_count = *(__IO uint32_t*)address;
    address += 4;
    
    // 验证参数数量
    uint32_t actual_count = 0;
    for (; strlen(param_table[actual_count].name) > 0; actual_count++);
    
    if (param_count > actual_count) {
        param_count = actual_count;
    }
    
    // 加载每个参数
    for (uint32_t i = 0; i < param_count; i++) {
        uint32_t stored_value = *(__IO uint32_t*)address;
        
        if (stored_value != 0xFFFFFFFF) { // 检查是否为擦除状态
            switch (param_table[i].data_type) {
                case PARAM_TYPE_INT8:
                    *(int8_t *)param_table[i].value_ptr = (int8_t)stored_value;
                    break;
                case PARAM_TYPE_UINT8:
                    *(uint8_t *)param_table[i].value_ptr = (uint8_t)stored_value;
                    break;
                case PARAM_TYPE_INT16:
                    *(int16_t *)param_table[i].value_ptr = (int16_t)stored_value;
                    break;
                case PARAM_TYPE_UINT16:
                    *(uint16_t *)param_table[i].value_ptr = (uint16_t)stored_value;
                    break;
                case PARAM_TYPE_INT32:
                    *(int32_t *)param_table[i].value_ptr = (int32_t)stored_value;
                    break;
                case PARAM_TYPE_UINT32:
                    *(uint32_t *)param_table[i].value_ptr = stored_value;
                    break;
                case PARAM_TYPE_FLOAT:
                    *(uint32_t *)param_table[i].value_ptr = stored_value; // 直接复制字节
                    break;
            }
        }
        address += 4;
    }
    
    BT_Send_Response("OK: Parameters loaded from flash");
}

/**
 * @brief 获取当前状态
 */
SystemState BT_Get_Current_State(void)
{
    return BT_Flag;
}
