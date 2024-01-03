#include "inverter.h"

#include "string.h"
#include "esp_log.h"
#include "modbus_params.h" // for modbus parameters structures
#include "mbcontroller.h"
#include "sdkconfig.h"

#define SLAVE_ID 1

#define COMMAND_READ 0x3
#define COMMAND_WRITE 0x6

#define CONTROL_REGISTER_ADDRESS 0x2000
#define CURRENT_FREQ_ADRESS 0x2001

#define CONTROL_REGISTER_WORK_ROTATION_NOMINAL 1
#define CONTROL_REGISTER_WORK_ROTATION_REVERESE 2
#define CONTROL_REGISTER_WORK_STOP 5
#define CONTROL_REGISTER_ERROR_CANCEL 7

#define CONFIG_MB_COMM_MODE_RTU 1
#define MB_PORT_NUM (UART_NUM_2) // Number of UART port used for Modbus connection
#define MB_DEV_SPEED (19200)     // The communication speed of the UART

#define CONFIG_MB_UART_TXD 17
#define CONFIG_MB_UART_RXD 16
#define CONFIG_MB_UART_RTS 18

// The number of parameters that intended to be used in the particular control process
#define MASTER_MAX_CIDS num_device_parameters

// Number of reading of parameters from slave
#define MASTER_MAX_RETRY 30

// Timeout to update cid over Modbus
#define UPDATE_CIDS_TIMEOUT_MS (500)
#define UPDATE_CIDS_TIMEOUT_TICS (UPDATE_CIDS_TIMEOUT_MS / portTICK_PERIOD_MS)

// Timeout between polls
#define POLL_TIMEOUT_MS (1)
#define POLL_TIMEOUT_TICS (POLL_TIMEOUT_MS / portTICK_PERIOD_MS)

// The macro to get offset for parameter in the appropriate structure
#define HOLD_OFFSET(field) ((uint16_t)(offsetof(holding_reg_params_t, field) + 1))
#define INPUT_OFFSET(field) ((uint16_t)(offsetof(input_reg_params_t, field) + 1))
#define COIL_OFFSET(field) ((uint16_t)(offsetof(coil_reg_params_t, field) + 1))
// Discrete offset macro
#define DISCR_OFFSET(field) ((uint16_t)(offsetof(discrete_reg_params_t, field) + 1))

#define STR(fieldname) ((const char *)(fieldname))
// Options can be used as bit masks or parameter limits
#define OPTS(min_val, max_val, step_val)                   \
    {                                                      \
        .opt1 = min_val, .opt2 = max_val, .opt3 = step_val \
    }

esp_err_t error;
mb_param_request_t request;
uint8_t rcv_data[2];
int current_direction = CONTROL_REGISTER_WORK_ROTATION_NOMINAL;

const TickType_t delay_100_ms = 100 / portTICK_PERIOD_MS;

int current_freq = 1;

// uint16_t freq_value[11] = {0x0000, 0x01F4, 0x0320, 0x044C, 0x0578, 0x06A4, 0x07D0, 0x08FC, 0x0A8C, 0x0C1C, 0x0DAC}; //5-35Hz
uint16_t freq_value[11] = {0x0000, 0x07D0, 0x08CA, 0x09C4, 0x0ABE, 0x0BB8, 0x0CB2, 0x0DAC, 0x0EA7, 0x0FA0, 0x1004}; // 20-41Hz

uint8_t INV_GetInverterState(void);
void INV_SetUpperLimitFrq(uint16_t limit_of_freq);

/*
 *
 *   INIT
 *
 */
int INV_InverterInit(void)
{
    /*
     * Initialize and start Modbus controller
     */
    mb_communication_info_t comm = {
        .port = MB_PORT_NUM,
#if CONFIG_MB_COMM_MODE_ASCII
        .mode = MB_MODE_ASCII,
#elif CONFIG_MB_COMM_MODE_RTU
        .mode = MB_MODE_RTU,
#endif
        .baudrate = MB_DEV_SPEED,
        .parity = UART_PARITY_EVEN,

    };
    void *master_handler = NULL;

    esp_err_t err = mbc_master_init(MB_PORT_SERIAL_MASTER, &master_handler);
    if (master_handler == NULL)
        printf("mb controller initialization fail.\r\n");

    if (err != ESP_OK)
        printf("mb controller initialization fail, returns(0x%lx).\r\n", (uint32_t)err);

    err = mbc_master_setup((void *)&comm);
    if (err != ESP_OK)
        printf("mb controller setup fail, returns(0x%lx).\r\n", (uint32_t)err);

    // Set UART pin numbers
    err = uart_set_pin(MB_PORT_NUM, CONFIG_MB_UART_TXD, CONFIG_MB_UART_RXD,
                       CONFIG_MB_UART_RTS, UART_PIN_NO_CHANGE);
    if (err != ESP_OK)
        printf("mb serial set pin failure, uart_set_pin() returned (0x%lx).\r\n", (uint32_t)err);

    err = mbc_master_start();
    if (err != ESP_OK)
        printf("mb controller start fail, returns(0x%lx).\r\n", (uint32_t)err);

    // Set driver mode to Half Duplex
    err = uart_set_mode(MB_PORT_NUM, UART_MODE_RS485_HALF_DUPLEX);
    if (err != ESP_OK)
        printf("mb serial set mode failure, uart_set_mode() returned (0x%lx).\r\n", (uint32_t)err);

    vTaskDelay(delay_100_ms);
    printf("Modbus master stack initialized...\r\n");

    /*
     * Program Inverter EEPROM
     */
    // if(IsEepromProgramMode())
    // {
    // printf("EEPROM program mode...\r\n");
    // /* Program upper limit of the output frequency (2005H and 2006H) */
    // INV_SetUpperLimitFrq(4100);
    // }
    // else printf("EEPROM not need to be programed.\r\n");

    /*
     * Set controller to init state
     */
    INV_GetInverterState();
    vTaskDelay(delay_100_ms);
    vTaskDelay(delay_100_ms);
    vTaskDelay(delay_100_ms);
    vTaskDelay(delay_100_ms);
    vTaskDelay(delay_100_ms);
    vTaskDelay(delay_100_ms);
    vTaskDelay(delay_100_ms);
    vTaskDelay(delay_100_ms);
    vTaskDelay(delay_100_ms);
    vTaskDelay(delay_100_ms);
    printf("3.\r\n");
    vTaskDelay(delay_100_ms);
    vTaskDelay(delay_100_ms);
    vTaskDelay(delay_100_ms);
    vTaskDelay(delay_100_ms);
    vTaskDelay(delay_100_ms);
    vTaskDelay(delay_100_ms);
    vTaskDelay(delay_100_ms);
    vTaskDelay(delay_100_ms);
    vTaskDelay(delay_100_ms);
    vTaskDelay(delay_100_ms);
    printf("2.\r\n");
    printf("Turn off.\r\n");
    vTaskDelay(delay_100_ms);
    vTaskDelay(delay_100_ms);

    /* Turn off */
    INV_TurnOffMotor();
    vTaskDelay(delay_100_ms);
    vTaskDelay(delay_100_ms);
    INV_GetInverterState();
    vTaskDelay(delay_100_ms);
    vTaskDelay(delay_100_ms);

    /* Set freqency to 0 */
    INV_SetFreqValue(1);

    return INVERTER_INIT_SUCCESS;
} /* INIT */

/*
 * GET STATE
 */
uint8_t INV_GetInverterState(void)
{
    request.slave_addr = SLAVE_ID;
    request.command = COMMAND_READ;
    request.reg_start = 0x2100;
    request.reg_size = 1;

    error = mbc_master_send_request(&request, rcv_data);

    printf("SW 1 of the inverter: 0x%X - ", rcv_data[0]);
    if (rcv_data[0] == 0x3)
        printf("Stop\r\n");
    else if (rcv_data[0] == 0x4)
        printf("Error\r\n");
    else if (rcv_data[0] == 0x5)
        printf("POFF\r\n");
    else if (rcv_data[0] == 0x1)
        printf("Work nominal\r\n");
    else if (rcv_data[0] == 0x2)
        printf("Working reverse\r\n");

    return rcv_data[0];
}

/*
 * CHECK MODBUS RESPONSE
 */
int CheckMbResponse(esp_err_t error_to_handle)
{
    if (error_to_handle == ESP_OK)
    {
        printf("MB Send OK\r\n");
        return 0;
    }
    else
    {
        printf("MB Send Error - ");
        if (error_to_handle == ESP_ERR_INVALID_ARG)
            printf("ESP_ERR_INVALID_ARG - invalid argument of function\r\n");
        else if (error_to_handle == ESP_ERR_INVALID_RESPONSE)
            printf("ESP_ERR_INVALID_RESPONSE - an invalid response from slave\r\n");
        else if (error_to_handle == ESP_ERR_TIMEOUT)
            printf("ESP_ERR_TIMEOUT - operation timeout or no response from slave\r\n");
        else if (error_to_handle == ESP_ERR_NOT_SUPPORTED)
            printf("ESP_ERR_NOT_SUPPORTED - the request command is not supported by slave\r\n");
        else if (error_to_handle == ESP_FAIL)
            printf("ESP_FAIL - slave returned an exception or other failure\r\n");
        return 4;
    }
}

/*
 * TURN OFF MOTOR
 */
int INV_TurnOffMotor(void)
{
    printf("Turn off motor.\r\n");
    request.slave_addr = SLAVE_ID;
    request.command = COMMAND_WRITE;
    request.reg_start = CONTROL_REGISTER_ADDRESS;
    request.reg_size = 2;
    rcv_data[0] = CONTROL_REGISTER_WORK_STOP;
    rcv_data[1] = 0;

    error = mbc_master_send_request(&request, rcv_data);
    return CheckMbResponse(error);
}

/*
 * TURN ON MOTOR
 */
int INV_TurnOnMotor(int direction)
{
    printf("Turn on motor... ");
    request.slave_addr = SLAVE_ID;
    request.command = COMMAND_WRITE;
    request.reg_start = CONTROL_REGISTER_ADDRESS;
    request.reg_size = 2;
    if ((direction != 1) && (direction != 2))
        direction = 1;

    rcv_data[0] = direction;
    rcv_data[1] = 0;

    current_direction = direction;

    error = mbc_master_send_request(&request, rcv_data);

    printf("Rcv Data: 0x%X, 0x%X\r\n", rcv_data[1], rcv_data[0]);

    uint8_t repeat_cnt = 0;
    while (rcv_data[0] == 0xff)
    {
        if (repeat_cnt > 10)
            break;
        vTaskDelay(delay_100_ms);
        rcv_data[0] = direction;
        rcv_data[1] = 0;
        mbc_master_send_request(&request, rcv_data);
        printf("Retransmit nr: %d. Rcv Data: 0x%X, 0x%X\r\n", repeat_cnt, rcv_data[1], rcv_data[0]);
        repeat_cnt++;
    }

    INV_SetFreqValue(current_freq);

    return CheckMbResponse(error);
}

/* Return new direction */
int INV_ChangeDirection(void)
{
    if (current_direction == CONTROL_REGISTER_WORK_ROTATION_NOMINAL)
        current_direction = CONTROL_REGISTER_WORK_ROTATION_REVERESE;
    else
        current_direction = CONTROL_REGISTER_WORK_ROTATION_NOMINAL;

    INV_TurnOnMotor(current_direction);

    return current_direction;
}

/* Return new direction */
int INV_ChangeDirectionDuringPause(void)
{
    if (current_direction == CONTROL_REGISTER_WORK_ROTATION_NOMINAL)
        current_direction = CONTROL_REGISTER_WORK_ROTATION_REVERESE;
    else
        current_direction = CONTROL_REGISTER_WORK_ROTATION_NOMINAL;

    return current_direction;
}

/* Return current direstion */
int INV_GetDirection(void)
{
    return current_direction;
}

/* Set new frequency value */
int INV_SetFreqValue(uint8_t new_freq_value)
{
    if (new_freq_value <= 10)
    {
        printf("Try to set speed %d: ", new_freq_value);
        request.slave_addr = SLAVE_ID;
        request.command = COMMAND_WRITE;
        request.reg_start = 0x2001;
        request.reg_size = 2;
        rcv_data[0] = freq_value[new_freq_value]; // [1][0]
        rcv_data[1] = freq_value[new_freq_value] >> 8;

        printf("Send Data: 0x%X, 0x%X\r\n", rcv_data[1], rcv_data[0]);

        error = mbc_master_send_request(&request, rcv_data);
        current_freq = new_freq_value;

        printf("Rcv Data: 0x%X, 0x%X\r\n", rcv_data[1], rcv_data[0]);
        return 0;
    }
    else
        return 1;
}

/* Return new RPM value */
int INV_RpmUp(uint8_t rpm_to_increase)
{
    current_freq += rpm_to_increase;
    if (current_freq > 10)
        current_freq = 10;

    INV_SetFreqValue(current_freq);

    CheckMbResponse(error);
    return current_freq;
}

/* Return new RPM value */
int INV_RpmDown(uint8_t rpm_to_decrease)
{
    current_freq -= rpm_to_decrease;
    if ((current_freq > 10) || (current_freq < 1))
        current_freq = 1;

    INV_SetFreqValue(current_freq);

    CheckMbResponse(error);
    return current_freq;
}

/* Program upper limit of the output frequency (2005H and 2006H)  0x1004*/
void INV_SetUpperLimitFrq(uint16_t limit_of_freq)
{
    printf("\r\nTry to set upper limit of the output frequency: %d\r\n", limit_of_freq);
    request.slave_addr = SLAVE_ID;
    request.command = COMMAND_WRITE;
    // request.reg_start = 0x2005;
    request.reg_start = 0x0004;
    request.reg_size = 2;
    rcv_data[0] = limit_of_freq; // [1][0]
    rcv_data[1] = limit_of_freq >> 8;

    // rcv_data[0] = (0x1004); // [1][0]
    // rcv_data[1] = (0x1004 >> 8);

    printf("Send Data: 0x%X, 0x%X\r\n", rcv_data[1], rcv_data[0]);
    error = mbc_master_send_request(&request, rcv_data);
    CheckMbResponse(error);
    printf("Rcv Data: 0x%X, 0x%X\r\n", rcv_data[1], rcv_data[0]);

    // rcv_data[0] = limit_of_freq; // [1][0]
    // rcv_data[1] = limit_of_freq >> 8;
    // rcv_data[0] = (0x1004); // [1][0]
    // rcv_data[1] = (0x1004 >> 8);
    // printf("Send Data: 0x%X, 0x%X\r\n", rcv_data[1], rcv_data[0]);
    // // request.reg_start = 0x2006;
    // error = mbc_master_send_request(&request, rcv_data);
    // CheckMbResponse(error);

    // printf("Rcv Data: 0x%X, 0x%X\r\n\r\n", rcv_data[1], rcv_data[0]);
}

void delay_between_commands(void)
{
    vTaskDelay(delay_100_ms);
}