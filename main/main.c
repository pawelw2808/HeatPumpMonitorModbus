
#include "main.h"

void app_main(void)
{
printf("Hello World");
INV_InverterInit();
ReadRandomDataFrom_Modbus();
// delay_between_commands();
// INV_TurnOnMotor(1);
// vTaskDelay(3000);
// INV_TurnOffMotor();
//sprawdzam pushna server

}
