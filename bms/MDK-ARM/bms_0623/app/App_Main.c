#include "App_Main.h"

/**
 * @brief can接收任务
 * 该任务负责初始化CAN通信，并循环接收CAN总线上的数据
 * 根据接收到的CAN ID，处理不同的电池状态信息
 */
void can_recv_task(void *arg)
{
    printf("can_recv_task\r\n");  // 打印任务启动信息
    uint8_t buff[8];  // 定义CAN数据接收缓冲区
    uint16_t len;     // 定义接收数据长度变量
    Int_Can_Init();   // 初始化CAN接口

    uint16_t can_id;  // 定义CAN ID变量
    while (1)  // 无限循环，持续接收CAN数据
    {

        vTaskDelay(100);  // 任务延时100ms
        len = Int_Can_Recv(&can_id, buff, 8);  // 接收CAN数据，最多8字节
        if (len > 0)  // 如果接收到数据
        {
            printf("can recv: status: %d\r\n", buff[0]);  // 打印接收状态
            printf("can recv: can_id: %d\r\n", can_id);   // 打印CAN ID
            if (can_id == can_id_discharge)  // 如果是放电ID
            {
                App_Batman_SetDischargeState(buff[0]);  // 设置放电状态
            }
            else if (can_id == can_id_charge)  // 如果是充电ID
            {
                App_Batman_SetChargeState(buff[0]);  // 设置充电状态
            }
        }
    }
}

/**
 * @brief 电池管理任务
 * 该任务负责初始化电池管理系统，并周期性地采集电池各项参数
 * 包括电压、电流、温度和SOC等
 */
void batman_task(void *arg)
{

    App_Batman_Init();  // 初始化电池管理系统
    vTaskDelay(100);    // 任务延时100ms
    // Int_BQ769_WakeUp();  // 唤醒BQ769芯片（已注释）

    while (1)  // 无限循环，周期性采集电池参数
    {
        App_Batman_loadCellsVoltage();    // 采集电芯电压
        App_Batman_loadBatVoltage();      // 采集电池总电压
        App_Batman_loadCurrent();         // 采集电池电流
        App_Batman_loadTemperature();     // 采集电池温度
        App_Batman_loadBatSocPercent();    // 采集电池SOC百分比



        vTaskDelay(5000);  // 任务延时5秒
        // printf("batman task .......\r\n");  // 打印任务运行信息（已注释）
        vTaskDelay(5000);
        //  printf("batman task .......\r\n");
    }
}

/**
 * @brief 电池均衡任务
 */
void cell_balance_task(void *arg)
{
    vTaskDelay(10 * 1000);
    while (1)
    {
        App_Batman_CellBalance();
        vTaskDelay(60 * 1000);
    }
}

/**
 * @brief 屏幕展示任务
 */
void display_task(void *arg)
{
    App_Display_Init();
    while (1)
    {
        App_Display_Show();
        vTaskDelay(5000);
    }
}

/**
 * @brief 启动任务
 * @param
 * @return void
 */
void App_Main(void)
{

    // 任务1
    xTaskCreate(batman_task, "batman_task", 512, NULL, 1, NULL);

    // 任务2
    xTaskCreate(display_task, "display_task", 512, NULL, 1, NULL);
    // 任务3
    //  xTaskCreate(cell_balance_task, "cell_balance_task", 512, NULL, 2, NULL);

    // 任务 4  can 接收任务
    xTaskCreate(can_recv_task, "can_recv_task", 512, NULL, 3, NULL);

    vTaskStartScheduler();
}
