//
// Created by 26757 on 2025/12/29.
//
#include <algorithm>
#include <cstring>
#include "task_public.h"
#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "Gimbal.h"
#include "queue.h"

// 左下角为正方向
struct TransmitPackage {
    float imu_angles[3];       // yaw, pitch, roll, in rad
    float yaw_imu_angle;       // end point yaw angle in rad
    float pitch_imu_angle;     // end point pitch angle in rad
    float yaw_motor_angle;     // yaw motor angle in rad
    float pitch_motor_angle;   // pitch motor angle in rad
    uint8_t laser_enabled;     // 0: disabled, 1: enabled
    uint8_t enabled;           // 0: disabled, 1: enabled
    uint8_t stability_enabled; // 0: disabled, 1: enabled
    uint8_t check_sum;         // checksum
} transmit_package;

struct ReceivePackage {
    float yaw_speed;           // in rpm
    float pitch_speed;         // in rpm
    uint8_t laser_enabled;     // 0: disable, 1: enable, other: no action
    uint8_t enabled;           // 0: disable, 1: enable, other: no action
    uint8_t stability_enabled; // 0: disable, 1: enable, other: no action
    uint8_t check_sum;         // checksum
};

xQueueHandle receive_package_queue;

extern float INS_angle[3]; // yaw,pitch,roll
extern Gimbal gimbal;      // 云台
extern QD4310 YawMotor;    // 云台偏航电机
extern QD4310 PitchMotor;  // 云台俯仰电机

volatile uint8_t UART6_RxBuffer[sizeof(ReceivePackage)]; // volatile: DMA异步访问,防止编译器优化

void StartTransmitTask(void *argument) {
    while (true) {
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS) {}
        transmit_package.laser_enabled = (__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1) > 0) ? 1 : 0;
        transmit_package.enabled = gimbal.enabled;
        transmit_package.stability_enabled = gimbal.stability_enabled;
        transmit_package.imu_angles[0] = INS_angle[0];
        transmit_package.imu_angles[1] = INS_angle[1];
        transmit_package.imu_angles[2] = INS_angle[2];
        transmit_package.yaw_imu_angle = gimbal.yaw_imu_angle;
        transmit_package.pitch_imu_angle = gimbal.pitch_imu_angle;
        transmit_package.yaw_motor_angle = YawMotor.angle;
        transmit_package.pitch_motor_angle = PitchMotor.angle;
        // 计算校验和
        uint8_t checksum = 0;
        for (size_t i = 0; i < sizeof(TransmitPackage) - sizeof(uint8_t); ++i) {
            checksum += reinterpret_cast<uint8_t *>(&transmit_package)[i];
        }
        transmit_package.check_sum = checksum;
        HAL_UART_Transmit_DMA(&huart6, reinterpret_cast<const uint8_t *>(&transmit_package), sizeof(TransmitPackage));
    }
}

void StartReceiveTask(void *argument) {
    receive_package_queue = xQueueCreate(5, sizeof(ReceivePackage));
    ReceivePackage receive_package{};
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, (uint8_t *)UART6_RxBuffer, sizeof(ReceivePackage));
    while (true) {
        xQueueReceive(receive_package_queue, &receive_package, portMAX_DELAY);
        // 校验和 — 必须在 queued 数据上计算（UART6_RxBuffer 已被 ISR 清零）
        uint8_t checksum = 0;
        for (size_t i = 0; i < sizeof(ReceivePackage) - sizeof(uint8_t); ++i) {
            checksum += reinterpret_cast<const uint8_t *>(&receive_package)[i];
        }
        if (checksum == receive_package.check_sum) {
            if (receive_package.laser_enabled == 0 || receive_package.laser_enabled == 1)
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,
                                      receive_package.laser_enabled ? 999 : 0);
            if (receive_package.enabled == 0 || receive_package.enabled == 1) {
                receive_package.enabled ? gimbal.enable() : gimbal.disable();
            }
            if (receive_package.stability_enabled == 0 || receive_package.stability_enabled == 1) {
                receive_package.stability_enabled ? gimbal.enable_stability() : gimbal.disable_stability();
            }
            gimbal.Ctrl(std::clamp(receive_package.yaw_speed, -50.0f, 50.0f),
                        std::clamp(receive_package.pitch_speed, -50.0f, 50.0f));
        }
    }
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    BaseType_t xHigherPriorityTaskWoken;
    if (huart->Instance == huart6.Instance) {
        if (Size == sizeof(ReceivePackage)) {
            xQueueSendToBackFromISR(receive_package_queue, (const void *)UART6_RxBuffer, &xHigherPriorityTaskWoken);
            if (xHigherPriorityTaskWoken) {
                taskYIELD();
            }
        }
        HAL_UARTEx_ReceiveToIdle_DMA(&huart6, (uint8_t *)UART6_RxBuffer, sizeof(ReceivePackage));
    }
}
