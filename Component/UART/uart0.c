/**
 * @file    uart0.c
 * @brief   UART0 调试串口模块（裸机版，无 FreeRTOS）
 */
#include <stdarg.h>
#include <stdio.h>
#include "UART/uart0.h"
#include "ti_msp_dl_config.h"

void uart0_sendStr(const char *str)
{
    while (*str) {
        DL_UART_transmitDataBlocking(UART_0_INST, (uint32_t)*str++);
    }
}

int _write(int fd, const char *buf, unsigned int size)
{
    (void)fd;
    for (unsigned int i = 0; i < size; i++) {
        DL_UART_transmitDataBlocking(UART_0_INST, (uint32_t)buf[i]);
    }
    return (int)size;
}

int write(int fd, const char *buf, unsigned int size)
{
    return _write(fd, buf, size);
}

void uart0_init(void)
{
    uint8_t dummy;
    while (DL_UART_receiveDataCheck(UART_0_INST, &dummy)) {}
    NVIC_SetPriority(UART_0_INST_INT_IRQN, 3);
    DL_UART_enableInterrupt(UART_0_INST, DL_UART_INTERRUPT_RX);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
}

void UART_0_INST_IRQHandler(void)
{
    DL_UART_IIDX iid = DL_UART_getPendingInterrupt(UART_0_INST);
    if (iid == DL_UART_IIDX_RX) {
        uint8_t byte = DL_UART_receiveData(UART_0_INST);
        (void)byte;
    }
}
