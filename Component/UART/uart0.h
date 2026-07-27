/**
 * @file    uart0.h
 * @brief   UART0 调试串口模块接口定义（裸机版）
 */
#ifndef UART0_H
#define UART0_H

#include <stdint.h>

void uart0_init(void);
void uart0_sendStr(const char *str);

#endif /* UART0_H */
