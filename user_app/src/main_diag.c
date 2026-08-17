/*
 * 最小诊断固件 — 仅测试 UART0 打印 + LED 闪烁
 * 用法：把 main.c 暂时替换为此文件内容，编译烧录
 */

#include <stdio.h>
#include "board.h"

int main(void)
{
    /* 1. 只用 board_init()：时钟 + 时钟打印 + 控制台 */
    board_init();

    /* 2. 初始化 LED */
    board_init_led_pins();

    /* 3. 反复打印 + 闪烁 LED */
    uint32_t count = 0;
    while (1) {
        printf("Hello %lu\n", count++);
        board_led_toggle();
        board_delay_ms(500);
    }
    return 0;
}
