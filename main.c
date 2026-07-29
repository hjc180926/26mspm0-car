/**
 * @file    main.c
 * @brief   8路循迹 + 按键计时
 */

#include "ti_msp_dl_config.h"
#include "led/led.h"
#include "tb6612/tb6612.h"
#include "encoder/encoder.h"
#include "oled/oled.h"
#include "tracking/tracking.h"
#include "buzzer/buzzer.h"
#include <stdio.h>
#include <stdint.h>

static volatile uint32_t g_tick_1ms = 0;

/* ── 循迹参数 ── */
#define TRK_STRAIGHT   35
#define TRK_SMALL      30
#define TRK_MID        25
#define TRK_BIG         20

#define BTN_U11 (DL_GPIO_readPins(GPIO_BTN_U11_PORT, GPIO_BTN_U11_PIN_BTN_U11_PIN)==0)

static uint32_t g_tmr_start=0;
static bool     g_tmr_on=false;

static void prvSetupHardware(void)
{
    SYSCFG_DL_init();
    led_init(); led1_on(); led2_on();
    buzzer_init();
    tracking_init();
    tb6612_init();
    encoder_init();
}

void TIMER_0_INST_IRQHandler(void)
{
    if(DL_TimerG_getPendingInterrupt(TIMER_0_INST)==DL_TIMER_IIDX_ZERO)
        g_tick_1ms++;
}

void GROUP1_IRQHandler(void)
{
    if(encoder_right_int_is_pending()) encoder_right_irq_handler();
}

int main(void)
{
    prvSetupHardware();
    DL_TimerG_stopCounter(TIMER_0_INST);
    DL_TimerG_setLoadValue(TIMER_0_INST,79999U);

    NVIC_SetPriority(TIMER_0_INST_INT_IRQN,3);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    DL_TimerG_startCounter(TIMER_0_INST);

    OLED_Init();
    uint32_t le=g_tick_1ms,lo=g_tick_1ms;
    uint32_t last_btn=0;
    bool pb9_was=false;

    while(1){
        uint32_t n=g_tick_1ms;

        /* PB9 计时启停 */
        if((int32_t)(n-last_btn)>=10){ last_btn+=10;
            bool pb9=BTN_U11;
            if(pb9&&!pb9_was){
                g_tmr_on=!g_tmr_on;
                if(g_tmr_on) g_tmr_start=n;
            }
            pb9_was=pb9;
        }

        while((int32_t)(n-le)>=10){ le+=10;
            /* 8路循迹 侧边优先 */
            static int16_t lx=0,rx=0;
            uint8_t m=tracking_read_all();
            uint8_t c0=(m>>0)&1,c1=(m>>1)&1,c2=(m>>2)&1,c3=(m>>3)&1;
            uint8_t c4=(m>>4)&1,c5=(m>>5)&1,c6=(m>>6)&1,c7=(m>>7)&1;
            uint8_t any=c0|c1|c2|c3|c4|c5|c6|c7;

            if(!any){tb6612_set_speed(lx,rx);}
            /* 左传感器→降右轮(左转), 右传感器→降左轮(右转) [电机线左右反] */
            else if(c0){lx=TRK_STRAIGHT;rx=TRK_BIG;tb6612_set_speed(lx,rx);}       /* PB25 大左 */
            else if(c7){lx=TRK_BIG;rx=TRK_STRAIGHT;tb6612_set_speed(lx,rx);}       /* PB10 大右 */
            else if(c1){lx=TRK_STRAIGHT;rx=TRK_MID;tb6612_set_speed(lx,rx);}       /* PB18 中左 */
            else if(c6){lx=TRK_MID;rx=TRK_STRAIGHT;tb6612_set_speed(lx,rx);}       /* PB1  中右 */
            else if(c2){lx=TRK_STRAIGHT;rx=TRK_SMALL;tb6612_set_speed(lx,rx);}     /* PB21 小左 */
            else if(c5){lx=TRK_SMALL;rx=TRK_STRAIGHT;tb6612_set_speed(lx,rx);}     /* PB0  小右 */
            else if(c3||c4){lx=TRK_STRAIGHT;rx=TRK_STRAIGHT;tb6612_set_speed(lx,rx);} /* PB22/PA30 直行 */
        }

        if((int32_t)(n-lo)>=100){ lo+=100;
            uint8_t tk=tracking_read_all();
            uint32_t t=g_tmr_on?(n-g_tmr_start):0;
            OLED_Clear();
            OLED_vsprint(0,0,16,"T:%d%d%d%d%d%d%d%d",
                (tk>>0)&1,(tk>>1)&1,(tk>>2)&1,(tk>>3)&1,
                (tk>>4)&1,(tk>>5)&1,(tk>>6)&1,(tk>>7)&1);
            OLED_vsprint(0,16,16,"Time:%lu.%lus %s",
                (unsigned long)(t/1000),(unsigned long)((t%1000)/100),
                g_tmr_on?"ON":"OFF");
            OLED_Refresh(); }
    }
}
