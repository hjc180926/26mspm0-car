/**
 * @file    main.c
 * @brief   8路循迹 + 按键计时 + 编码器里程自动停车
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

volatile uint32_t g_tick_1ms = 0;

/* -- 循迹参数 -- */
#define TRK_STRAIGHT   35
#define TRK_SMALL      30
#define TRK_MID        25
#define TRK_BIG         20

/* -- 里程: avg*10/71 ~= avg*9231>>16 -- */
#define PULSES_TO_CM_X10(avg)  ((uint32_t)((avg) * 9231U) >> 16)
#define TARGET_CM_X10           6142

#define BTN_U11 (DL_GPIO_readPins(GPIO_BTN_U11_PORT, GPIO_BTN_U11_PIN_BTN_U11_PIN)==0)

static uint32_t g_tmr_start=0;
static bool     g_tmr_on=false;
static bool     g_run=false;
static uint32_t g_tmr_frozen=0;

static int32_t  g_enc_start_l = 0;
static int32_t  g_enc_start_r = 0;
static uint32_t g_dist_cm_x10 = 0;

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
    {
        DL_TimerG_clearInterruptStatus(TIMER_0_INST, DL_TIMER_IIDX_ZERO);
        g_tick_1ms++;
    }
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
                g_run=!g_run;
                if(g_run){
                    encoder_data_t enc; encoder_get_data(&enc);
                    g_enc_start_l = enc.left_count;
                    g_enc_start_r = enc.right_count;
                    g_dist_cm_x10 = 0;
                    g_tmr_on=true; g_tmr_start=n;
                } else {
                    g_tmr_frozen=n-g_tmr_start; g_tmr_on=false;
                    tb6612_set_speed(0,0);
                }
            }
            pb9_was=pb9;
        }

        while((int32_t)(n-le)>=10){ le+=10;
            static int16_t lx=0,rx=0;
            if(g_run){
            /* 编码器里程累加 */
            encoder_data_t enc; encoder_get_data(&enc);
            int32_t a = enc.left_count  - g_enc_start_l;
            int32_t b = enc.right_count - g_enc_start_r;
            if(a<0)a=-a; if(b<0)b=-b;
            uint32_t avg = ((uint32_t)a + (uint32_t)b) / 2U;
            g_dist_cm_x10 = PULSES_TO_CM_X10(avg);

            if(g_dist_cm_x10 >= TARGET_CM_X10){
                g_run=false; g_tmr_on=false; g_tmr_frozen=n-g_tmr_start;
                tb6612_set_speed(0,0);
            }

            /* 循迹逻辑 */
            if(g_run){
            uint8_t m=tracking_read_all();
            uint8_t c0=(m>>0)&1,c1=(m>>1)&1,c2=(m>>2)&1,c3=(m>>3)&1;
            uint8_t c4=(m>>4)&1,c5=(m>>5)&1,c6=(m>>6)&1,c7=(m>>7)&1;
            uint8_t any=c0|c1|c2|c3|c4|c5|c6|c7;

            if(!any){tb6612_set_speed(lx,rx);}
            else if(c0){lx=TRK_STRAIGHT;rx=TRK_BIG;tb6612_set_speed(lx,rx);}
            else if(c7){lx=TRK_BIG;rx=TRK_STRAIGHT;tb6612_set_speed(lx,rx);}
            else if(c1){lx=TRK_STRAIGHT;rx=TRK_MID;tb6612_set_speed(lx,rx);}
            else if(c6){lx=TRK_MID;rx=TRK_STRAIGHT;tb6612_set_speed(lx,rx);}
            else if(c2){lx=TRK_STRAIGHT;rx=TRK_SMALL;tb6612_set_speed(lx,rx);}
            else if(c5){lx=TRK_SMALL;rx=TRK_STRAIGHT;tb6612_set_speed(lx,rx);}
            else if(c3||c4){lx=TRK_STRAIGHT;rx=TRK_STRAIGHT;tb6612_set_speed(lx,rx);}
            } /* if(g_run) tracking guard */
            }
        }

        if((int32_t)(n-lo)>=100){ lo+=100;
            uint8_t tk=tracking_read_all();
            uint32_t t=g_tmr_on?(n-g_tmr_start):g_tmr_frozen;
            uint32_t cm = g_dist_cm_x10 / 10;
            uint32_t mm = g_dist_cm_x10 % 10;
            OLED_Clear();
            OLED_vsprint(0,0,16,"T:%d%d%d%d%d%d%d%d",
                (tk>>0)&1,(tk>>1)&1,(tk>>2)&1,(tk>>3)&1,
                (tk>>4)&1,(tk>>5)&1,(tk>>6)&1,(tk>>7)&1);
            OLED_vsprint(0,16,16,"Time:%lu.%lus %s",
                (unsigned long)(t/1000),(unsigned long)((t%1000)/100),
                g_tmr_on?"ON":"OFF");
            OLED_ShowNum(0,32,cm,4,16,1);
            OLED_ShowChar(4*8,32,'.',16,1);
            OLED_ShowNum(5*8,32,mm,1,16,1);
            OLED_Refresh(); }
    }
}
