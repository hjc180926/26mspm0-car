/**
 * @file    main.c
 * @brief   8路循迹 + 按键计时 + 编码器里程自动停车
 */

#include "ti_msp_dl_config.h"
#include "led/led.h"
#include "tb6612/tb6612.h"
#include "UART/uart0.h"
#include "encoder/encoder.h"
#include "oled/oled.h"
#include "tracking/tracking.h"
#include "servo/servo.h"
#include "buzzer/buzzer.h"
#include <stdint.h>
#include <stdbool.h>

volatile uint32_t g_tick_1ms = 0;

/* -- 循迹参数 -- */
#define TRK_STRAIGHT   35
#define TRK_SMALL      30
#define TRK_MID        25
#define TRK_BIG         20

/* -- 里程 -- */
#define TARGET_CM_X10           6113  /* 614.16cm */

#define BTN_U11 (DL_GPIO_readPins(GPIO_BTN_U11_PORT, GPIO_BTN_U11_PIN_BTN_U11_PIN)==0)

static uint32_t g_tmr_start=0;
static bool     g_tmr_on=false;
static bool     g_run=false;
static uint32_t g_tmr_frozen=0;

static int32_t  g_enc_start_l = 0;
static int32_t  g_enc_start_r = 0;
static uint32_t g_dist_cm_x10 = 0;

static uint8_t  g_trk_display = 0;  /* OLED 显示用，避免重复读污染消抖 */
static int16_t  g_lx = 0, g_rx = 0;   /* 当前电机速度，OLED 显示用 */
static int16_t  g_dev = 0;            /* 当前偏差，调试用 */

static void prvSetupHardware(void)
{
    SYSCFG_DL_init();
    led_init(); led1_on(); led2_on();
    buzzer_init();
    tracking_init();
    uart0_init();
    tb6612_init();
    encoder_init();
    servo_init();
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
            if(g_run){
            /* 编码器里程累加 */
            encoder_data_t enc; encoder_get_data(&enc);
            int32_t a = enc.left_count  - g_enc_start_l;
            int32_t b = enc.right_count - g_enc_start_r;
            if(a<0)a=-a; if(b<0)b=-b;
            uint32_t avg = ((uint32_t)a + (uint32_t)b) / 2U;
            g_dist_cm_x10 = (uint32_t)((avg * 10U) / ENCODER_PULSES_PER_CM);

            if(g_dist_cm_x10 >= TARGET_CM_X10){
                g_run=false; g_tmr_on=false; g_tmr_frozen=n-g_tmr_start;
                tb6612_set_speed(0,0);
            }

            /* 循迹逻辑 */
            if(g_run){
            uint8_t m=tracking_read_all();
            g_trk_display = m;  /* 缓存给 OLED，避免二次调用污染消抖 */
            uint8_t c0=(m>>0)&1,c1=(m>>1)&1,c2=(m>>2)&1,c3=(m>>3)&1;
            uint8_t c4=(m>>4)&1,c5=(m>>5)&1,c6=(m>>6)&1,c7=(m>>7)&1;
            uint8_t any=c0|c1|c2|c3|c4|c5|c6|c7;

            if(!any){tb6612_set_speed(g_lx,g_rx);g_dev=0;}
            else if(c0){g_lx=TRK_BIG;g_rx=TRK_STRAIGHT;tb6612_set_speed(g_lx,g_rx);g_dev=-7;}
            else if(c7){g_lx=TRK_STRAIGHT;g_rx=TRK_BIG;tb6612_set_speed(g_lx,g_rx);g_dev= 7;}
            else if(c7){g_lx=TRK_STRAIGHT;g_rx=TRK_BIG;tb6612_set_speed(g_lx,g_rx);g_dev= 7;}
            else if(c7){g_lx=TRK_STRAIGHT;g_rx=TRK_BIG;tb6612_set_speed(g_lx,g_rx);g_dev= 7;}
            else if(c1){g_lx=TRK_MID;g_rx=TRK_STRAIGHT;tb6612_set_speed(g_lx,g_rx);g_dev=-5;}
            else if(c6){g_lx=TRK_STRAIGHT;g_rx=TRK_MID;tb6612_set_speed(g_lx,g_rx);g_dev= 5;}
            else if(c6){g_lx=TRK_STRAIGHT;g_rx=TRK_MID;tb6612_set_speed(g_lx,g_rx);g_dev= 5;}
            else if(c6){g_lx=TRK_STRAIGHT;g_rx=TRK_MID;tb6612_set_speed(g_lx,g_rx);g_dev= 5;}
            else if(c2){g_lx=TRK_SMALL;g_rx=TRK_STRAIGHT;tb6612_set_speed(g_lx,g_rx);g_dev=-3;}
            else if(c5){g_lx=TRK_STRAIGHT;g_rx=TRK_SMALL;tb6612_set_speed(g_lx,g_rx);g_dev= 3;}
            else if(c5){g_lx=TRK_STRAIGHT;g_rx=TRK_SMALL;tb6612_set_speed(g_lx,g_rx);g_dev= 3;}
            else if(c5){g_lx=TRK_STRAIGHT;g_rx=TRK_SMALL;tb6612_set_speed(g_lx,g_rx);g_dev= 3;}
            else if(c3||c4){g_lx=TRK_STRAIGHT;g_rx=TRK_STRAIGHT;tb6612_set_speed(g_lx,g_rx);g_dev=0;}
            } /* if(g_run) tracking guard */
            }
        }

        if((int32_t)(n-lo)>=100){ lo+=100;
            uint8_t tk=g_trk_display;  /* 使用缓存值，不再二次调用 */
            int16_t lx_d=g_lx, rx_d=g_rx;  /* 同时快照，避免与 tk 不同步 */
            int16_t dv_d=g_dev;
            uint32_t t=g_tmr_on?(n-g_tmr_start):g_tmr_frozen;
            uint32_t sec  = t / 1000;
            uint32_t ts   = (t % 1000) / 100;
            uint32_t cm   = g_dist_cm_x10 / 10;
            uint32_t mm   = g_dist_cm_x10 % 10;
            int      i;

            OLED_Clear();

            /* line 0: T:00000000 */
            OLED_ShowChar(0,0,'T',16,1);
            OLED_ShowChar(8,0,':',16,1);
            for(i=0;i<8;i++){
                OLED_ShowChar((uint8_t)((2+i)*8),0,
                    ((tk>>i)&1)?'1':'0',16,1);
            }

            /* line 1: Time:XXX.Xs */
            OLED_ShowString(0,16,"Time:",16,1);
            OLED_ShowNum(5*8,16,sec,3,16,1);
            OLED_ShowChar(8*8,16,'.',16,1);
            OLED_ShowNum(9*8,16,ts,1,16,1);
            OLED_ShowChar(10*8,16,'s',16,1);
            OLED_ShowString(12*8,16,g_tmr_on?" ON":"OFF",16,1);

            /* line 2: Dist:XXX.Xcm */
            OLED_ShowString(0,32,"Dist:",16,1);
            OLED_ShowNum(5*8,32,cm,4,16,1);
            OLED_ShowChar(9*8,32,'.',16,1);
            OLED_ShowNum(10*8,32,mm,1,16,1);
            OLED_ShowString(11*8,32,"cm",16,1);

            /* line 3: L:XX R:XX */
            OLED_ShowString(0,48,"D:",16,1);
            OLED_ShowNum(2*8,48,(uint16_t)(dv_d<0?-dv_d:dv_d),2,16,1);
            OLED_ShowChar(4*8,48,dv_d<0?'L':'R',16,1);
            OLED_ShowString(6*8,48,"L:",16,1);
            OLED_ShowNum(8*8,48,(uint16_t)lx_d,2,16,1);
            OLED_ShowString(11*8,48,"R:",16,1);
            OLED_ShowNum(13*8,48,(uint16_t)rx_d,2,16,1);

            OLED_Refresh(); }
    }
}
