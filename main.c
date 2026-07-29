/**
 * @file    main.c
 * @brief   5路循迹 + 按键计时 + 编码器里程自动停车
 *
 * 循迹通道: CH0=PB25(最左) CH1=PB18 CH2=PB21(中) CH3=PB22 CH4=PA30(最右)
 * 策略: 黑线中心跟随 (哪个传感器看到黑线就往哪边转)
 */

#include "ti_msp_dl_config.h"
#include "led/led.h"
#include "tb6612/tb6612.h"
#include "encoder/encoder.h"
#include "oled/oled.h"
#include "tracking/tracking.h"
#include "buzzer/buzzer.h"
#include <stdint.h>
#include <stdbool.h>

volatile uint32_t g_tick_1ms = 0;

/* -- 循迹速度参数 -- */
#define TRK_STRAIGHT   35   /* 直行快轮 */
#define TRK_LIGHT      19   /* 小转慢轮 */
#define TRK_MID          5   /* 大转慢轮 */

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
    uint32_t le=g_tick_1ms, lo=g_tick_1ms;
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
            static int16_t lx=TRK_STRAIGHT, rx=TRK_STRAIGHT;
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

            /* ── 5路循迹逻辑 ──
             * tracking_read 返回 true=黑线(低电平)
             * 优先级: M(居中) > L1/R1(内) > L2/R2(外)
             * 策略: 哪个传感器看到黑线就往哪边转, 保持黑线在中间 */
            if(g_run){
            uint8_t m=tracking_read_all();
            uint8_t L2=(m>>0)&1, L1=(m>>1)&1, M=(m>>2)&1;
            uint8_t R1=(m>>3)&1, R2=(m>>4)&1;

            if(!L2 && !L1 && !M && !R1 && !R2){
                /* 全白: 丢线, 停止 */
                tb6612_set_speed(0,0);
            } else if(L2 && L1 && M && R1 && R2){
                /* 全黑: 十字/宽线, 保持上次 */
                tb6612_set_speed(lx, rx);
            } else if(M){
                /* 居中 = 直行 */
                lx=TRK_STRAIGHT; rx=TRK_STRAIGHT;
                tb6612_set_speed(lx, rx);
            } else if(L1){
                /* 左内 = 向右小转 */
                lx=TRK_STRAIGHT; rx=TRK_LIGHT;
                tb6612_set_speed(lx, rx);
            } else if(R1){
                /* 右内 = 向左小转 */
                lx=TRK_LIGHT; rx=TRK_STRAIGHT;
                tb6612_set_speed(lx, rx);
            } else if(L2){
                /* 最左 = 向右急转 */
                lx=TRK_STRAIGHT; rx=TRK_MID;
                tb6612_set_speed(lx, rx);
            } else if(R2){
                /* 最右 = 向左急转 */
                lx=TRK_MID; rx=TRK_STRAIGHT;
                tb6612_set_speed(lx, rx);
            }
            } /* g_run tracking guard */
            }
        }

        if((int32_t)(n-lo)>=100){ lo+=100;
            uint8_t tk=tracking_read_all();
            uint32_t t=g_tmr_on?(n-g_tmr_start):g_tmr_frozen;
            uint32_t sec  = t / 1000;
            uint32_t ts   = (t % 1000) / 100;
            uint32_t cm   = g_dist_cm_x10 / 10;
            uint32_t mm   = g_dist_cm_x10 % 10;
            int      i;

            OLED_Clear();

            /* line 0: T:00000 (5路) */
            OLED_ShowChar(0,0,'T',16,1);
            OLED_ShowChar(8,0,':',16,1);
            for(i=0;i<5;i++){
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

            OLED_Refresh(); }
    }
}
