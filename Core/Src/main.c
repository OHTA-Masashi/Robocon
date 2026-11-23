#include "stm32f4xx_hal.h"

/* =========================================================
 * ADC Handle
 * ========================================================= */
ADC_HandleTypeDef hadc1;


/* =========================================================
 * Function Prototypes
 * ========================================================= */
void SystemClock_Config(void);

static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);

static void Speaker_Beep(void);

uint32_t Read_ADC(uint32_t channel);

void Error_Handler(void);


/* =========================================================
 * ADC判定値
 *
 * 回路：
 * 通常時    → 3.3Vへプルアップ
 * センサON → LOW
 *
 * 12bit ADC
 * 3.3V = 4095
 *
 * 約1V以下をONと判定
 * ========================================================= */
#define ADC_ON_THRESHOLD    1241


/* =========================================================
 * ADC値
 * デバッグ時にも確認できるようvolatile
 * ========================================================= */
volatile uint32_t adc_pc0 = 0;
volatile uint32_t adc_pc1 = 0;
volatile uint32_t adc_pc2 = 0;
volatile uint32_t adc_pc3 = 0;

volatile uint32_t adc_pa4 = 0;
volatile uint32_t adc_pa6 = 0;
volatile uint32_t adc_pa7 = 0;


/* =========================================================
 * Limit Switch
 *
 * 1 = 押されていない
 * 0 = 押されている
 *
 * PC4 / PC5は内部PULL-UP
 * スイッチONでGNDへ落ちる前提
 * ========================================================= */
volatile GPIO_PinState limit_pc4 = GPIO_PIN_SET;
volatile GPIO_PinState limit_pc5 = GPIO_PIN_SET;


/* =========================================================
 * MAIN
 * ========================================================= */
int main(void)
{
    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();

    MX_ADC1_Init();


    /* =====================================================
     * 起動音
     * ===================================================== */
    Speaker_Beep();


    /* =====================================================
     * 起動後5秒待機
     *
     * この間、LEDは全てOFF
     * ===================================================== */
    HAL_Delay(5000);


    /* =====================================================
     * メインループ
     * ===================================================== */
    while (1)
    {
        /* =================================================
         * 7つのADC読み取り
         * ================================================= */

        /* PC3 = ADC1_IN13 */
        adc_pc3 = Read_ADC(ADC_CHANNEL_13);

        /* PC2 = ADC1_IN12 */
        adc_pc2 = Read_ADC(ADC_CHANNEL_12);

        /* PC1 = ADC1_IN11 */
        adc_pc1 = Read_ADC(ADC_CHANNEL_11);

        /* PC0 = ADC1_IN10 */
        adc_pc0 = Read_ADC(ADC_CHANNEL_10);

        /* PA4 = ADC1_IN4 */
        adc_pa4 = Read_ADC(ADC_CHANNEL_4);

        /* PA6 = ADC1_IN6 */
        adc_pa6 = Read_ADC(ADC_CHANNEL_6);

        /* PA7 = ADC1_IN7 */
        adc_pa7 = Read_ADC(ADC_CHANNEL_7);



        /* =================================================
         * ライントレース確認LED
         *
         * ADCがLOW → センサON → LED点灯
         * ================================================= */


        /* -------------------------------------------------
         * PC3 → PD2
         * ------------------------------------------------- */
        if (adc_pc3 <= ADC_ON_THRESHOLD)
        {
            HAL_GPIO_WritePin(
                GPIOD,
                GPIO_PIN_2,
                GPIO_PIN_SET
            );
        }
        else
        {
            HAL_GPIO_WritePin(
                GPIOD,
                GPIO_PIN_2,
                GPIO_PIN_RESET
            );
        }


        /* -------------------------------------------------
         * PC2 → PB2
         * ------------------------------------------------- */
        if (adc_pc2 <= ADC_ON_THRESHOLD)
        {
            HAL_GPIO_WritePin(
                GPIOB,
                GPIO_PIN_2,
                GPIO_PIN_SET
            );
        }
        else
        {
            HAL_GPIO_WritePin(
                GPIOB,
                GPIO_PIN_2,
                GPIO_PIN_RESET
            );
        }


        /* -------------------------------------------------
         * PC1 → PB1
         * ------------------------------------------------- */
        if (adc_pc1 <= ADC_ON_THRESHOLD)
        {
            HAL_GPIO_WritePin(
                GPIOB,
                GPIO_PIN_1,
                GPIO_PIN_SET
            );
        }
        else
        {
            HAL_GPIO_WritePin(
                GPIOB,
                GPIO_PIN_1,
                GPIO_PIN_RESET
            );
        }


        /* -------------------------------------------------
         * PC0 → PA15
         * ------------------------------------------------- */
        if (adc_pc0 <= ADC_ON_THRESHOLD)
        {
            HAL_GPIO_WritePin(
                GPIOA,
                GPIO_PIN_15,
                GPIO_PIN_SET
            );
        }
        else
        {
            HAL_GPIO_WritePin(
                GPIOA,
                GPIO_PIN_15,
                GPIO_PIN_RESET
            );
        }


        /* -------------------------------------------------
         * PA4 → PA10
         * ------------------------------------------------- */
        if (adc_pa4 <= ADC_ON_THRESHOLD)
        {
            HAL_GPIO_WritePin(
                GPIOA,
                GPIO_PIN_10,
                GPIO_PIN_SET
            );
        }
        else
        {
            HAL_GPIO_WritePin(
                GPIOA,
                GPIO_PIN_10,
                GPIO_PIN_RESET
            );
        }


        /*
         * PA6 / PA7
         *
         * 今回はADC値を読むだけ。
         * LED制御には使用しない。
         *
         * adc_pa6
         * adc_pa7
         */


        /* =================================================
         * Limit Switch
         * ================================================= */

        limit_pc4 = HAL_GPIO_ReadPin(
            GPIOC,
            GPIO_PIN_4
        );

        limit_pc5 = HAL_GPIO_ReadPin(
            GPIOC,
            GPIO_PIN_5
        );


        /*
         * 現状では読み取るだけ。
         *
         * limit_pc4 == GPIO_PIN_RESET
         * → PC4のリミットSW ON
         *
         * limit_pc5 == GPIO_PIN_RESET
         * → PC5のリミットSW ON
         */


        HAL_Delay(10);
    }
}


/* =========================================================
 * ADC読み取り
 *
 * 指定されたADC Channelを4回読んで平均
 * ========================================================= */
uint32_t Read_ADC(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    uint32_t sum = 0;
    uint32_t i;


    /* =====================================================
     * Channel設定
     * ===================================================== */
    sConfig.Channel = channel;

    sConfig.Rank = 1;

    sConfig.SamplingTime =
        ADC_SAMPLETIME_480CYCLES;


    if (HAL_ADC_ConfigChannel(
            &hadc1,
            &sConfig) != HAL_OK)
    {
        Error_Handler();
    }


    /* =====================================================
     * チャンネル切替直後の最初の測定は捨てる
     * ===================================================== */

    if (HAL_ADC_Start(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }


    if (HAL_ADC_PollForConversion(
            &hadc1,
            100) == HAL_OK)
    {
        (void)HAL_ADC_GetValue(&hadc1);
    }
    else
    {
        Error_Handler();
    }


    HAL_ADC_Stop(&hadc1);


    /* =====================================================
     * 4回平均
     * ===================================================== */

    for (i = 0; i < 4; i++)
    {
        if (HAL_ADC_Start(&hadc1) != HAL_OK)
        {
            Error_Handler();
        }


        if (HAL_ADC_PollForConversion(
                &hadc1,
                100) == HAL_OK)
        {
            sum += HAL_ADC_GetValue(&hadc1);
        }
        else
        {
            Error_Handler();
        }


        HAL_ADC_Stop(&hadc1);
    }


    return sum / 4;
}


/* =========================================================
 * ADC1 Init
 * ========================================================= */
static void MX_ADC1_Init(void)
{
    __HAL_RCC_ADC1_CLK_ENABLE();


    hadc1.Instance = ADC1;


    hadc1.Init.ClockPrescaler =
        ADC_CLOCK_SYNC_PCLK_DIV4;


    hadc1.Init.Resolution =
        ADC_RESOLUTION_12B;


    hadc1.Init.ScanConvMode =
        DISABLE;


    hadc1.Init.ContinuousConvMode =
        DISABLE;


    hadc1.Init.DiscontinuousConvMode =
        DISABLE;


    hadc1.Init.ExternalTrigConvEdge =
        ADC_EXTERNALTRIGCONVEDGE_NONE;


    hadc1.Init.ExternalTrigConv =
        ADC_SOFTWARE_START;


    hadc1.Init.DataAlign =
        ADC_DATAALIGN_RIGHT;


    hadc1.Init.NbrOfConversion =
        1;


    hadc1.Init.DMAContinuousRequests =
        DISABLE;


    hadc1.Init.EOCSelection =
        ADC_EOC_SINGLE_CONV;


    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }
}


/* =========================================================
 * GPIO Init
 * ========================================================= */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};


    /* =====================================================
     * GPIO Clock Enable
     * ===================================================== */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();


    /* =====================================================
     * 全LED 初期OFF
     * ===================================================== */

    HAL_GPIO_WritePin(
        GPIOA,
        GPIO_PIN_10 |
        GPIO_PIN_15,
        GPIO_PIN_RESET
    );


    HAL_GPIO_WritePin(
        GPIOB,
        GPIO_PIN_1 |
        GPIO_PIN_2,
        GPIO_PIN_RESET
    );


    HAL_GPIO_WritePin(
        GPIOD,
        GPIO_PIN_2,
        GPIO_PIN_RESET
    );


    /* =====================================================
     * Speaker OFF
     * ===================================================== */

    HAL_GPIO_WritePin(
        GPIOC,
        GPIO_PIN_9,
        GPIO_PIN_RESET
    );


    /* =====================================================
     * PA10 / PA15
     * LED出力
     * ===================================================== */

    GPIO_InitStruct.Pin =
        GPIO_PIN_10 |
        GPIO_PIN_15;

    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(
        GPIOA,
        &GPIO_InitStruct
    );


    /* =====================================================
     * PB1 / PB2
     * LED出力
     * ===================================================== */

    GPIO_InitStruct.Pin =
        GPIO_PIN_1 |
        GPIO_PIN_2;

    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(
        GPIOB,
        &GPIO_InitStruct
    );


    /* =====================================================
     * PD2
     * LED出力
     * ===================================================== */

    GPIO_InitStruct.Pin =
        GPIO_PIN_2;

    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(
        GPIOD,
        &GPIO_InitStruct
    );


    /* =====================================================
     * PC9
     * Speaker GPIO
     * ===================================================== */

    GPIO_InitStruct.Pin =
        GPIO_PIN_9;

    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(
        GPIOC,
        &GPIO_InitStruct
    );


    /* =====================================================
     * PC0～PC3
     * ADC
     * ===================================================== */

    GPIO_InitStruct.Pin =
        GPIO_PIN_0 |
        GPIO_PIN_1 |
        GPIO_PIN_2 |
        GPIO_PIN_3;

    GPIO_InitStruct.Mode =
        GPIO_MODE_ANALOG;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    HAL_GPIO_Init(
        GPIOC,
        &GPIO_InitStruct
    );


    /* =====================================================
     * PA4 / PA6 / PA7
     * ADC
     * ===================================================== */

    GPIO_InitStruct.Pin =
        GPIO_PIN_4 |
        GPIO_PIN_6 |
        GPIO_PIN_7;

    GPIO_InitStruct.Mode =
        GPIO_MODE_ANALOG;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    HAL_GPIO_Init(
        GPIOA,
        &GPIO_InitStruct
    );


    /* =====================================================
     * PC4 / PC5
     * Limit Switch
     *
     * 通常HIGH
     * スイッチONでGND = LOW
     * ===================================================== */

    GPIO_InitStruct.Pin =
        GPIO_PIN_4 |
        GPIO_PIN_5;

    GPIO_InitStruct.Mode =
        GPIO_MODE_INPUT;

    GPIO_InitStruct.Pull =
        GPIO_PULLUP;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(
        GPIOC,
        &GPIO_InitStruct
    );
}


/* =========================================================
 * 起動音
 *
 * PC9
 * 約2kHz
 * 約200ms
 * ========================================================= */
static void Speaker_Beep(void)
{
    uint32_t i;


    for (i = 0; i < 400; i++)
    {
        /* ON */
        HAL_GPIO_WritePin(
            GPIOC,
            GPIO_PIN_9,
            GPIO_PIN_SET
        );


        for (volatile uint32_t d = 0;
             d < 1000;
             d++)
        {
            __NOP();
        }


        /* OFF */
        HAL_GPIO_WritePin(
            GPIOC,
            GPIO_PIN_9,
            GPIO_PIN_RESET
        );


        for (volatile uint32_t d = 0;
             d < 1000;
             d++)
        {
            __NOP();
        }
    }


    /* 最後はOFF */
    HAL_GPIO_WritePin(
        GPIOC,
        GPIO_PIN_9,
        GPIO_PIN_RESET
    );
}


/* =========================================================
 * System Clock
 * HSI = 16MHz
 * ========================================================= */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};

    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};


    RCC_OscInitStruct.OscillatorType =
        RCC_OSCILLATORTYPE_HSI;


    RCC_OscInitStruct.HSIState =
        RCC_HSI_ON;


    RCC_OscInitStruct.HSICalibrationValue =
        RCC_HSICALIBRATION_DEFAULT;


    RCC_OscInitStruct.PLL.PLLState =
        RCC_PLL_NONE;


    if (HAL_RCC_OscConfig(
            &RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }


    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_PCLK1 |
        RCC_CLOCKTYPE_PCLK2;


    RCC_ClkInitStruct.SYSCLKSource =
        RCC_SYSCLKSOURCE_HSI;


    RCC_ClkInitStruct.AHBCLKDivider =
        RCC_SYSCLK_DIV1;


    RCC_ClkInitStruct.APB1CLKDivider =
        RCC_HCLK_DIV1;


    RCC_ClkInitStruct.APB2CLKDivider =
        RCC_HCLK_DIV1;


    if (HAL_RCC_ClockConfig(
            &RCC_ClkInitStruct,
            FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
}


/* =========================================================
 * Error Handler
 * ========================================================= */
void Error_Handler(void)
{
    __disable_irq();


    /* Speaker OFF */
    HAL_GPIO_WritePin(
        GPIOC,
        GPIO_PIN_9,
        GPIO_PIN_RESET
    );


    /* LED OFF */
    HAL_GPIO_WritePin(
        GPIOA,
        GPIO_PIN_10 |
        GPIO_PIN_15,
        GPIO_PIN_RESET
    );


    HAL_GPIO_WritePin(
        GPIOB,
        GPIO_PIN_1 |
        GPIO_PIN_2,
        GPIO_PIN_RESET
    );


    HAL_GPIO_WritePin(
        GPIOD,
        GPIO_PIN_2,
        GPIO_PIN_RESET
    );


    while (1)
    {
    }
}
