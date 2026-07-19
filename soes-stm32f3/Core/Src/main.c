/****************************************************************
 * Includes
 ****************************************************************/
#include "main.h"
#include "esc.h"
#include "ecat_slv.h"
#include "utypes.h"

/****************************************************************
 * Data
 ****************************************************************/
/* CANopen Object Dictionary */
_Objects Obj;

TIM_HandleTypeDef htim2; // Timer for servo PWM

volatile uint8_t target_angle = 145;

/**
 * Forward declarations of PDO update functions.
 * (COE_pdoUnpack and COE_pdoPack are already defined in esc_coe.h)
 */
extern void RXPDO_update(void);
extern void TXPDO_update(void);

/* Expose the SOES internal mapping arrays needed for pack/unpack */
extern _SMmap SMmap2[];
extern _SMmap SMmap3[];

/**
 * SOES configuration.
 * We set application_hook to NULL and let the native SOES callback system
 * (cb_set_outputs / cb_get_inputs) handle the state changes cleanly.
 */
static esc_cfg_t config =
{
    .user_arg = "/dev/lan9252",
    .use_interrupt = 0,
    .watchdog_cnt = 500,
    .set_defaults_hook = NULL,
    .pre_state_change_hook = NULL,
    .post_state_change_hook = NULL,
    .application_hook = NULL, // Set to NULL to let native callbacks run
    .safeoutput_override = NULL,
    .pre_object_download_hook = NULL,
    .post_object_download_hook = NULL,
    .rxpdo_override = NULL,
    .txpdo_override = NULL,
    .esc_hw_interrupt_enable = NULL,
    .esc_hw_interrupt_disable = NULL,
    .esc_hw_eep_handler = NULL,
    .esc_check_dc_handler = NULL,
};

/****************************************************************
 * Private function prototypes
 ****************************************************************/
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/****************************************************************
 * Private functions
 ****************************************************************/
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* Configure PA0 as Alternate Function for TIM2_CH1 (servo PWM) */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        __HAL_RCC_TIM2_CLK_ENABLE();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1){}
}

/****************************************************************
 * Public functions (SOES native callbacks)
 ****************************************************************/

/**
 * cb_set_outputs() is automatically called by SOES immediately after
 * a valid RXPDO packet has been received, verified, and unpacked.
 */
void cb_set_outputs(void)
{
    // 1. Get incoming data
    uint8_t master_val = Obj.led[0].state;

    // 2. Set PWM
    uint32_t pulse = 50 + (((uint32_t)master_val) * 200 / 180);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);

    // 3. Update the echo value
    Obj.echo_byte = master_val + 1;

    // 4. Force the SOES stack to update the SPI buffer immediately
    TXPDO_update();
}

/**
 * cb_get_inputs() is automatically called by SOES right before
 * packing and sending the TXPDO packet back to the Master.
 */
void cb_get_inputs(void)
{
    // The TX value is already updated in cb_set_outputs, nothing extra needed here.
}

void MX_TIM2_PWM_Init(void) {
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 79;              // 8MHz / (79+1) = 100kHz
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 1999;               // 100kHz / (1999+1) = 50Hz
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 50 + (145 * 200 / 180); // Default startup pulse (145 degrees)
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
}

/****************************************************************
 * Application
 ****************************************************************/
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM2_PWM_Init(); // Initialize PWM for servo

    /* Set SM addresses and lengths in both SOES struct and Hardware.
     * SM2 at 0x1100 (Output), SM3 at 0x1180 (Input) matching options.h defaults.
     * Lengths are set to 32 to match the object dictionary PDO size (SOES
     * recalculates these during state transitions anyway). */
    ESCvar.SM[2].PSA = 0x1100;
    ESCvar.SM[2].Length = 32;
    ESCvar.ESC_SM2_sml = 32;

    ESCvar.SM[3].PSA = 0x1180;
    ESCvar.SM[3].Length = 32;
    ESCvar.ESC_SM3_sml = 32;

    /* Hardware SM2 (Output) */
    ESC_write(0x804, "\x00\x11", 2); // PSA: 0x1100
    ESC_write(0x806, "\x64\x00", 2); // Control: 0x64 (Buffered, Output, Enable)
    ESC_write(0x808, "\x20\x00", 2); // Length: 32

    /* Hardware SM3 (Input) */
    ESC_write(0x80C, "\x80\x11", 2); // PSA: 0x1180 (matching options.h SM3_sma)
    ESC_write(0x80E, "\x20\x00", 2); // Control: 0x20 (Buffered, Input, Enable)
    ESC_write(0x810, "\x20\x00", 2); // Length: 32

    ecat_slv_init(&config);

    while (1)
    {
        ecat_slv();
    }
}
