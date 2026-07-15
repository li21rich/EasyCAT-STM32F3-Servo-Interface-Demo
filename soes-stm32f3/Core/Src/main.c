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

/**
 * SOES configuration.
 * TODO: implement task specific function and specify
 * 		 here the address if needed.
 */
static esc_cfg_t config =
{
    .user_arg = "/dev/lan9252",
    .use_interrupt = 0,
    .watchdog_cnt = 500,
    .set_defaults_hook = NULL,
    .pre_state_change_hook = NULL,
    .post_state_change_hook = NULL,
    .application_hook = cb_set_outputs,
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
/** system clock configuration */
static void SystemClock_Config(void);

/** GPIO initialization function */
static void MX_GPIO_Init(void);


/****************************************************************
 * Private function
 ****************************************************************/
static void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/**
	 * Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/* Initializes the CPU, AHB and APB buses clocks */
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

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* Configure PA0 as Alternate Function for TIM2_CH1 (servo PWM) */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2; // TIM2_CH1 on PA0
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
 * @brief TIM2 HAL MSP Initialization
 *        This function is called from HAL_TIM_PWM_Init() and handles
 *        the low-level peripheral clock enable.
 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        /* Enable TIM2 clock */
        __HAL_RCC_TIM2_CLK_ENABLE();
    }
}

void Error_Handler(void)
{
	/* disable implementation to report the HAL error return state */
	__disable_irq();
	while (1){}
}


/****************************************************************
 * Public functions
 ****************************************************************/
/**
 * Initialize TIM2 for 50Hz servo PWM on channel 1 (PA0).
 * With HSI = 8MHz, prescaler 79 gives 100kHz timer clock.
 * Period 19999 gives 5Hz (200ms) - wait, that's wrong for 50Hz.
 * Actually: 8MHz / (79+1) = 100kHz. Period 1999 gives 50Hz (20ms).
 */
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
    sConfigOC.Pulse = 150;                  // 1.5ms pulse = center (150 * 10us)
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * Read physical input values and assigns the corresponding members
 * in the CANopen object dictionary so that the slave can send that
 * info back to the master with TXPDO or SDO.
 */
void cb_get_inputs()
{
	Obj.debug_buffer[0] = Obj.rtd_filter + 10;
}

/**
 * Write physical output values from the corresponding members of
 * the CANopen object dictionary (i.e. set DO, PWM, ...).
 * Here we read led[0].state (which receives the first output byte
 * from the master via the PDO mapping) and convert it to a servo
 * PWM pulse width.
 */
void cb_set_outputs()
{
	/* Read the servo angle sent by the master (0-180 degrees) */
	uint8_t servo_angle = Obj.led[0].state;

	/* Clamp to safe range */
	if (servo_angle > 180) servo_angle = 180;
	/* Convert angle (0-180) to pulse width (500-2500 us)
	 * Timer runs at 100kHz → 1 tick = 10 us
	 * 500us  = 50 ticks  (0 deg)
	 * 1500us = 150 ticks (90 deg)
	 * 2500us = 250 ticks (180 deg)
	 */
	uint32_t pulse = 50 + (servo_angle * 200 / 180);

	/* Update the PWM compare register */
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);
}


/****************************************************************
 * Application
 ****************************************************************/
int main(void)
{
	/**
	 * Reset of all peripherals, initializes the Flash interface
	 * and the SysTick.
	 */
	HAL_Init();

	/* configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();

	MX_TIM2_PWM_Init(); // Initialize PWM for servo

	/* Test: set servo to ~170° position before EtherCAT starts */
	/* 170° → pulse = 50 + (170 * 200 / 180) = 50 + 188 = 238 ticks (2.38ms) */
	//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 238);

	/* initialize EtherCAT slave */
	ecat_slv_init(&config);
	while (1)
	{
		/* run slave logic (polling mode) */
		ecat_slv();
	}
}
