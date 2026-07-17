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
TIM_HandleTypeDef htim3;

volatile uint8_t target_angle = 142;
volatile float current_angle = 0.0f;

/**
 * Forward declaration of RXPDO_update() from ecat_slv.c.
 * We call this unconditionally to ensure PDO data is always
 * read from the ESC SM2 memory, regardless of ALevent state
 * (the LAN9252 CSR reads can consume ALevent bits before
 * DIG_process checks them).
 */
extern void RXPDO_update(void);
extern void TXPDO_update(void);

/**
 * Application hook: runs every ecat_slv() cycle.
 * Forces PDO read then sets outputs.
 */
static void app_hook(void)
{
    /* Unconditionally read SM2 process data from ESC into object dictionary.
     * This bypasses the ALevent-based SM2 detection in DIG_process,
     * which can miss events on LAN9252 due to CSR ALevent register reads
     * clearing bits before DIG_process checks them.
     */
    RXPDO_update();

    /* Now read the fresh value from object dictionary */
    target_angle = Obj.led[0].state;
    if (target_angle > 180) target_angle = 180;
    Obj.debug_buffer[0] = target_angle;

    /* Echo the received Rx value back so the master can verify the
     * round-trip over the TxPDO. */
    Obj.echo_byte = Obj.led[0].state;

    /* Unconditionally pack SM3 process data from object dictionary into the ESC.
     * This bypasses the App.state gate in DIG_process()'s input section, which
     * can leave TXPDO_update() skipped (echo stays 0) if the internal
     * APPSTATE_INPUT flag is not set. Mirrors the RXPDO_update() call above. */
    TXPDO_update();
}

/**
 * SOES configuration.
 * TODO: implement task specific function and specify
 *        here the address if needed.
 */
static esc_cfg_t config =
{
    .user_arg = "/dev/lan9252",
    .use_interrupt = 0,
    .watchdog_cnt = 500,
    .set_defaults_hook = NULL,
    .pre_state_change_hook = NULL,
    .post_state_change_hook = NULL,
    .application_hook = app_hook,
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
// In main.c, outside the loop
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
        // Slew rate control
        if (current_angle < target_angle) current_angle += 1.0f;
        else if (current_angle > target_angle) current_angle -= 1.0f;

        uint32_t pulse = 50 + (((uint32_t)current_angle) * 200 / 180);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);
    }
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
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
}
void MX_TIM3_100Hz_Init(void) {
    __HAL_RCC_TIM3_CLK_ENABLE();

    // Use the global htim3 handle
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 79;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 999;
    HAL_TIM_Base_Init(&htim3);

    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);

    HAL_TIM_Base_Start_IT(&htim3);
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
 *
 * NOTE: This function is called from DIG_process() inside the
 * output section when SM2 event is detected. The application_hook
 * (app_hook) calls RXPDO_update() unconditionally and then does
 * this same work, so this function may not be called if SM2
 * events are missed -- the app_hook handles that case.
 */
void cb_set_outputs()
{
    /* Capture target angle immediately upon network receipt */
    target_angle = Obj.led[0].state;
    if (target_angle > 180) target_angle = 180;
    Obj.debug_buffer[0] = target_angle;
}
void update_servo_smoothly(void)
{
    static uint32_t last_tick = 0;

    // Update every 10ms (100Hz)
    if (HAL_GetTick() - last_tick >= 10)
    {
        last_tick = HAL_GetTick();

        // Slew rate control (adjust step size for speed)
        if (current_angle < target_angle) current_angle += 1.0f;
        else if (current_angle > target_angle) current_angle -= 1.0f;

        // Apply smooth pulse
        uint32_t pulse = 50 + (((uint32_t)current_angle) * 200 / 180);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);
    }
}
/****************************************************************
 * Application
 ****************************************************************/
int main(void)
{
	/* Reset of all peripherals, initializes the Flash interface and the SysTick.*/
	HAL_Init();
	/* configure the system clock */
	SystemClock_Config();
	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_TIM2_PWM_Init(); // Initialize PWM for servo
	MX_TIM3_100Hz_Init(); // Servo heartbeat
	/* initialize EtherCAT slave */
	ecat_slv_init(&config);
	while (1)
	{
		// run slave logic (polling mode)
		ecat_slv();
	}
}
