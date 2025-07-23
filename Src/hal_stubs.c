/*
 * hal_stubs.c - Minimal HAL implementation stubs for grblHAL
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// GPIO definitions and types
typedef enum {
    GPIO_PIN_RESET = 0U,
    GPIO_PIN_SET = 1U
} GPIO_PinState;

typedef enum {
    GPIO_MODE_INPUT = 0x00000000U,
    GPIO_MODE_OUTPUT_PP = 0x00000001U,
    GPIO_MODE_OUTPUT_OD = 0x00000011U,
    GPIO_MODE_AF_PP = 0x00000002U,
    GPIO_MODE_AF_OD = 0x00000012U,
    GPIO_MODE_IT_RISING = 0x10110000U,
    GPIO_MODE_IT_FALLING = 0x10210000U,
    GPIO_MODE_IT_RISING_FALLING = 0x10310000U,
    GPIO_MODE_EVT_RISING = 0x10120000U,
    GPIO_MODE_EVT_FALLING = 0x10220000U,
    GPIO_MODE_EVT_RISING_FALLING = 0x10320000U
} GPIO_Mode;

typedef enum {
    GPIO_NOPULL = 0x00000000U,
    GPIO_PULLUP = 0x00000001U,
    GPIO_PULLDOWN = 0x00000002U
} GPIO_Pull;

typedef enum {
    GPIO_SPEED_FREQ_LOW = 0x00000000U,
    GPIO_SPEED_FREQ_MEDIUM = 0x00000001U,
    GPIO_SPEED_FREQ_HIGH = 0x00000002U,
    GPIO_SPEED_FREQ_VERY_HIGH = 0x00000003U
} GPIO_Speed;

typedef struct {
    uint32_t Pin;
    uint32_t Mode;
    uint32_t Pull;
    uint32_t Speed;
    uint32_t Alternate;
} GPIO_InitTypeDef;

// GPIO port base addresses (STM32G0xx)
#define GPIOA_BASE 0x50000000UL
#define GPIOB_BASE 0x50000400UL
#define GPIOC_BASE 0x50000800UL
#define GPIOD_BASE 0x50000C00UL
#define GPIOF_BASE 0x50001400UL

#define GPIOA ((void*)GPIOA_BASE)
#define GPIOB ((void*)GPIOB_BASE)
#define GPIOC ((void*)GPIOC_BASE)
#define GPIOD ((void*)GPIOD_BASE)
#define GPIOF ((void*)GPIOF_BASE)

// GPIO pin definitions
#define GPIO_PIN_0  0x0001U
#define GPIO_PIN_1  0x0002U
#define GPIO_PIN_2  0x0004U
#define GPIO_PIN_3  0x0008U
#define GPIO_PIN_4  0x0010U
#define GPIO_PIN_5  0x0020U
#define GPIO_PIN_6  0x0040U
#define GPIO_PIN_7  0x0080U
#define GPIO_PIN_8  0x0100U
#define GPIO_PIN_9  0x0200U
#define GPIO_PIN_10 0x0400U
#define GPIO_PIN_11 0x0800U
#define GPIO_PIN_12 0x1000U
#define GPIO_PIN_13 0x2000U
#define GPIO_PIN_14 0x4000U
#define GPIO_PIN_15 0x8000U

// IRQ definitions
typedef enum {
    EXTI0_1_IRQn = 5,
    EXTI2_3_IRQn = 6,
    EXTI4_15_IRQn = 7,
    TIM1_CC_IRQn = 14,
    TIM2_IRQn = 15,
    TIM3_IRQn = 16,
    TIM14_IRQn = 19,
    TIM16_IRQn = 21,
    TIM17_IRQn = 22,
    USART1_IRQn = 27,
    USART2_IRQn = 28
} IRQn_Type;

// Basic UART handle structure stub
typedef struct {
    void *Instance;
    uint32_t Init;
    uint8_t *pTxBuffPtr;
    uint16_t TxXferSize;
    uint16_t TxXferCount;
    uint8_t *pRxBuffPtr;
    uint16_t RxXferSize;
    uint16_t RxXferCount;
    uint32_t State;
    uint32_t ErrorCode;
} UART_HandleTypeDef;

// Global UART handle
UART_HandleTypeDef huart2;

// Minimal UART function stubs
bool uart_init(void) {
    // Initialize basic UART settings
    huart2.Instance = (void*)0x40004400; // USART2 base address for STM32G0
    huart2.State = 0x20; // HAL_UART_STATE_READY
    huart2.ErrorCode = 0;
    return true;
}

void uart_write_char(char c) {
    // Stub for writing single character
    (void)c;
}

int uart_read_char(void) {
    // Stub for reading single character
    return -1; // No data available
}

bool uart_tx_ready(void) {
    // Stub for checking if transmit is ready
    return true;
}

bool uart_rx_ready(void) {
    // Stub for checking if receive data available
    return false;
}

void uart_flush_tx(void) {
    // Stub for flushing transmit buffer
}

void uart_flush_rx(void) {
    // Stub for flushing receive buffer
}

// HAL-style function stubs that grblHAL might expect
int HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    (void)huart;
    (void)pData;
    (void)Size;
    (void)Timeout;
    return 0; // HAL_OK
}

int HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    (void)huart;
    (void)pData;
    (void)Size;
    (void)Timeout;
    return 3; // HAL_TIMEOUT (no data)
}

int HAL_UART_Init(UART_HandleTypeDef *huart) {
    (void)huart;
    return 0; // HAL_OK
}

// Additional HAL functions needed by serial.c
uint32_t HAL_GetTick(void) {
    // Simple tick counter stub - would normally be milliseconds since boot
    static uint32_t tick_counter = 0;
    return tick_counter++;
}

void HAL_UART_IRQHandler(UART_HandleTypeDef *huart) {
    // UART interrupt handler stub
    (void)huart;
}

// GPIO HAL function stubs
GPIO_PinState HAL_GPIO_ReadPin(void *GPIOx, uint16_t GPIO_Pin) {
    (void)GPIOx;
    (void)GPIO_Pin;
    return GPIO_PIN_RESET; // Default to low state
}

void HAL_GPIO_WritePin(void *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState) {
    (void)GPIOx;
    (void)GPIO_Pin;
    (void)PinState;
    // GPIO write stub
}

int HAL_GPIO_Init(void *GPIOx, GPIO_InitTypeDef *GPIO_Init) {
    (void)GPIOx;
    (void)GPIO_Init;
    return 0; // HAL_OK
}

void HAL_GPIO_EXTI_IRQHandler(uint16_t GPIO_Pin) {
    (void)GPIO_Pin;
    // GPIO EXTI interrupt handler stub
}

// NVIC and interrupt control stubs
void HAL_NVIC_SetPriority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority) {
    (void)IRQn;
    (void)PreemptPriority;
    (void)SubPriority;
}

void NVIC_EnableIRQ(IRQn_Type IRQn) {
    (void)IRQn;
}

void NVIC_DisableIRQ(IRQn_Type IRQn) {
    (void)IRQn;
}

// System tick functions
void HAL_IncTick(void) {
    // Increment system tick stub
}

// UART extended functions
int HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size) {
    (void)huart;
    (void)pData;
    (void)Size;
    return 0; // HAL_OK
}

int HAL_UARTEx_SetTxFifoThreshold(UART_HandleTypeDef *huart, uint32_t Threshold) {
    (void)huart;
    (void)Threshold;
    return 0; // HAL_OK
}

int HAL_UARTEx_SetRxFifoThreshold(UART_HandleTypeDef *huart, uint32_t Threshold) {
    (void)huart;
    (void)Threshold;
    return 0; // HAL_OK
}

int HAL_UARTEx_DisableFifoMode(UART_HandleTypeDef *huart) {
    (void)huart;
    return 0; // HAL_OK
}

// Additional missing HAL functions
// HAL Status definitions  
#define HAL_OK      0x00U
#define HAL_ERROR   0x01U
#define HAL_BUSY    0x02U
#define HAL_TIMEOUT 0x03U

// Enhanced HAL functions for proper boot sequence
int HAL_Init(void) {
    // Initialize HAL but DO NOT enable SysTick in simulation
    // Real hardware would configure SysTick here, but in simulation
    // we want to avoid timer interrupts that cause CPU abort
    return HAL_OK;
}

// SystemCoreClockUpdate is already defined in system_stm32g0xx.c - removed duplicate

int HAL_PWREx_ControlVoltageScaling(uint32_t VoltageScaling) {
    (void)VoltageScaling;
    // Voltage scaling configuration successful
    return HAL_OK;
}

int HAL_RCC_OscConfig(void *RCC_OscInitStruct) {
    (void)RCC_OscInitStruct;
    // Clock oscillator configuration successful
    return HAL_OK;
}

int HAL_RCC_ClockConfig(void *RCC_ClkInitStruct, uint32_t FLatency) {
    (void)RCC_ClkInitStruct;
    (void)FLatency;
    // Clock configuration successful
    return HAL_OK;
}

// Error_Handler is already defined in main.c - removed duplicate

// Add GPIO clock enable functions (commonly called during initialization)
void __HAL_RCC_GPIOA_CLK_ENABLE(void) {
    // Enable GPIOA clock
}

void __HAL_RCC_GPIOB_CLK_ENABLE(void) {
    // Enable GPIOB clock  
}

void __HAL_RCC_GPIOC_CLK_ENABLE(void) {
    // Enable GPIOC clock
}

void __HAL_RCC_GPIOD_CLK_ENABLE(void) {
    // Enable GPIOD clock
}

void __HAL_RCC_USART1_CLK_ENABLE(void) {
    // Enable USART1 clock
}

void __HAL_RCC_USB_CLK_ENABLE(void) {
    // Enable USB clock
}

// UART functions already defined earlier in this file - removed duplicates

// HAL Tick functions already defined above - removed duplicates

void HAL_Delay(uint32_t Delay) {
    // Simple delay implementation
    (void)Delay;
    // In simulation, just return immediately
}

// Critical System Interrupt Handlers
// These prevent CPU abort when interrupts fire during boot

// SysTick_Handler is likely already defined in driver.c - removed duplicate

void HardFault_Handler(void) {
    // Hard fault handler - important for debugging
    while(1) {
        // Hang here for debugging
    }
}

void NMI_Handler(void) {
    // Non-maskable interrupt handler
    while(1) {
        // Hang here for debugging  
    }
}

// System call handlers
void SVC_Handler(void) {
    // Supervisor call handler
}

void PendSV_Handler(void) {
    // PendSV handler
}

// Default_Handler is already defined in startup file - removed duplicate

// Only add interrupt handlers that are NOT already defined in the firmware
// The firmware already has USART1, TIM2, EXTI handlers, etc.

void USB_IRQHandler(void) {
    // USB interrupt handler (if not defined elsewhere)
}

void RCC_IRQHandler(void) {
    // RCC interrupt handler
}

void DMA1_Channel1_IRQHandler(void) {
    // DMA Channel 1 interrupt handler
}

// Board initialization stub (required by driver.c)
bool board_init(void) {
    return true;
}

#if USB_SERIAL_CDC
// USB Device function stubs (only when USB is enabled)
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len) {
    (void)Buf;
    (void)Len;
    return 0; // USBD_OK
}

void MX_USB_DEVICE_Init(void) {
    // USB device initialization stub
}
#endif