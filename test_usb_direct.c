/*
  test_usb_direct.c - Direct USB register access test for STM32G0xx
  
  This test directly accesses USB device controller registers to verify
  our USB implementation in the Renode simulation.
*/

#include "stm32g0xx_hal.h"

// USB device controller register base address
#define USB_BASE                0x40005C00UL
#define USB                     ((USB_TypeDef *)USB_BASE)

// USB register offsets (from STM32G0xx reference manual)
#define USB_EP0R_OFFSET         0x00
#define USB_EP1R_OFFSET         0x04
#define USB_CNTR_OFFSET         0x40
#define USB_ISTR_OFFSET         0x44
#define USB_FNR_OFFSET          0x48
#define USB_DADDR_OFFSET        0x4C
#define USB_BTABLE_OFFSET       0x50

// USB control register bits
#define USB_CNTR_FRES           0x0001  // Force USB Reset
#define USB_CNTR_PDWN           0x0002  // Power Down
#define USB_CNTR_LPMODE         0x0004  // Low Power Mode

// Simple delay function
void delay_ms(uint32_t ms) {
    HAL_Delay(ms);
}

// Direct register access functions
static void usb_write_reg(uint32_t offset, uint16_t value) {
    *((volatile uint16_t *)(USB_BASE + offset)) = value;
}

static uint16_t usb_read_reg(uint32_t offset) {
    return *((volatile uint16_t *)(USB_BASE + offset));
}

// Test USB register access
void test_usb_registers(void) {
    uint16_t test_value, read_value;
    
    // Test 1: Write and read USB control register
    test_value = USB_CNTR_FRES | USB_CNTR_PDWN;
    usb_write_reg(USB_CNTR_OFFSET, test_value);
    read_value = usb_read_reg(USB_CNTR_OFFSET);
    
    // Test 2: Clear reset bit
    test_value = USB_CNTR_PDWN;
    usb_write_reg(USB_CNTR_OFFSET, test_value);
    read_value = usb_read_reg(USB_CNTR_OFFSET);
    
    // Test 3: Enable USB (clear power down)
    test_value = 0x0000;
    usb_write_reg(USB_CNTR_OFFSET, test_value);
    read_value = usb_read_reg(USB_CNTR_OFFSET);
    
    // Test 4: Set device address
    usb_write_reg(USB_DADDR_OFFSET, 0x0081);  // Address 1 + enable
    read_value = usb_read_reg(USB_DADDR_OFFSET);
    
    // Test 5: Configure endpoint 0
    usb_write_reg(USB_EP0R_OFFSET, 0x0200);  // Control endpoint
    read_value = usb_read_reg(USB_EP0R_OFFSET);
}

// RCC USB clock enable test
void test_usb_clock(void) {
    // Enable USB clock via RCC
    __HAL_RCC_USB_CLK_ENABLE();
    
    // Small delay for clock to stabilize
    delay_ms(1);
}

int main(void) {
    // Initialize HAL
    HAL_Init();
    
    // Configure system clock (basic 16MHz HSI)
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);
    
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);
    
    // Test USB clock enable
    test_usb_clock();
    
    // Test direct USB register access
    test_usb_registers();
    
    // Main loop - keep testing USB
    while (1) {
        test_usb_registers();
        delay_ms(1000);  // Test every second
    }
    
    return 0;
}