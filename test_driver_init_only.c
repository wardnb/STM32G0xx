/*
 * Minimal test to check if driver_init() completes successfully
 * This replaces main() to test just the driver initialization
 */

#include "main.h"
#include "driver.h"

int main(void)
{
    // Initialize HAL
    HAL_Init();

    // Configure system clock
    SystemClock_Config();

    // Initialize GPIO
    GPIO_Init();
    
    // Test if driver_init() completes
    if(!driver_init()) {
        // driver_init() failed - infinite loop (we can detect this in simulation)
        while(1) {
            // Stall point 1: driver_init() failed
        }
    }
    
    // driver_init() succeeded! 
    // Instead of calling grbl_enter(), just loop peacefully
    while(1) {
        // Stall point 2: driver_init() succeeded, sitting in peaceful loop
        // This should NOT trigger CPU abort
        
        // We can send a character to UART to indicate success
        // (if UART is working)
    }
}