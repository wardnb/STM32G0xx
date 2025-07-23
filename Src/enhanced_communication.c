/*
 * enhanced_communication.c - Enhanced communication layer providing bulletproof 
 * UART and USB communication with real hardware simulation
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

// Enhanced communication state tracking
typedef struct {
    // UART State
    bool uart_connected;
    bool uart_tx_busy;
    uint32_t uart_tx_count;
    uint32_t uart_rx_count;
    uint32_t uart_error_count;
    uint32_t uart_last_activity;
    
    // USB State  
    bool usb_connected;
    bool usb_enumerated;
    bool usb_tx_busy;
    uint32_t usb_tx_count;
    uint32_t usb_rx_count;
    uint32_t usb_timeout_count;
    uint32_t usb_last_activity;
    
    // System timing
    uint32_t system_tick;
    uint32_t last_heartbeat;
    
    // Communication buffers
    char uart_tx_buffer[256];
    char uart_rx_buffer[256];
    volatile uint16_t uart_tx_head, uart_tx_tail;
    volatile uint16_t uart_rx_head, uart_rx_tail;
    
    char usb_tx_buffer[128];
    char usb_rx_buffer[256];
    volatile uint16_t usb_tx_head, usb_tx_tail;
    volatile uint16_t usb_rx_head, usb_rx_tail;
} enhanced_comm_state_t;

static enhanced_comm_state_t comm_state = {0};

// Initialize enhanced communication system
bool enhanced_comm_init(void) {
    // Reset all state
    memset(&comm_state, 0, sizeof(comm_state));
    
    // Initialize UART as connected (for grbl compatibility)
    comm_state.uart_connected = true;
    comm_state.system_tick = 1000; // Start at reasonable tick count
    
    // Initialize USB as disconnected (requires explicit connection)
    comm_state.usb_connected = false;
    comm_state.usb_enumerated = false;
    
    return true;
}

// Enhanced UART transmit with bulletproof reliability
bool enhanced_uart_transmit(const uint8_t *data, uint16_t size) {
    if (!comm_state.uart_connected || !data || size == 0) {
        comm_state.uart_error_count++;
        return false;
    }
    
    // Simulate transmission delay and success rate
    comm_state.system_tick += 5; // Simulate 5ms transmission time
    comm_state.uart_tx_count += size;
    comm_state.uart_last_activity = comm_state.system_tick;
    
    // Simulate occasional transmission issues for robustness testing
    if ((comm_state.uart_tx_count % 1000) == 999) {
        // Simulate 0.1% transmission failure rate
        comm_state.uart_error_count++;
        return false;
    }
    
    // Buffer the data (simplified circular buffer)
    for (uint16_t i = 0; i < size && i < sizeof(comm_state.uart_tx_buffer) - 1; i++) {
        uint16_t head = (comm_state.uart_tx_head + 1) % sizeof(comm_state.uart_tx_buffer);
        if (head != comm_state.uart_tx_tail) {
            comm_state.uart_tx_buffer[comm_state.uart_tx_head] = data[i];
            comm_state.uart_tx_head = head;
        }
    }
    
    return true;
}

// Enhanced USB transmit with connection state management  
bool enhanced_usb_transmit(const uint8_t *data, uint16_t size) {
    if (!comm_state.usb_connected || !comm_state.usb_enumerated || !data || size == 0) {
        comm_state.usb_timeout_count++;
        return false;
    }
    
    if (comm_state.usb_tx_busy) {
        comm_state.usb_timeout_count++;
        return false; // Busy, try again later
    }
    
    // Simulate USB transmission
    comm_state.system_tick += 2; // USB is faster than UART
    comm_state.usb_tx_count += size;
    comm_state.usb_last_activity = comm_state.system_tick;
    comm_state.usb_tx_busy = true;
    
    // Buffer the data
    for (uint16_t i = 0; i < size && i < sizeof(comm_state.usb_tx_buffer) - 1; i++) {
        uint16_t head = (comm_state.usb_tx_head + 1) % sizeof(comm_state.usb_tx_buffer);
        if (head != comm_state.usb_tx_tail) {
            comm_state.usb_tx_buffer[comm_state.usb_tx_head] = data[i];
            comm_state.usb_tx_head = head;
        }
    }
    
    // Simulate completion callback after short delay
    if (comm_state.system_tick % 10 == 0) {
        comm_state.usb_tx_busy = false; // Complete transmission
    }
    
    return true;
}

// Enhanced system tick with realistic timing
uint32_t enhanced_get_tick(void) {
    comm_state.system_tick++; // Increment each time called
    
    // Simulate periodic heartbeat and connection monitoring
    if (comm_state.system_tick - comm_state.last_heartbeat > 1000) {
        comm_state.last_heartbeat = comm_state.system_tick;
        
        // Simulate USB connection state changes
        if (!comm_state.usb_connected && (comm_state.system_tick % 5000) == 0) {
            comm_state.usb_connected = true;
            comm_state.usb_enumerated = true;
        }
        
        // Simulate occasional USB disconnection for robustness
        if (comm_state.usb_connected && (comm_state.system_tick % 50000) == 0) {
            comm_state.usb_connected = false;
            comm_state.usb_enumerated = false;
        }
    }
    
    return comm_state.system_tick;
}

// Enhanced GPIO operations with state tracking
typedef struct {
    bool pin_states[16][6]; // 16 pins x 6 ports (A-F)
    uint32_t last_read_time;
    uint32_t read_count;
} enhanced_gpio_state_t;

static enhanced_gpio_state_t gpio_state = {0};

bool enhanced_gpio_read(void *port, uint16_t pin) {
    // Convert port pointer to index
    uintptr_t port_addr = (uintptr_t)port;
    int port_index = 0;
    
    if (port_addr == 0x50000000) port_index = 0; // GPIOA
    else if (port_addr == 0x50000400) port_index = 1; // GPIOB  
    else if (port_addr == 0x50000800) port_index = 2; // GPIOC
    else if (port_addr == 0x50000C00) port_index = 3; // GPIOD
    else if (port_addr == 0x50001400) port_index = 5; // GPIOF
    
    // Find pin bit position
    int pin_index = 0;
    for (int i = 0; i < 16; i++) {
        if (pin & (1 << i)) {
            pin_index = i;
            break;
        }
    }
    
    gpio_state.last_read_time = enhanced_get_tick();
    gpio_state.read_count++;
    
    // Simulate realistic pin behavior
    if (port_index == 2 && pin_index >= 12 && pin_index <= 15) {
        // Simulate control inputs (PC12-PC15) - occasionally active
        return (gpio_state.read_count % 1000) < 10; // Active 1% of the time
    }
    
    return gpio_state.pin_states[pin_index][port_index];
}

void enhanced_gpio_write(void *port, uint16_t pin, bool state) {
    // Convert port pointer to index
    uintptr_t port_addr = (uintptr_t)port;
    int port_index = 0;
    
    if (port_addr == 0x50000000) port_index = 0; // GPIOA
    else if (port_addr == 0x50000400) port_index = 1; // GPIOB  
    else if (port_addr == 0x50000800) port_index = 2; // GPIOC
    else if (port_addr == 0x50000C00) port_index = 3; // GPIOD
    else if (port_addr == 0x50001400) port_index = 5; // GPIOF
    
    // Find pin bit position
    int pin_index = 0;
    for (int i = 0; i < 16; i++) {
        if (pin & (1 << i)) {
            pin_index = i;
            break;
        }
    }
    
    gpio_state.pin_states[pin_index][port_index] = state;
}

// Get communication statistics for monitoring
void enhanced_get_comm_stats(char *buffer, size_t size) {
    snprintf(buffer, size,
        "UART: TX=%lu RX=%lu Errors=%lu Activity=%lu\n"
        "USB: TX=%lu RX=%lu Timeouts=%lu Connected=%s\n"
        "System: Tick=%lu Heartbeat=%lu\n",
        comm_state.uart_tx_count, comm_state.uart_rx_count, 
        comm_state.uart_error_count, comm_state.uart_last_activity,
        comm_state.usb_tx_count, comm_state.usb_rx_count,
        comm_state.usb_timeout_count, comm_state.usb_connected ? "Yes" : "No",
        comm_state.system_tick, comm_state.last_heartbeat
    );
}

// Export enhanced functions for HAL integration
bool (*enhanced_comm_init_ptr)(void) = enhanced_comm_init;
bool (*enhanced_uart_transmit_ptr)(const uint8_t *, uint16_t) = enhanced_uart_transmit;
bool (*enhanced_usb_transmit_ptr)(const uint8_t *, uint16_t) = enhanced_usb_transmit;
uint32_t (*enhanced_get_tick_ptr)(void) = enhanced_get_tick;
bool (*enhanced_gpio_read_ptr)(void *, uint16_t) = enhanced_gpio_read;
void (*enhanced_gpio_write_ptr)(void *, uint16_t, bool) = enhanced_gpio_write;