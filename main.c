#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "clock_config.h"
#include "pin_mux.h"
#include "ultrasonic.h"

void dist(int distance) {
    PRINTF("Distance: %d cm\n", distance / 10);
}

int main() {
    ultrasonic_t mu;
    
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    ultrasonic_init_with_callback(&mu, GPIOC, 16U, GPIOA, 1U, TPM0, 1U, 0.1, 1.0, dist);

    ultrasonic_start_updates(&mu);

    while (1) {
        int distance = ultrasonic_get_current_distance(&mu);
        PRINTF("Current distance: %d cm\n", distance / 10);
        ultrasonic_check_distance(&mu);
        for (volatile int i = 0; i < 1000000; i++);
    }
}
