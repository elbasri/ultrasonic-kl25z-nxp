#include "MKL25Z4.h"
#include "ultrasonic.h"
#include <stdio.h> // Include this for printf

void dist(int distance) {
    printf("Distance: %d cm\n", distance / 10);
}

int main(void) {
    // Initialize system
    SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTC_MASK;  // Enable clock for PORTA and PORTC
    SIM_SOPT2 |= SIM_SOPT2_PLLFLLSEL_MASK | SIM_SOPT2_TPMSRC(1); // Select PLLFLLCLK as TPM clock source

    // Initialize debug console
    // Initialize UART for printf (if not already initialized)

    ultrasonic_t mu;
    ultrasonic_init_with_callback(&mu, 16U, 1U, 0.1, 1.0, dist);
    ultrasonic_start_updates(&mu);

    while (1) {
        int distance = ultrasonic_get_current_distance(&mu);
        printf("Current distance: %d cm\n", distance / 10);
        ultrasonic_check_distance(&mu);
        for (volatile int i = 0; i < 1000000; i++);
    }
}
