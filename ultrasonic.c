#include "ultrasonic.h"
#include "MKL25Z4.h"

// Forward declarations of static functions
static void start_t(ultrasonic_t *ultrasonic);
static void update_dist(ultrasonic_t *ultrasonic);
static void start_trig(ultrasonic_t *ultrasonic);

// Variable for storing ultrasonic sensor instance
static ultrasonic_t *ultrasonic_instance;

void ultrasonic_init(ultrasonic_t *ultrasonic, uint32_t trig_pin, uint32_t echo_pin, float updateSpeed, float timeout) {
    ultrasonic->trig_pin = trig_pin;
    ultrasonic->echo_pin = echo_pin;
    ultrasonic->updateSpeed = updateSpeed;
    ultrasonic->timeout = timeout;
    ultrasonic->done = 0;

    // Store the instance for use in the interrupt handler
    ultrasonic_instance = ultrasonic;

    // Enable clock for ports
    SIM_SCGC5 |= (SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTC_MASK);

    // Configure trigger pin as output
    PORTC_PCR[trig_pin] = PORT_PCR_MUX(1);
    GPIOC_PDDR |= (1U << trig_pin);
    GPIOC_PCOR |= (1U << trig_pin);

    // Configure echo pin as input
    PORTA_PCR[echo_pin] = PORT_PCR_MUX(1) | PORT_PCR_IRQC(0x09); // Interrupt on rising edge
    GPIOA_PDDR &= ~(1U << echo_pin);

    // Enable TPM0 clock
    SIM_SCGC6 |= SIM_SCGC6_TPM0_MASK;
    SIM_SOPT2 |= SIM_SOPT2_TPMSRC(1); // Select MCGFLLCLK clock or MCGPLLCLK/2

    TPM0_SC = 0;  // Disable TPM0 while configuring
    TPM0_MOD = 0xFFFF;  // Set modulo value
    TPM0_SC = TPM_SC_PS(7);  // Prescaler /128
    TPM0_SC |= TPM_SC_TOIE_MASK;  // Enable overflow interrupt
    NVIC_EnableIRQ(TPM0_IRQn);  // Enable TPM0 interrupt in NVIC

    TPM0_SC |= TPM_SC_CMOD(1);  // Start TPM0
}

void ultrasonic_init_with_callback(ultrasonic_t *ultrasonic, uint32_t trig_pin, uint32_t echo_pin, float updateSpeed, float timeout, ultrasonic_callback_t onUpdate) {
    ultrasonic_init(ultrasonic, trig_pin, echo_pin, updateSpeed, timeout);
    ultrasonic->onUpdateMethod = onUpdate;
}

int ultrasonic_get_current_distance(ultrasonic_t *ultrasonic) {
    return ultrasonic->distance;
}

void ultrasonic_pause_updates(ultrasonic_t *ultrasonic) {
    NVIC_DisableIRQ(PORTA_IRQn);  // Disable echo pin interrupts
}

void ultrasonic_start_updates(ultrasonic_t *ultrasonic) {
    start_trig(ultrasonic);
}

void ultrasonic_attach_on_update(ultrasonic_t *ultrasonic, ultrasonic_callback_t method) {
    ultrasonic->onUpdateMethod = method;
}

void ultrasonic_change_update_speed(ultrasonic_t *ultrasonic, float updateSpeed) {
    ultrasonic->updateSpeed = updateSpeed;
}

float ultrasonic_get_update_speed(ultrasonic_t *ultrasonic) {
    return ultrasonic->updateSpeed;
}

int ultrasonic_is_updated(ultrasonic_t *ultrasonic) {
    int d = ultrasonic->done;
    ultrasonic->done = 0;
    return d;
}

void ultrasonic_check_distance(ultrasonic_t *ultrasonic) {
    if (ultrasonic_is_updated(ultrasonic)) {
        ultrasonic->onUpdateMethod(ultrasonic->distance);
    }
}

static void start_t(ultrasonic_t *ultrasonic) {
    ultrasonic->start = TPM0_CNT;
}

static void update_dist(ultrasonic_t *ultrasonic) {
    ultrasonic->end = TPM0_CNT;
    ultrasonic->done = 1;
    ultrasonic->distance = (ultrasonic->end - ultrasonic->start) / 6;
}

static void start_trig(ultrasonic_t *ultrasonic) {
    GPIOC_PSOR |= (1U << ultrasonic->trig_pin);  // Set trigger pin high
    for (volatile int i = 0; i < 1000; i++);  // Delay
    GPIOC_PCOR |= (1U << ultrasonic->trig_pin);  // Set trigger pin low

    NVIC_EnableIRQ(PORTA_IRQn);  // Enable echo pin interrupts
}

void PORTA_IRQHandler(void) {
    if (PORTA_ISFR & (1U << ultrasonic_instance->echo_pin)) {  // Check if interrupt occurred on echo pin
        PORTA_ISFR |= (1U << ultrasonic_instance->echo_pin);  // Clear interrupt flag
        // Handle echo pin interrupt
        static int isRisingEdge = 1;
        if (isRisingEdge) {
            start_t(ultrasonic_instance);
            PORTA_PCR[ultrasonic_instance->echo_pin] = (PORTA_PCR[ultrasonic_instance->echo_pin] & ~PORT_PCR_IRQC_MASK) | PORT_PCR_IRQC(0x0A);  // Falling edge
        } else {
            update_dist(ultrasonic_instance);
            PORTA_PCR[ultrasonic_instance->echo_pin] = (PORTA_PCR[ultrasonic_instance->echo_pin] & ~PORT_PCR_IRQC_MASK) | PORT_PCR_IRQC(0x09);  // Rising edge
        }
        isRisingEdge = !isRisingEdge;
    }
}

void TPM0_IRQHandler(void) {
    if (TPM0_SC & TPM_SC_TOF_MASK) {
        TPM0_SC &= ~TPM_SC_TOF_MASK;  // Clear overflow flag
    }
}
