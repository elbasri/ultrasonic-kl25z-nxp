#include "ultrasonic.h"
#include "fsl_tpm.h"

static void start_t(ultrasonic_t *ultrasonic);
static void update_dist(ultrasonic_t *ultrasonic);
static void start_trig(ultrasonic_t *ultrasonic);

void ultrasonic_init(ultrasonic_t *ultrasonic, GPIO_Type *trig_base, uint32_t trig_pin, GPIO_Type *echo_base, uint32_t echo_pin, TPM_Type *timer_base, uint32_t tpm_channel, float updateSpeed, float timeout) {
    ultrasonic->trig_config.pinDirection = kGPIO_DigitalOutput;
    GPIO_PinInit(trig_base, trig_pin, &(ultrasonic->trig_config));
    
    ultrasonic->echo_config.pinDirection = kGPIO_DigitalInput;
    GPIO_PinInit(echo_base, echo_pin, &(ultrasonic->echo_config));
    
    ultrasonic->timer_base = timer_base;
    ultrasonic->tpm_channel = tpm_channel;
    ultrasonic->updateSpeed = updateSpeed;
    ultrasonic->timeout = timeout;
    ultrasonic->done = 0;
}

void ultrasonic_init_with_callback(ultrasonic_t *ultrasonic, GPIO_Type *trig_base, uint32_t trig_pin, GPIO_Type *echo_base, uint32_t echo_pin, TPM_Type *timer_base, uint32_t tpm_channel, float updateSpeed, float timeout, ultrasonic_callback_t onUpdate) {
    ultrasonic_init(ultrasonic, trig_base, trig_pin, echo_base, echo_pin, timer_base, tpm_channel, updateSpeed, timeout);
    ultrasonic->onUpdateMethod = onUpdate;
    TPM_StartTimer(timer_base, kTPM_SystemClock);
}

int ultrasonic_get_current_distance(ultrasonic_t *ultrasonic) {
    return ultrasonic->distance;
}

void ultrasonic_pause_updates(ultrasonic_t *ultrasonic) {
    TPM_StopTimer(ultrasonic->timer_base);
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
    if (TPM_GetCurrentTimerCount(ultrasonic->timer_base) > 600) {
        TPM_StopTimer(ultrasonic->timer_base);
        TPM_StartTimer(ultrasonic->timer_base, kTPM_SystemClock);
    }
    ultrasonic->start = TPM_GetCurrentTimerCount(ultrasonic->timer_base);
}

static void update_dist(ultrasonic_t *ultrasonic) {
    ultrasonic->end = TPM_GetCurrentTimerCount(ultrasonic->timer_base);
    ultrasonic->done = 1;
    ultrasonic->distance = (ultrasonic->end - ultrasonic->start) / 6;
    TPM_StopTimer(ultrasonic->timer_base);
    TPM_StartTimer(ultrasonic->timer_base, kTPM_SystemClock);
}

static void start_trig(ultrasonic_t *ultrasonic) {
    GPIO_WritePinOutput(GPIOC, 16U, 1);
    for (volatile int i = 0; i < 100; i++);
    GPIO_WritePinOutput(GPIOC, 16U, 0);
    
    PORT_SetPinInterruptConfig(PORTA, 1U, kPORT_InterruptEitherEdge);
    TPM_StartTimer(ultrasonic->timer_base, kTPM_SystemClock);
}
