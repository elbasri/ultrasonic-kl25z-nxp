#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_tpm.h"

typedef void (*ultrasonic_callback_t)(int);

typedef struct {
    gpio_pin_config_t trig_config;
    gpio_pin_config_t echo_config;
    TPM_Type *timer_base;
    tpm_chnl_pwm_signal_param_t tpm_param;
    uint32_t tpm_source_clock;
    uint32_t tpm_channel;
    int distance;
    float updateSpeed;
    float timeout;
    int start;
    int end;
    volatile int done;
    ultrasonic_callback_t onUpdateMethod;
} ultrasonic_t;

void ultrasonic_init(ultrasonic_t *ultrasonic, GPIO_Type *trig_base, uint32_t trig_pin, GPIO_Type *echo_base, uint32_t echo_pin, TPM_Type *timer_base, uint32_t tpm_channel, float updateSpeed, float timeout);
void ultrasonic_init_with_callback(ultrasonic_t *ultrasonic, GPIO_Type *trig_base, uint32_t trig_pin, GPIO_Type *echo_base, uint32_t echo_pin, TPM_Type *timer_base, uint32_t tpm_channel, float updateSpeed, float timeout, ultrasonic_callback_t onUpdate);
int ultrasonic_get_current_distance(ultrasonic_t *ultrasonic);
void ultrasonic_pause_updates(ultrasonic_t *ultrasonic);
void ultrasonic_start_updates(ultrasonic_t *ultrasonic);
void ultrasonic_attach_on_update(ultrasonic_t *ultrasonic, ultrasonic_callback_t method);
void ultrasonic_change_update_speed(ultrasonic_t *ultrasonic, float updateSpeed);
float ultrasonic_get_update_speed(ultrasonic_t *ultrasonic);
int ultrasonic_is_updated(ultrasonic_t *ultrasonic);
void ultrasonic_check_distance(ultrasonic_t *ultrasonic);

#endif
