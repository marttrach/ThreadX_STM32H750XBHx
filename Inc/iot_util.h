#ifndef IOT_UTIL_H
#define IOT_UTIL_H

#include "tim.h"
#include "adc.h"

#define HUB_ENC_PIN(PORT_CHAR, NUM)  (uint16_t)(((PORT_CHAR - 'A') << 8) | (NUM))
#define HUB_GET_PORT(code)  ((GPIO_TypeDef *)(((code >> 8) & 0xFF) * 0x400UL + (uint32_t)GPIOA))
#define HUB_GET_PIN(code)   (uint16_t)(1U << (code & 0x0F))

static void hub_apply_gpio_cfg(const hub_cfg_payload_t *cfg)
{
    GPIO_InitTypeDef init = {0};
    init.Pin   = HUB_GET_PIN(cfg->pin_code);

    switch (cfg->cfg) {
        case HUB_IOCT_CFG_INPUT:    init.Mode = GPIO_MODE_INPUT;         break;
        case HUB_IOCT_CFG_OUTPUT:   init.Mode = GPIO_MODE_OUTPUT_PP;     break;
        case HUB_IOCT_CFG_OD:       init.Mode = GPIO_MODE_OUTPUT_OD;     break;
        case HUB_IOCT_CFG_ANALOG:   init.Mode = GPIO_MODE_ANALOG;        break;
        case HUB_IOCT_CFG_AF:       init.Mode = GPIO_MODE_AF_PP;         break;
        default:                    init.Mode = GPIO_MODE_INPUT;         break;
    }
    /* Pull */
    if (cfg->cfg == HUB_IOCT_CFG_PULLUP)   init.Pull = GPIO_PULLUP;
    else if (cfg->cfg == HUB_IOCT_CFG_PULLDOWN) init.Pull = GPIO_PULLDOWN;
    else init.Pull = GPIO_NOPULL;
    /* AF */
    if (init.Mode == GPIO_MODE_AF_PP || init.Mode == GPIO_MODE_AF_OD)
        init.Alternate = cfg->af;

    GPIO_TypeDef *port = HUB_GET_PORT(cfg->pin_code);
    HAL_GPIO_DeInit(port, init.Pin);
    HAL_GPIO_Init(port, &init);
}

/**
 * GPIO
 */
/* PIN */
#define HUB_GPIO_PIN_1 HUB_ENC_PIN('D', 11)
#define HUB_GPIO_PIN_2 HUB_ENC_PIN('D', 12)
#define HUB_GPIO_PIN_3 HUB_ENC_PIN('D', 13)
#define HUB_GPIO_PIN_4 HUB_ENC_PIN('A', 3)
#define HUB_GPIO_PIN_5 HUB_ENC_PIN('A', 4)

/**
 * PWM
 */
/* PIN */
#define HUB_PWM_PIN_1 HUB_ENC_PIN('A', 2)
#define HUB_PWM_PIN_2 HUB_ENC_PIN('B', 0)
#define HUB_PWM_PIN_3 HUB_ENC_PIN('A', 15)
#define HUB_PWM_PIN_4 HUB_ENC_PIN('B', 3)
#define HUB_PWM_PIN_5 HUB_ENC_PIN('H', 6)
#define HUB_PWM_PIN_6 HUB_ENC_PIN('C', 7)

typedef struct {
    uint16_t             pin_code;      /* HUB_PWM_PIN_* Marco */
    TIM_HandleTypeDef   *htim;
    uint32_t             ch;
    void (*mx_reinit)(void);            /* Reinit (AF mode)  */
} pwm_map_t;

static const pwm_map_t pwm_map[] = {
    { HUB_PWM_PIN_1, &htim2 , TIM_CHANNEL_3, MX_TIM2_Init  }, /* PA2  */
    { HUB_PWM_PIN_2, &htim3 , TIM_CHANNEL_3, MX_TIM3_Init  }, /* PB0  */
    { HUB_PWM_PIN_3, &htim2 , TIM_CHANNEL_1, MX_TIM2_Init  }, /* PA15 */
    { HUB_PWM_PIN_4, &htim2 , TIM_CHANNEL_2, MX_TIM2_Init  }, /* PB3  */
    { HUB_PWM_PIN_5, &htim12, TIM_CHANNEL_1, MX_TIM12_Init }, /* PH6  */
    { HUB_PWM_PIN_6, &htim8 , TIM_CHANNEL_2, MX_TIM8_Init  }, /* PC7  */
};

static const pwm_map_t *hub_pwm_lookup(uint16_t pin_code)
{
    for (size_t i = 0; i < sizeof(pwm_map)/sizeof(pwm_map[0]); ++i) {
        if (pwm_map[i].pin_code == pin_code) return &pwm_map[i];
    }
    return NULL;
}

static void hub_pwm_to_gpio_generic(const pwm_map_t *m, const hub_cfg_payload_t *cfg)
{
    HAL_TIM_PWM_DeInit(m->htim);
    hub_apply_gpio_cfg(cfg);
}
/* END PWM */

/**
 * ADC
 */
/* PIN */
#define HUB_ADC_PIN_0 HUB_ENC_PIN('C', 2)
#define HUB_ADC_PIN_1 HUB_ENC_PIN('A', 6)
#define HUB_ADC_PIN_2 HUB_ENC_PIN('C', 4)
#define HUB_ADC_PIN_3 HUB_ENC_PIN('B', 1)
#define HUB_ADC_PIN_4 HUB_ENC_PIN('C', 0)
#define HUB_ADC_PIN_5 HUB_ENC_PIN('C', 1)

typedef struct {
    uint16_t pin_code;
    uint32_t ch;                 /* ADC_CHANNEL_x Marco */
} adc_map_t;

static const adc_map_t adc_map[] = {
    { HUB_ADC_PIN_0, ADC_CHANNEL_12 }, /* PC2  */
    { HUB_ADC_PIN_1, ADC_CHANNEL_3  }, /* PA6  */
    { HUB_ADC_PIN_2, ADC_CHANNEL_4  }, /* PC4  */
    { HUB_ADC_PIN_3, ADC_CHANNEL_5  }, /* PB1  */
    { HUB_ADC_PIN_4, ADC_CHANNEL_10 }, /* PC0  */
    { HUB_ADC_PIN_5, ADC_CHANNEL_11 }, /* PC1  */
};

static const adc_map_t *hub_adc_lookup(uint16_t pin_code)
{
    for (size_t i = 0; i < sizeof(adc_map)/sizeof(adc_map[0]); ++i) {
        if (adc_map[i].pin_code == pin_code) return &adc_map[i];
    }
    return NULL;
}

static void hub_adc_to_gpio(uint16_t pin_code, const hub_cfg_payload_t *cfg)
{
    HAL_ADC_DeInit(&hadc1);
    hub_apply_gpio_cfg(cfg);
}
/* END ADC */


#endif /* IOT_UTIL_H */