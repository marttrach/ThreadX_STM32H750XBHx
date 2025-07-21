#ifndef I2C_HUB_H
#define I2C_HUB_H

/* Command targets */
#define HUB_TARGET_UART 0
#define HUB_TARGET_GPIO 1

/* GPIO operations */
#define HUB_GPIO_CFG_PULLUP 1

void hub_start(void);

#endif // I2C_HUB_H
