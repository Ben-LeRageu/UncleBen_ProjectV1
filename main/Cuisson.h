#ifndef CUISSON_CONTROL_H
#define CUISSON_CONTROL_H

#include <stdbool.h>

void start_cuisson();
void stop_cuisson();
void maintenir_temperature(float temperature_cible);
void cuisson_gpio_init();
#endif // CUISSON_CONTROL_H
