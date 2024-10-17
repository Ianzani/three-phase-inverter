#ifndef ADC_INTERFACE_H
#define ADC_INTERFACE_H

void adc_interface_init(void);

uint16_t get_bus_voltage(void);

uint16_t get_stator_current(void);

#endif
