idf_component_register(SRCS "can_interface.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver control_loop adc_interface)
