idf_component_register(SRCS "manager.c"
                    INCLUDE_DIRS "include"
                    REQUIRES can_interface control_loop sin_calculator pwm_control)
