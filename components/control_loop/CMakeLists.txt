idf_component_register(SRCS "control_loop.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver encoder sin_calculator manager)
