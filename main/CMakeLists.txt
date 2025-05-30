idf_component_register(SRCS "Cuisson.c" "test_touch-buttons.c" "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver is31fl3731
                    REQUIRES driver ht16k33
                    )
