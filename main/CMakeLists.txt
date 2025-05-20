idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    # PRIV_INCLUDE_DIRS "$ENV{IDF_PATH}/components/driver/private_include"
                    REQUIRES driver
                    )
