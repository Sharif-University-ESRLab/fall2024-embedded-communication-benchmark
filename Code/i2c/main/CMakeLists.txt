set(srcs "i2c_slave_main.c")

idf_component_register(SRCS ${srcs}
                    PRIV_REQUIRES driver esp_event json esp_driver_i2c esp_timer
                    INCLUDE_DIRS ".")
