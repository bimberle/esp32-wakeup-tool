idf_component_register(SRCS "main.c" "usb.c" "tof_sensor.c" "led.c" "led_strip_encoder.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver)
