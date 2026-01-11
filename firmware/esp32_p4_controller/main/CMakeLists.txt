idf_component_register(
    SRCS 
        "main.c"
        "canopen_master.c"
        "eth_comm.c"
        "trajectory_task.c"
        "display_task.c"
        "standalone_mode.c"
    INCLUDE_DIRS 
        "."
        "../../common/canopen"
    REQUIRES
        driver
        esp_eth
        esp_lcd
        esp_timer
        freertos
        nvs_flash
        lwip
)

# Enable TWAI (CAN) driver
# ESP32-P4 has 3 TWAI controllers
