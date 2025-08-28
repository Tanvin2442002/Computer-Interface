file(GLOB_RECURSE srcs "main.c" "src/*.c")

idf_component_register(SRCS "${srcs}"
                       PRIV_REQUIRES bt nvs_flash esp_driver_gpio esp_driver_ledc esp_wifi esp_http_server esp_netif esp_event json driver
                       INCLUDE_DIRS "./include")
# file(GLOB_RECURSE srcs "main.c" "src/*.c")

# idf_component_register(
#     SRCS ${srcs}                  # no quotes here!
#     PRIV_REQUIRES bt nvs_flash driver    # usually `driver` not `esp_driver_gpio`
#     INCLUDE_DIRS "include"
# )
