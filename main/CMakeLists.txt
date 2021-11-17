# Embed the server root certificate into the final binary
#
# (If this was a component, we would set COMPONENT_EMBED_TXTFILES here.)
idf_component_register(SRCS "esp32_dht22.c"
                    INCLUDE_DIRS "."
                    EMBED_TXTFILES howsmyssl_com_root_cert.pem iot_dht22_herokuapp_com_root_cert.pem)
