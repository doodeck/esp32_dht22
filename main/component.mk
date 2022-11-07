#
# "main" pseudo-component makefile.
#
# (Uses default behaviour of compiling all source files in directory, adding 'include' to include path.)

# embed files from the "certs" directory as binary data symbols
# in the app
COMPONENT_EMBED_TXTFILES := (howsmyssl_com_root_cert.pem iot_dht22_herokuapp_com_root_cert.pem next_dht22_vercel_app_root_cert.pem)
