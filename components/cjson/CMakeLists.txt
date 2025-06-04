idf_component_register(
    SRCS "cJSON.c"
    INCLUDE_DIRS "."
)

target_compile_definitions(${COMPONENT_LIB} PUBLIC CJSON_HIDE_SYMBOLS) 