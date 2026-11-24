idf_component_register(SRCS "ros.cpp" "main.cpp" "parameters.cpp" "webserver.cpp" "servo.cpp"
                    INCLUDE_DIRS "." "./include")

# Apply flags only to this component's library target
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wall")
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wextra")
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wpedantic")
