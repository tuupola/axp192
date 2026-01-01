add_executable(unit unit.c mock_i2c.c)
target_link_libraries(unit PRIVATE axp192)
target_compile_options(unit PRIVATE
    -g
    -Wall
    -Wextra
    -Werror
    -Wmissing-prototypes
    -Wstrict-prototypes
)

add_test(NAME unit COMMAND unit)
