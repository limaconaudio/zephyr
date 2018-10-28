add_subdirectory(core)

# FIXME: For the time being, keep arch specific flags hardcoded
zephyr_compile_options(
  -mabi=lp64d
  -march=rv64imafdc
  )

set_property(GLOBAL PROPERTY PROPERTY_OUTPUT_FORMAT elf64-littleriscv)
