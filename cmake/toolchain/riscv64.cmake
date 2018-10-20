# NB: Depends on host-tools.cmake having been executed already

# Ensures toolchain-gcc.cmake is run
set(COMPILER gcc)

# TODO fetch from environment

set(TOOLCHAIN_VENDOR unknown-linux-gnu)
set(TOOLCHAIN_HOME /opt/rv64gc)

# These defaults work for some targets like RISC-V
set(CROSS_COMPILE_TARGET ${ARCH}-${TOOLCHAIN_VENDOR})
set(SYSROOT_TARGET       ${ARCH}-${TOOLCHAIN_VENDOR})

set(CROSS_COMPILE ${TOOLCHAIN_HOME}/bin/${CROSS_COMPILE_TARGET}-)
set(SYSROOT_DIR ${ZEPHYR_SDK_INSTALL_DIR}/sysroots/${SYSROOT_TARGET}/usr)

list(APPEND TOOLCHAIN_C_FLAGS --sysroot ${SYSROOT_DIR})
