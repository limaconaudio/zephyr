cflags-y += $(call cc-option,-ffunction-sections,) $(call cc-option,-fdata-sections,)
cflags-$(CONFIG_LTO) = $(call cc-option,-flto,)

soc_ld_include := -I$(srctree)/arch/$(ARCH)/soc/$(SOC_PATH)
arch_cflags += $(soc_ld_include)
EXTRA_LINKER_CMD_OPT += $(soc_ld_include)

include $(srctree)/arch/$(ARCH)/soc/$(SOC_PATH)/Makefile

KBUILD_CFLAGS += $(cflags-y)
KBUILD_CFLAGS += -Wno-unused-function
KBUILD_CXXFLAGS += $(cflags-y)
