#if defined(CONFIG_GEN_ISR_TABLES) && !defined(CONFIG_DYNAMIC_INTERRUPTS)
		*(SW_ISR_TABLE)
#endif
#ifdef CONFIG_CPLUSPLUS
		. = ALIGN(4);
		__CTOR_LIST__ = .;
		LONG((__CTOR_END__ - __CTOR_LIST__) / 4 - 2)
		KEEP(*(SORT_BY_NAME(".ctors*")))
		LONG(0)
		__CTOR_END__ = .;


		. = ALIGN(4);
		__init_array_start = .;
		KEEP(*(SORT_BY_NAME(".init_array*")))
		__init_array_end = .;
#endif

#ifdef CONFIG_USERSPACE
		__object_access_start = .;
		KEEP(*(".object_access.*"))
		__object_access_end = .;
#endif
		__devconfig_start = .;
		*(".devconfig.*")
		KEEP(*(SORT_BY_NAME(".devconfig*")))
		__devconfig_end = .;

		__net_l2_start = .;
		*(".net_l2.init")
		KEEP(*(SORT_BY_NAME(".net_l2.init*")))
		__net_l2_end = .;

#if defined(CONFIG_BT_SETTINGS)
		_bt_settings_start = .;
		KEEP(*(SORT_BY_NAME("._bt_settings.static.*")))
		_bt_settings_end = .;
#endif
		__log_const_start = .;
		KEEP(*(SORT(.log_const_*)));
		__log_const_end = .;

		__log_backends_start = .;
		KEEP(*(".log_backends"));
		__log_backends_end = .;

		__shell_root_cmds_start = .;
		KEEP(*(SORT(.shell_root_cmd_*)));
		__shell_root_cmds_end = .;

		__font_entry_start = .;
		KEEP(*(SORT_BY_NAME(".font_entry.*")))
		__font_entry_end = .;
