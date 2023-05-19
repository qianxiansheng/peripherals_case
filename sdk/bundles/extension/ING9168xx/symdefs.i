--define_symbol att_dispatch_client_can_send_now=0x02004449
--define_symbol att_dispatch_client_request_can_send_now_event=0x0200444f
--define_symbol att_dispatch_register_client=0x02004455
--define_symbol att_dispatch_register_server=0x02004469
--define_symbol att_dispatch_server_can_send_now=0x0200447d
--define_symbol att_dispatch_server_request_can_send_now_event=0x02004483
--define_symbol att_emit_general_event=0x02004535
--define_symbol att_server_can_send_packet_now=0x02004c51
--define_symbol att_server_deferred_read_response=0x02004c55
--define_symbol att_server_get_mtu=0x02004c6d
--define_symbol att_server_indicate=0x02004ce5
--define_symbol att_server_init=0x02004d69
--define_symbol att_server_notify=0x02004da5
--define_symbol att_server_register_packet_handler=0x02004ebd
--define_symbol att_server_request_can_send_now_event=0x02004ec9
--define_symbol att_set_db=0x02004ee5
--define_symbol att_set_read_callback=0x02004ef9
--define_symbol att_set_write_callback=0x02004f05
--define_symbol bd_addr_cmp=0x020050ed
--define_symbol bd_addr_copy=0x020050f3
--define_symbol bd_addr_to_str=0x020050fd
--define_symbol big_endian_read_16=0x02005135
--define_symbol big_endian_read_32=0x0200513d
--define_symbol big_endian_store_16=0x02005151
--define_symbol big_endian_store_32=0x0200515d
--define_symbol btstack_config=0x020052bd
--define_symbol btstack_memory_pool_create=0x020053fb
--define_symbol btstack_memory_pool_free=0x02005425
--define_symbol btstack_memory_pool_get=0x02005485
--define_symbol btstack_push_user_msg=0x020054ed
--define_symbol btstack_push_user_runnable=0x020054f9
--define_symbol char_for_nibble=0x02005797
--define_symbol eTaskConfirmSleepModeStatus=0x02005ae9
--define_symbol gap_add_dev_to_periodic_list=0x0200618d
--define_symbol gap_add_whitelist=0x0200619d
--define_symbol gap_aes_encrypt=0x020061a9
--define_symbol gap_clear_white_lists=0x020061ed
--define_symbol gap_clr_adv_set=0x020061f9
--define_symbol gap_clr_periodic_adv_list=0x02006205
--define_symbol gap_create_connection_cancel=0x02006211
--define_symbol gap_default_periodic_adv_sync_transfer_param=0x0200621d
--define_symbol gap_disconnect=0x02006235
--define_symbol gap_disconnect_all=0x02006261
--define_symbol gap_ext_create_connection=0x020062a1
--define_symbol gap_get_connection_parameter_range=0x02006391
--define_symbol gap_le_read_channel_map=0x020063cd
--define_symbol gap_periodic_adv_create_sync=0x02006439
--define_symbol gap_periodic_adv_create_sync_cancel=0x0200645d
--define_symbol gap_periodic_adv_set_info_transfer=0x02006469
--define_symbol gap_periodic_adv_sync_transfer=0x02006479
--define_symbol gap_periodic_adv_sync_transfer_param=0x02006489
--define_symbol gap_periodic_adv_term_sync=0x020064a5
--define_symbol gap_read_antenna_info=0x0200652d
--define_symbol gap_read_periodic_adv_list_size=0x02006539
--define_symbol gap_read_phy=0x02006545
--define_symbol gap_read_remote_used_features=0x02006551
--define_symbol gap_read_remote_version=0x0200655d
--define_symbol gap_read_rssi=0x02006569
--define_symbol gap_remove_whitelist=0x02006575
--define_symbol gap_rmv_adv_set=0x020065f1
--define_symbol gap_rmv_dev_from_periodic_list=0x020065fd
--define_symbol gap_rx_test_v2=0x0200660d
--define_symbol gap_rx_test_v3=0x0200661d
--define_symbol gap_set_adv_set_random_addr=0x02006669
--define_symbol gap_set_connection_cte_request_enable=0x020066b1
--define_symbol gap_set_connection_cte_response_enable=0x020066cd
--define_symbol gap_set_connection_cte_rx_param=0x020066dd
--define_symbol gap_set_connection_cte_tx_param=0x02006739
--define_symbol gap_set_connection_parameter_range=0x0200678d
--define_symbol gap_set_connectionless_cte_tx_enable=0x020067a5
--define_symbol gap_set_connectionless_cte_tx_param=0x020067b5
--define_symbol gap_set_connectionless_iq_sampling_enable=0x02006815
--define_symbol gap_set_data_length=0x02006879
--define_symbol gap_set_def_phy=0x02006891
--define_symbol gap_set_ext_adv_data=0x020068a1
--define_symbol gap_set_ext_adv_enable=0x020068b9
--define_symbol gap_set_ext_adv_para=0x02006935
--define_symbol gap_set_ext_scan_enable=0x02006a05
--define_symbol gap_set_ext_scan_para=0x02006a1d
--define_symbol gap_set_ext_scan_response_data=0x02006ac5
--define_symbol gap_set_host_channel_classification=0x02006add
--define_symbol gap_set_periodic_adv_data=0x02006aed
--define_symbol gap_set_periodic_adv_enable=0x02006b5d
--define_symbol gap_set_periodic_adv_para=0x02006b6d
--define_symbol gap_set_periodic_adv_rx_enable=0x02006b85
--define_symbol gap_set_phy=0x02006b95
--define_symbol gap_set_random_device_address=0x02006bb1
--define_symbol gap_start_ccm=0x02006be1
--define_symbol gap_test_end=0x02006c35
--define_symbol gap_tx_test_v2=0x02006c41
--define_symbol gap_tx_test_v4=0x02006c59
--define_symbol gap_update_connection_parameters=0x02006c7d
--define_symbol gap_vendor_tx_continuous_wave=0x02006cc1
--define_symbol gatt_client_cancel_write=0x020071e9
--define_symbol gatt_client_discover_characteristic_descriptors=0x0200720f
--define_symbol gatt_client_discover_characteristics_for_handle_range_by_uuid128=0x0200724f
--define_symbol gatt_client_discover_characteristics_for_handle_range_by_uuid16=0x0200729f
--define_symbol gatt_client_discover_characteristics_for_service=0x020072ef
--define_symbol gatt_client_discover_primary_services=0x02007325
--define_symbol gatt_client_discover_primary_services_by_uuid128=0x02007357
--define_symbol gatt_client_discover_primary_services_by_uuid16=0x0200739b
--define_symbol gatt_client_execute_write=0x020073d9
--define_symbol gatt_client_find_included_services_for_service=0x020073ff
--define_symbol gatt_client_get_mtu=0x0200742d
--define_symbol gatt_client_is_ready=0x020074f1
--define_symbol gatt_client_listen_for_characteristic_value_updates=0x02007507
--define_symbol gatt_client_prepare_write=0x02007529
--define_symbol gatt_client_read_characteristic_descriptor_using_descriptor_handle=0x02007565
--define_symbol gatt_client_read_long_characteristic_descriptor_using_descriptor_handle=0x0200758f
--define_symbol gatt_client_read_long_characteristic_descriptor_using_descriptor_handle_with_offset=0x02007595
--define_symbol gatt_client_read_long_value_of_characteristic_using_value_handle=0x020075c3
--define_symbol gatt_client_read_long_value_of_characteristic_using_value_handle_with_offset=0x020075c9
--define_symbol gatt_client_read_multiple_characteristic_values=0x020075f7
--define_symbol gatt_client_read_value_of_characteristic_using_value_handle=0x02007627
--define_symbol gatt_client_read_value_of_characteristics_by_uuid128=0x02007655
--define_symbol gatt_client_read_value_of_characteristics_by_uuid16=0x020076a1
--define_symbol gatt_client_register_handler=0x020076ed
--define_symbol gatt_client_reliable_write_long_value_of_characteristic=0x020076f9
--define_symbol gatt_client_signed_write_without_response=0x02007b2d
--define_symbol gatt_client_write_characteristic_descriptor_using_descriptor_handle=0x02007bf1
--define_symbol gatt_client_write_client_characteristic_configuration=0x02007c2b
--define_symbol gatt_client_write_long_characteristic_descriptor_using_descriptor_handle=0x02007c7d
--define_symbol gatt_client_write_long_characteristic_descriptor_using_descriptor_handle_with_offset=0x02007c8d
--define_symbol gatt_client_write_long_value_of_characteristic=0x02007cc9
--define_symbol gatt_client_write_long_value_of_characteristic_with_offset=0x02007cd9
--define_symbol gatt_client_write_value_of_characteristic=0x02007d15
--define_symbol gatt_client_write_value_of_characteristic_without_response=0x02007d4b
--define_symbol hci_add_event_handler=0x02009345
--define_symbol hci_power_control=0x02009b45
--define_symbol hci_register_acl_packet_handler=0x02009cf9
--define_symbol kv_commit=0x0200a22d
--define_symbol kv_get=0x0200a285
--define_symbol kv_init=0x0200a29d
--define_symbol kv_put=0x0200a305
--define_symbol kv_remove=0x0200a37d
--define_symbol kv_remove_all=0x0200a3b9
--define_symbol kv_value_modified=0x0200a3fd
--define_symbol kv_visit=0x0200a401
--define_symbol l2cap_add_event_handler=0x0200a4b5
--define_symbol l2cap_can_send_packet_now=0x0200a4c5
--define_symbol l2cap_create_le_credit_based_connection_request=0x0200a689
--define_symbol l2cap_credit_based_send=0x0200a7d1
--define_symbol l2cap_credit_based_send_continue=0x0200a7fd
--define_symbol l2cap_disconnect=0x0200a879
--define_symbol l2cap_get_le_credit_based_connection_credits=0x0200aa65
--define_symbol l2cap_get_peer_mtu_for_local_cid=0x0200aa81
--define_symbol l2cap_init=0x0200aec1
--define_symbol l2cap_le_send_flow_control_credit=0x0200afb7
--define_symbol l2cap_max_le_mtu=0x0200b2c5
--define_symbol l2cap_register_packet_handler=0x0200b3ed
--define_symbol l2cap_register_service=0x0200b3f9
--define_symbol l2cap_request_can_send_now_event=0x0200b509
--define_symbol l2cap_request_connection_parameter_update=0x0200b523
--define_symbol l2cap_send_echo_request=0x0200ba05
--define_symbol l2cap_unregister_service=0x0200bae5
--define_symbol le_device_db_add=0x0200bb3d
--define_symbol le_device_db_find=0x0200bc11
--define_symbol le_device_db_from_key=0x0200bc3d
--define_symbol le_device_db_iter_cur=0x0200bc45
--define_symbol le_device_db_iter_cur_key=0x0200bc49
--define_symbol le_device_db_iter_init=0x0200bc4d
--define_symbol le_device_db_iter_next=0x0200bc55
--define_symbol le_device_db_remove_key=0x0200bc7b
--define_symbol ll_ackable_packet_alloc=0x0200bca7
--define_symbol ll_ackable_packet_get_status=0x0200bdb3
--define_symbol ll_ackable_packet_run=0x0200be21
--define_symbol ll_ackable_packet_set_tx_data=0x0200bee5
--define_symbol ll_adjust_conn_peer_tx_power=0x0200bf01
--define_symbol ll_aes_encrypt=0x0200bf2d
--define_symbol ll_allow_nonstandard_adv_type=0x0200bfa1
--define_symbol ll_attach_cte_to_adv_set=0x0200bfb9
--define_symbol ll_free=0x0200c151
--define_symbol ll_hint_on_ce_len=0x0200c15d
--define_symbol ll_legacy_adv_set_interval=0x0200c195
--define_symbol ll_lock_frequency=0x0200c1a5
--define_symbol ll_malloc=0x0200c1f9
--define_symbol ll_override_whitening_init_value=0x0200c201
--define_symbol ll_raw_packet_alloc=0x0200c329
--define_symbol ll_raw_packet_free=0x0200c3fd
--define_symbol ll_raw_packet_get_bare_rx_data=0x0200c427
--define_symbol ll_raw_packet_get_iq_samples=0x0200c4e7
--define_symbol ll_raw_packet_get_rx_data=0x0200c581
--define_symbol ll_raw_packet_recv=0x0200c615
--define_symbol ll_raw_packet_send=0x0200c6e9
--define_symbol ll_raw_packet_set_bare_data=0x0200c805
--define_symbol ll_raw_packet_set_bare_mode=0x0200c843
--define_symbol ll_raw_packet_set_param=0x0200c947
--define_symbol ll_raw_packet_set_rx_cte=0x0200c9a9
--define_symbol ll_raw_packet_set_tx_cte=0x0200ca3f
--define_symbol ll_raw_packet_set_tx_data=0x0200ca7d
--define_symbol ll_scan_set_fixed_channel=0x0200cb35
--define_symbol ll_scanner_enable_iq_sampling=0x0200cb41
--define_symbol ll_scanner_enable_iq_sampling_on_legacy=0x0200cbe9
--define_symbol ll_set_adv_access_address=0x0200cdd5
--define_symbol ll_set_adv_coded_scheme=0x0200cde1
--define_symbol ll_set_conn_coded_scheme=0x0200ce19
--define_symbol ll_set_conn_interval_unit=0x0200ce45
--define_symbol ll_set_conn_latency=0x0200ce51
--define_symbol ll_set_conn_tx_power=0x0200ce81
--define_symbol ll_set_cte_bit=0x0200ceb1
--define_symbol ll_set_def_antenna=0x0200ced9
--define_symbol ll_set_initiating_coded_scheme=0x0200cef9
--define_symbol ll_set_max_conn_number=0x0200cf05
--define_symbol ll_set_tx_power_range=0x0200cfa1
--define_symbol ll_unlock_frequency=0x0200cfc9
--define_symbol nibble_for_char=0x0201fb25
--define_symbol platform_32k_rc_auto_tune=0x0201fbd1
--define_symbol platform_32k_rc_tune=0x0201fc1d
--define_symbol platform_calibrate_32k=0x0201fc39
--define_symbol platform_config=0x0201fc3d
--define_symbol platform_enable_irq=0x0201fd61
--define_symbol platform_get_current_task=0x0201fd95
--define_symbol platform_get_gen_os_driver=0x0201fdb9
--define_symbol platform_get_heap_status=0x0201fdc1
--define_symbol platform_get_task_handle=0x0201fdd9
--define_symbol platform_get_us_time=0x0201fdf9
--define_symbol platform_get_version=0x0201fdfd
--define_symbol platform_hrng=0x0201fe05
--define_symbol platform_install_isr_stack=0x0201fe0d
--define_symbol platform_install_task_stack=0x0201fe19
--define_symbol platform_patch_rf_init_data=0x0201fe51
--define_symbol platform_printf=0x0201fe5d
--define_symbol platform_raise_assertion=0x0201fe71
--define_symbol platform_rand=0x0201fe85
--define_symbol platform_read_info=0x0201fe89
--define_symbol platform_read_persistent_reg=0x0201feb9
--define_symbol platform_reset=0x0201fec5
--define_symbol platform_set_evt_callback=0x0201fed9
--define_symbol platform_set_evt_callback_table=0x0201feed
--define_symbol platform_set_irq_callback=0x0201fef9
--define_symbol platform_set_irq_callback_table=0x0201ff15
--define_symbol platform_set_rf_clk_source=0x0201ff21
--define_symbol platform_set_rf_init_data=0x0201ff2d
--define_symbol platform_set_rf_power_mapping=0x0201ff39
--define_symbol platform_set_timer=0x0201ff45
--define_symbol platform_shutdown=0x0201ff49
--define_symbol platform_switch_app=0x0201ff4d
--define_symbol platform_trace_raw=0x0201ff65
--define_symbol platform_write_persistent_reg=0x0201ff7d
--define_symbol printf_hexdump=0x02020135
--define_symbol pvPortMalloc=0x02020c09
--define_symbol pvTaskIncrementMutexHeldCount=0x02020cf1
--define_symbol pvTimerGetTimerID=0x02020d09
--define_symbol pxPortInitialiseStack=0x02020d35
--define_symbol reverse_128=0x02020f1d
--define_symbol reverse_24=0x02020f23
--define_symbol reverse_48=0x02020f29
--define_symbol reverse_56=0x02020f2f
--define_symbol reverse_64=0x02020f35
--define_symbol reverse_bd_addr=0x02020f3b
--define_symbol reverse_bytes=0x02020f41
--define_symbol sm_add_event_handler=0x0202127d
--define_symbol sm_address_resolution_lookup=0x020213d5
--define_symbol sm_authenticated=0x02021735
--define_symbol sm_authorization_decline=0x02021743
--define_symbol sm_authorization_grant=0x02021763
--define_symbol sm_authorization_state=0x02021783
--define_symbol sm_bonding_decline=0x0202179d
--define_symbol sm_config=0x02021bb5
--define_symbol sm_config_conn=0x02021bcd
--define_symbol sm_encryption_key_size=0x02021d83
--define_symbol sm_just_works_confirm=0x020222bd
--define_symbol sm_le_device_key=0x020225f9
--define_symbol sm_passkey_input=0x0202268f
--define_symbol sm_private_random_address_generation_get=0x02022a3d
--define_symbol sm_private_random_address_generation_get_mode=0x02022a45
--define_symbol sm_private_random_address_generation_set_mode=0x02022a51
--define_symbol sm_private_random_address_generation_set_update_period=0x02022a79
--define_symbol sm_register_oob_data_callback=0x02022bb5
--define_symbol sm_request_pairing=0x02022bc1
--define_symbol sm_send_security_request=0x020235ff
--define_symbol sm_set_accepted_stk_generation_methods=0x02023625
--define_symbol sm_set_authentication_requirements=0x02023631
--define_symbol sm_set_encryption_key_size_range=0x0202363d
--define_symbol sscanf_bd_addr=0x02023a15
--define_symbol sysSetPublicDeviceAddr=0x02023e81
--define_symbol uuid128_to_str=0x0202462d
--define_symbol uuid_add_bluetooth_prefix=0x02024685
--define_symbol uuid_has_bluetooth_prefix=0x020246a5
--define_symbol uxListRemove=0x020246c1
--define_symbol uxQueueMessagesWaiting=0x020246e9
--define_symbol uxQueueMessagesWaitingFromISR=0x02024711
--define_symbol uxQueueSpacesAvailable=0x0202472d
--define_symbol uxTaskGetStackHighWaterMark=0x02024759
--define_symbol uxTaskPriorityGet=0x02024779
--define_symbol uxTaskPriorityGetFromISR=0x02024795
--define_symbol vListInitialise=0x0202483b
--define_symbol vListInitialiseItem=0x02024851
--define_symbol vListInsert=0x02024857
--define_symbol vListInsertEnd=0x02024887
--define_symbol vPortEndScheduler=0x020248a1
--define_symbol vPortEnterCritical=0x020248cd
--define_symbol vPortExitCritical=0x02024911
--define_symbol vPortFree=0x02024945
--define_symbol vPortSuppressTicksAndSleep=0x020249e9
--define_symbol vPortValidateInterruptPriority=0x02024b11
--define_symbol vQueueDelete=0x02024b6d
--define_symbol vQueueWaitForMessageRestricted=0x02024b99
--define_symbol vTaskDelay=0x02024bdd
--define_symbol vTaskInternalSetTimeOutState=0x02024c29
--define_symbol vTaskMissedYield=0x02024c39
--define_symbol vTaskPlaceOnEventList=0x02024c45
--define_symbol vTaskPlaceOnEventListRestricted=0x02024c7d
--define_symbol vTaskPriorityDisinheritAfterTimeout=0x02024cbd
--define_symbol vTaskPrioritySet=0x02024d69
--define_symbol vTaskResume=0x02024e31
--define_symbol vTaskStartScheduler=0x02024eb5
--define_symbol vTaskStepTick=0x02024f45
--define_symbol vTaskSuspend=0x02024f75
--define_symbol vTaskSuspendAll=0x02025031
--define_symbol vTaskSwitchContext=0x02025041
--define_symbol xPortStartScheduler=0x020250e9
--define_symbol xQueueAddToSet=0x020251ed
--define_symbol xQueueCreateCountingSemaphore=0x02025211
--define_symbol xQueueCreateCountingSemaphoreStatic=0x0202524d
--define_symbol xQueueCreateMutex=0x02025291
--define_symbol xQueueCreateMutexStatic=0x020252a7
--define_symbol xQueueCreateSet=0x020252c1
--define_symbol xQueueGenericCreate=0x020252c9
--define_symbol xQueueGenericCreateStatic=0x02025315
--define_symbol xQueueGenericReset=0x0202537d
--define_symbol xQueueGenericSend=0x02025409
--define_symbol xQueueGenericSendFromISR=0x02025575
--define_symbol xQueueGiveFromISR=0x02025635
--define_symbol xQueueGiveMutexRecursive=0x020256d9
--define_symbol xQueueIsQueueEmptyFromISR=0x02025719
--define_symbol xQueueIsQueueFullFromISR=0x0202573d
--define_symbol xQueuePeek=0x02025765
--define_symbol xQueuePeekFromISR=0x0202588d
--define_symbol xQueueReceive=0x020258f9
--define_symbol xQueueReceiveFromISR=0x02025a25
--define_symbol xQueueRemoveFromSet=0x02025ab9
--define_symbol xQueueSelectFromSet=0x02025adb
--define_symbol xQueueSelectFromSetFromISR=0x02025aed
--define_symbol xQueueSemaphoreTake=0x02025b01
--define_symbol xQueueTakeMutexRecursive=0x02025c6d
--define_symbol xTaskCheckForTimeOut=0x02025cb1
--define_symbol xTaskCreate=0x02025d21
--define_symbol xTaskCreateStatic=0x02025d7d
--define_symbol xTaskGetCurrentTaskHandle=0x02025ded
--define_symbol xTaskGetSchedulerState=0x02025df9
--define_symbol xTaskGetTickCount=0x02025e15
--define_symbol xTaskGetTickCountFromISR=0x02025e21
--define_symbol xTaskIncrementTick=0x02025e31
--define_symbol xTaskPriorityDisinherit=0x02025efd
--define_symbol xTaskPriorityInherit=0x02025f91
--define_symbol xTaskRemoveFromEventList=0x02026025
--define_symbol xTaskResumeAll=0x020260a5
--define_symbol xTaskResumeFromISR=0x0202616d
--define_symbol xTimerCreate=0x020261f9
--define_symbol xTimerCreateStatic=0x0202622d
--define_symbol xTimerCreateTimerTask=0x02026265
--define_symbol xTimerGenericCommand=0x020262d1
--define_symbol xTimerGetExpiryTime=0x02026341
--define_symbol xTimerGetTimerDaemonTaskHandle=0x02026361
