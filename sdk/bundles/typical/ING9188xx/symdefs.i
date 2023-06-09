--define_symbol att_dispatch_client_can_send_now=0x00005875
--define_symbol att_dispatch_client_request_can_send_now_event=0x0000587b
--define_symbol att_dispatch_register_client=0x00005881
--define_symbol att_dispatch_register_server=0x00005895
--define_symbol att_dispatch_server_can_send_now=0x000058a9
--define_symbol att_dispatch_server_request_can_send_now_event=0x000058af
--define_symbol att_emit_general_event=0x00005961
--define_symbol att_server_can_send_packet_now=0x00006075
--define_symbol att_server_deferred_read_response=0x00006079
--define_symbol att_server_get_mtu=0x00006091
--define_symbol att_server_indicate=0x00006109
--define_symbol att_server_init=0x0000618d
--define_symbol att_server_notify=0x000061c9
--define_symbol att_server_register_packet_handler=0x000062e1
--define_symbol att_server_request_can_send_now_event=0x000062ed
--define_symbol att_set_db=0x00006309
--define_symbol att_set_read_callback=0x0000631d
--define_symbol att_set_write_callback=0x00006329
--define_symbol bd_addr_cmp=0x00006499
--define_symbol bd_addr_copy=0x0000649f
--define_symbol bd_addr_to_str=0x000064a9
--define_symbol big_endian_read_16=0x000064e1
--define_symbol big_endian_read_32=0x000064e9
--define_symbol big_endian_store_16=0x000064fd
--define_symbol big_endian_store_32=0x00006509
--define_symbol btstack_config=0x0000665d
--define_symbol btstack_memory_pool_create=0x0000679b
--define_symbol btstack_memory_pool_free=0x000067c5
--define_symbol btstack_memory_pool_get=0x00006825
--define_symbol btstack_push_user_msg=0x0000688d
--define_symbol btstack_push_user_runnable=0x00006899
--define_symbol char_for_nibble=0x00006b21
--define_symbol eTaskConfirmSleepModeStatus=0x00006ded
--define_symbol gap_add_dev_to_periodic_list=0x00007489
--define_symbol gap_add_whitelist=0x00007499
--define_symbol gap_aes_encrypt=0x000074a5
--define_symbol gap_clear_white_lists=0x000074e9
--define_symbol gap_clr_adv_set=0x000074f5
--define_symbol gap_clr_periodic_adv_list=0x00007501
--define_symbol gap_create_connection_cancel=0x0000750d
--define_symbol gap_default_periodic_adv_sync_transfer_param=0x00007519
--define_symbol gap_disconnect=0x00007531
--define_symbol gap_disconnect_all=0x0000755d
--define_symbol gap_ext_create_connection=0x0000759d
--define_symbol gap_get_connection_parameter_range=0x0000768d
--define_symbol gap_le_read_channel_map=0x000076c9
--define_symbol gap_periodic_adv_create_sync=0x00007735
--define_symbol gap_periodic_adv_create_sync_cancel=0x00007759
--define_symbol gap_periodic_adv_set_info_transfer=0x00007765
--define_symbol gap_periodic_adv_sync_transfer=0x00007775
--define_symbol gap_periodic_adv_sync_transfer_param=0x00007785
--define_symbol gap_periodic_adv_term_sync=0x000077a1
--define_symbol gap_read_antenna_info=0x00007829
--define_symbol gap_read_periodic_adv_list_size=0x00007835
--define_symbol gap_read_phy=0x00007841
--define_symbol gap_read_remote_used_features=0x0000784d
--define_symbol gap_read_remote_version=0x00007859
--define_symbol gap_read_rssi=0x00007865
--define_symbol gap_remove_whitelist=0x00007871
--define_symbol gap_rmv_adv_set=0x000078ed
--define_symbol gap_rmv_dev_from_periodic_list=0x000078f9
--define_symbol gap_rx_test_v2=0x00007909
--define_symbol gap_rx_test_v3=0x00007919
--define_symbol gap_set_adv_set_random_addr=0x00007965
--define_symbol gap_set_connection_cte_request_enable=0x000079ad
--define_symbol gap_set_connection_cte_response_enable=0x000079c9
--define_symbol gap_set_connection_cte_rx_param=0x000079d9
--define_symbol gap_set_connection_cte_tx_param=0x00007a35
--define_symbol gap_set_connection_parameter_range=0x00007a89
--define_symbol gap_set_connectionless_cte_tx_enable=0x00007aa1
--define_symbol gap_set_connectionless_cte_tx_param=0x00007ab1
--define_symbol gap_set_connectionless_iq_sampling_enable=0x00007b11
--define_symbol gap_set_data_length=0x00007b75
--define_symbol gap_set_def_phy=0x00007b8d
--define_symbol gap_set_ext_adv_data=0x00007b9d
--define_symbol gap_set_ext_adv_enable=0x00007bb5
--define_symbol gap_set_ext_adv_para=0x00007c31
--define_symbol gap_set_ext_scan_enable=0x00007d09
--define_symbol gap_set_ext_scan_para=0x00007d21
--define_symbol gap_set_ext_scan_response_data=0x00007dc9
--define_symbol gap_set_host_channel_classification=0x00007de1
--define_symbol gap_set_periodic_adv_data=0x00007df1
--define_symbol gap_set_periodic_adv_enable=0x00007e61
--define_symbol gap_set_periodic_adv_para=0x00007e71
--define_symbol gap_set_periodic_adv_rx_enable=0x00007e89
--define_symbol gap_set_phy=0x00007e99
--define_symbol gap_set_random_device_address=0x00007eb5
--define_symbol gap_start_ccm=0x00007ee5
--define_symbol gap_test_end=0x00007f39
--define_symbol gap_tx_test_v2=0x00007f45
--define_symbol gap_tx_test_v4=0x00007f5d
--define_symbol gap_update_connection_parameters=0x00007f81
--define_symbol gap_vendor_tx_continuous_wave=0x00007fc5
--define_symbol gatt_client_cancel_write=0x000084ed
--define_symbol gatt_client_discover_characteristic_descriptors=0x00008513
--define_symbol gatt_client_discover_characteristics_for_handle_range_by_uuid128=0x00008553
--define_symbol gatt_client_discover_characteristics_for_handle_range_by_uuid16=0x000085a3
--define_symbol gatt_client_discover_characteristics_for_service=0x000085f3
--define_symbol gatt_client_discover_primary_services=0x00008629
--define_symbol gatt_client_discover_primary_services_by_uuid128=0x0000865b
--define_symbol gatt_client_discover_primary_services_by_uuid16=0x0000869f
--define_symbol gatt_client_execute_write=0x000086db
--define_symbol gatt_client_find_included_services_for_service=0x00008701
--define_symbol gatt_client_get_mtu=0x0000872f
--define_symbol gatt_client_is_ready=0x000087d1
--define_symbol gatt_client_listen_for_characteristic_value_updates=0x000087e7
--define_symbol gatt_client_prepare_write=0x00008809
--define_symbol gatt_client_read_characteristic_descriptor_using_descriptor_handle=0x00008845
--define_symbol gatt_client_read_long_characteristic_descriptor_using_descriptor_handle=0x0000886f
--define_symbol gatt_client_read_long_characteristic_descriptor_using_descriptor_handle_with_offset=0x00008875
--define_symbol gatt_client_read_long_value_of_characteristic_using_value_handle=0x000088a3
--define_symbol gatt_client_read_long_value_of_characteristic_using_value_handle_with_offset=0x000088a9
--define_symbol gatt_client_read_multiple_characteristic_values=0x000088d7
--define_symbol gatt_client_read_value_of_characteristic_using_value_handle=0x00008907
--define_symbol gatt_client_read_value_of_characteristics_by_uuid128=0x00008935
--define_symbol gatt_client_read_value_of_characteristics_by_uuid16=0x00008981
--define_symbol gatt_client_register_handler=0x000089cd
--define_symbol gatt_client_reliable_write_long_value_of_characteristic=0x000089d9
--define_symbol gatt_client_signed_write_without_response=0x00008e09
--define_symbol gatt_client_write_characteristic_descriptor_using_descriptor_handle=0x00008ecd
--define_symbol gatt_client_write_client_characteristic_configuration=0x00008f07
--define_symbol gatt_client_write_long_characteristic_descriptor_using_descriptor_handle=0x00008f59
--define_symbol gatt_client_write_long_characteristic_descriptor_using_descriptor_handle_with_offset=0x00008f69
--define_symbol gatt_client_write_long_value_of_characteristic=0x00008fa5
--define_symbol gatt_client_write_long_value_of_characteristic_with_offset=0x00008fb5
--define_symbol gatt_client_write_value_of_characteristic=0x00008ff1
--define_symbol gatt_client_write_value_of_characteristic_without_response=0x00009027
--define_symbol hci_add_event_handler=0x0000a569
--define_symbol hci_power_control=0x0000ad5d
--define_symbol hci_register_acl_packet_handler=0x0000af11
--define_symbol kv_commit=0x0000b485
--define_symbol kv_get=0x0000b4dd
--define_symbol kv_init=0x0000b4f5
--define_symbol kv_put=0x0000b55d
--define_symbol kv_remove=0x0000b5d5
--define_symbol kv_remove_all=0x0000b611
--define_symbol kv_value_modified=0x0000b655
--define_symbol kv_visit=0x0000b659
--define_symbol l2cap_add_event_handler=0x0000b70d
--define_symbol l2cap_can_send_packet_now=0x0000b71d
--define_symbol l2cap_create_le_credit_based_connection_request=0x0000b8d9
--define_symbol l2cap_credit_based_send=0x0000ba1d
--define_symbol l2cap_credit_based_send_continue=0x0000ba49
--define_symbol l2cap_disconnect=0x0000bac5
--define_symbol l2cap_get_le_credit_based_connection_credits=0x0000bd15
--define_symbol l2cap_get_peer_mtu_for_local_cid=0x0000bd31
--define_symbol l2cap_init=0x0000c105
--define_symbol l2cap_le_send_flow_control_credit=0x0000c1fb
--define_symbol l2cap_max_le_mtu=0x0000c505
--define_symbol l2cap_register_packet_handler=0x0000c62d
--define_symbol l2cap_register_service=0x0000c639
--define_symbol l2cap_request_can_send_now_event=0x0000c749
--define_symbol l2cap_request_connection_parameter_update=0x0000c763
--define_symbol l2cap_send_echo_request=0x0000cc35
--define_symbol l2cap_unregister_service=0x0000ccf5
--define_symbol le_device_db_add=0x0000cd4d
--define_symbol le_device_db_find=0x0000ce21
--define_symbol le_device_db_from_key=0x0000ce4d
--define_symbol le_device_db_iter_cur=0x0000ce55
--define_symbol le_device_db_iter_cur_key=0x0000ce59
--define_symbol le_device_db_iter_init=0x0000ce5d
--define_symbol le_device_db_iter_next=0x0000ce65
--define_symbol le_device_db_remove_key=0x0000ce8b
--define_symbol ll_aes_encrypt=0x0000ceb9
--define_symbol ll_free=0x0000cf35
--define_symbol ll_hint_on_ce_len=0x0000cf41
--define_symbol ll_legacy_adv_set_interval=0x0000cf79
--define_symbol ll_malloc=0x0000cf89
--define_symbol ll_query_timing_info=0x0000d0c1
--define_symbol ll_scan_set_fixed_channel=0x0000d165
--define_symbol ll_set_adv_access_address=0x0000d279
--define_symbol ll_set_adv_coded_scheme=0x0000d285
--define_symbol ll_set_conn_coded_scheme=0x0000d2b5
--define_symbol ll_set_conn_latency=0x0000d2e1
--define_symbol ll_set_conn_tx_power=0x0000d311
--define_symbol ll_set_def_antenna=0x0000d359
--define_symbol ll_set_initiating_coded_scheme=0x0000d375
--define_symbol ll_set_max_conn_number=0x0000d381
--define_symbol nibble_for_char=0x0001e195
--define_symbol platform_32k_rc_auto_tune=0x0001e241
--define_symbol platform_32k_rc_tune=0x0001e2bd
--define_symbol platform_calibrate_32k=0x0001e2d1
--define_symbol platform_config=0x0001e2d5
--define_symbol platform_enable_irq=0x0001e3fd
--define_symbol platform_get_current_task=0x0001e435
--define_symbol platform_get_gen_os_driver=0x0001e459
--define_symbol platform_get_heap_status=0x0001e461
--define_symbol platform_get_task_handle=0x0001e479
--define_symbol platform_get_us_time=0x0001e499
--define_symbol platform_get_version=0x0001e49d
--define_symbol platform_hrng=0x0001e4a5
--define_symbol platform_install_isr_stack=0x0001e4ad
--define_symbol platform_install_task_stack=0x0001e4b9
--define_symbol platform_patch_rf_init_data=0x0001e4f1
--define_symbol platform_printf=0x0001e4fd
--define_symbol platform_raise_assertion=0x0001e511
--define_symbol platform_rand=0x0001e525
--define_symbol platform_read_info=0x0001e529
--define_symbol platform_read_persistent_reg=0x0001e559
--define_symbol platform_reset=0x0001e569
--define_symbol platform_set_evt_callback=0x0001e58d
--define_symbol platform_set_evt_callback_table=0x0001e5a1
--define_symbol platform_set_irq_callback=0x0001e5ad
--define_symbol platform_set_irq_callback_table=0x0001e5c9
--define_symbol platform_set_rf_clk_source=0x0001e5d5
--define_symbol platform_set_rf_init_data=0x0001e5e1
--define_symbol platform_set_rf_power_mapping=0x0001e5ed
--define_symbol platform_set_timer=0x0001e5f9
--define_symbol platform_shutdown=0x0001e5fd
--define_symbol platform_switch_app=0x0001e601
--define_symbol platform_trace_raw=0x0001e62d
--define_symbol platform_write_persistent_reg=0x0001e645
--define_symbol printf_hexdump=0x0001e7f9
--define_symbol pvPortMalloc=0x0001f2e9
--define_symbol pvTaskIncrementMutexHeldCount=0x0001f3d1
--define_symbol pvTimerGetTimerID=0x0001f3e9
--define_symbol pxPortInitialiseStack=0x0001f415
--define_symbol reverse_128=0x0001f5f5
--define_symbol reverse_24=0x0001f5fb
--define_symbol reverse_48=0x0001f601
--define_symbol reverse_56=0x0001f607
--define_symbol reverse_64=0x0001f60d
--define_symbol reverse_bd_addr=0x0001f613
--define_symbol reverse_bytes=0x0001f619
--define_symbol sm_add_event_handler=0x0001f8d9
--define_symbol sm_address_resolution_lookup=0x0001fa31
--define_symbol sm_authenticated=0x0001fd91
--define_symbol sm_authorization_decline=0x0001fd9f
--define_symbol sm_authorization_grant=0x0001fdbf
--define_symbol sm_authorization_state=0x0001fddf
--define_symbol sm_bonding_decline=0x0001fdf9
--define_symbol sm_config=0x00020219
--define_symbol sm_config_conn=0x00020231
--define_symbol sm_encryption_key_size=0x000203e7
--define_symbol sm_just_works_confirm=0x00020921
--define_symbol sm_le_device_key=0x00020c5d
--define_symbol sm_passkey_input=0x00020cf3
--define_symbol sm_private_random_address_generation_get=0x000210a1
--define_symbol sm_private_random_address_generation_get_mode=0x000210a9
--define_symbol sm_private_random_address_generation_set_mode=0x000210b5
--define_symbol sm_private_random_address_generation_set_update_period=0x000210dd
--define_symbol sm_register_oob_data_callback=0x00021219
--define_symbol sm_request_pairing=0x00021225
--define_symbol sm_send_security_request=0x00021c5f
--define_symbol sm_set_accepted_stk_generation_methods=0x00021c85
--define_symbol sm_set_authentication_requirements=0x00021c91
--define_symbol sm_set_encryption_key_size_range=0x00021c9d
--define_symbol sscanf_bd_addr=0x00021ff9
--define_symbol sysSetPublicDeviceAddr=0x000223ad
--define_symbol uuid128_to_str=0x00022b21
--define_symbol uuid_add_bluetooth_prefix=0x00022b79
--define_symbol uuid_has_bluetooth_prefix=0x00022b99
--define_symbol uxListRemove=0x00022bb5
--define_symbol uxQueueMessagesWaiting=0x00022bdd
--define_symbol uxQueueMessagesWaitingFromISR=0x00022c05
--define_symbol uxQueueSpacesAvailable=0x00022c21
--define_symbol uxTaskGetStackHighWaterMark=0x00022c4d
--define_symbol uxTaskPriorityGet=0x00022c6d
--define_symbol uxTaskPriorityGetFromISR=0x00022c89
--define_symbol vListInitialise=0x00022d43
--define_symbol vListInitialiseItem=0x00022d59
--define_symbol vListInsert=0x00022d5f
--define_symbol vListInsertEnd=0x00022d8f
--define_symbol vPortEndScheduler=0x00022da9
--define_symbol vPortEnterCritical=0x00022dd5
--define_symbol vPortExitCritical=0x00022e19
--define_symbol vPortFree=0x00022e4d
--define_symbol vPortSuppressTicksAndSleep=0x00022ee1
--define_symbol vPortValidateInterruptPriority=0x00022ffd
--define_symbol vQueueDelete=0x00023059
--define_symbol vQueueWaitForMessageRestricted=0x00023085
--define_symbol vTaskDelay=0x000230cd
--define_symbol vTaskInternalSetTimeOutState=0x00023119
--define_symbol vTaskMissedYield=0x00023129
--define_symbol vTaskPlaceOnEventList=0x00023135
--define_symbol vTaskPlaceOnEventListRestricted=0x0002316d
--define_symbol vTaskPriorityDisinheritAfterTimeout=0x000231ad
--define_symbol vTaskPrioritySet=0x00023259
--define_symbol vTaskResume=0x00023321
--define_symbol vTaskStartScheduler=0x000233a5
--define_symbol vTaskStepTick=0x00023435
--define_symbol vTaskSuspend=0x00023465
--define_symbol vTaskSuspendAll=0x00023521
--define_symbol vTaskSwitchContext=0x00023531
--define_symbol xPortStartScheduler=0x000235d9
--define_symbol xQueueAddToSet=0x000236a1
--define_symbol xQueueCreateCountingSemaphore=0x000236c5
--define_symbol xQueueCreateCountingSemaphoreStatic=0x00023701
--define_symbol xQueueCreateMutex=0x00023745
--define_symbol xQueueCreateMutexStatic=0x0002375b
--define_symbol xQueueCreateSet=0x00023775
--define_symbol xQueueGenericCreate=0x0002377d
--define_symbol xQueueGenericCreateStatic=0x000237c9
--define_symbol xQueueGenericReset=0x00023831
--define_symbol xQueueGenericSend=0x000238bd
--define_symbol xQueueGenericSendFromISR=0x00023a29
--define_symbol xQueueGiveFromISR=0x00023ae9
--define_symbol xQueueGiveMutexRecursive=0x00023b8d
--define_symbol xQueueIsQueueEmptyFromISR=0x00023bcd
--define_symbol xQueueIsQueueFullFromISR=0x00023bf1
--define_symbol xQueuePeek=0x00023c19
--define_symbol xQueuePeekFromISR=0x00023d41
--define_symbol xQueueReceive=0x00023dad
--define_symbol xQueueReceiveFromISR=0x00023ed9
--define_symbol xQueueRemoveFromSet=0x00023f6d
--define_symbol xQueueSelectFromSet=0x00023f8f
--define_symbol xQueueSelectFromSetFromISR=0x00023fa1
--define_symbol xQueueSemaphoreTake=0x00023fb5
--define_symbol xQueueTakeMutexRecursive=0x00024121
--define_symbol xTaskCheckForTimeOut=0x00024165
--define_symbol xTaskCreate=0x000241d5
--define_symbol xTaskCreateStatic=0x00024231
--define_symbol xTaskGetCurrentTaskHandle=0x000242a1
--define_symbol xTaskGetSchedulerState=0x000242ad
--define_symbol xTaskGetTickCount=0x000242c9
--define_symbol xTaskGetTickCountFromISR=0x000242d5
--define_symbol xTaskIncrementTick=0x000242e5
--define_symbol xTaskPriorityDisinherit=0x000243b1
--define_symbol xTaskPriorityInherit=0x00024445
--define_symbol xTaskRemoveFromEventList=0x000244d9
--define_symbol xTaskResumeAll=0x00024559
--define_symbol xTaskResumeFromISR=0x00024621
--define_symbol xTimerCreate=0x000246ad
--define_symbol xTimerCreateStatic=0x000246e1
--define_symbol xTimerCreateTimerTask=0x00024719
--define_symbol xTimerGenericCommand=0x00024785
--define_symbol xTimerGetExpiryTime=0x000247f5
--define_symbol xTimerGetTimerDaemonTaskHandle=0x00024815
