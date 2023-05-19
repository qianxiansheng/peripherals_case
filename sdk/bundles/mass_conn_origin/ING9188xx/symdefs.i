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
--define_symbol gap_add_dev_to_periodic_list=0x00007481
--define_symbol gap_add_whitelist=0x00007491
--define_symbol gap_aes_encrypt=0x0000749d
--define_symbol gap_clear_white_lists=0x000074e1
--define_symbol gap_clr_adv_set=0x000074ed
--define_symbol gap_clr_periodic_adv_list=0x000074f9
--define_symbol gap_create_connection_cancel=0x00007505
--define_symbol gap_default_periodic_adv_sync_transfer_param=0x00007511
--define_symbol gap_disconnect=0x00007529
--define_symbol gap_disconnect_all=0x00007555
--define_symbol gap_ext_create_connection=0x00007595
--define_symbol gap_get_connection_parameter_range=0x00007685
--define_symbol gap_le_read_channel_map=0x000076bd
--define_symbol gap_periodic_adv_create_sync=0x00007729
--define_symbol gap_periodic_adv_create_sync_cancel=0x0000774d
--define_symbol gap_periodic_adv_set_info_transfer=0x00007759
--define_symbol gap_periodic_adv_sync_transfer=0x00007769
--define_symbol gap_periodic_adv_sync_transfer_param=0x00007779
--define_symbol gap_periodic_adv_term_sync=0x00007795
--define_symbol gap_read_antenna_info=0x0000781d
--define_symbol gap_read_periodic_adv_list_size=0x00007829
--define_symbol gap_read_phy=0x00007835
--define_symbol gap_read_remote_used_features=0x00007841
--define_symbol gap_read_remote_version=0x0000784d
--define_symbol gap_read_rssi=0x00007859
--define_symbol gap_remove_whitelist=0x00007865
--define_symbol gap_rmv_adv_set=0x000078e1
--define_symbol gap_rmv_dev_from_periodic_list=0x000078ed
--define_symbol gap_rx_test_v2=0x000078fd
--define_symbol gap_rx_test_v3=0x0000790d
--define_symbol gap_set_adv_set_random_addr=0x00007959
--define_symbol gap_set_connection_cte_request_enable=0x000079a5
--define_symbol gap_set_connection_cte_response_enable=0x000079c1
--define_symbol gap_set_connection_cte_rx_param=0x000079d1
--define_symbol gap_set_connection_cte_tx_param=0x00007a2d
--define_symbol gap_set_connection_parameter_range=0x00007a81
--define_symbol gap_set_connectionless_cte_tx_enable=0x00007a9d
--define_symbol gap_set_connectionless_cte_tx_param=0x00007aad
--define_symbol gap_set_connectionless_iq_sampling_enable=0x00007b0d
--define_symbol gap_set_data_length=0x00007b71
--define_symbol gap_set_def_phy=0x00007b89
--define_symbol gap_set_ext_adv_data=0x00007b99
--define_symbol gap_set_ext_adv_enable=0x00007bb1
--define_symbol gap_set_ext_adv_para=0x00007c2d
--define_symbol gap_set_ext_scan_enable=0x00007d05
--define_symbol gap_set_ext_scan_para=0x00007d1d
--define_symbol gap_set_ext_scan_response_data=0x00007dc5
--define_symbol gap_set_host_channel_classification=0x00007ddd
--define_symbol gap_set_periodic_adv_data=0x00007ded
--define_symbol gap_set_periodic_adv_enable=0x00007e5d
--define_symbol gap_set_periodic_adv_para=0x00007e6d
--define_symbol gap_set_periodic_adv_rx_enable=0x00007e85
--define_symbol gap_set_phy=0x00007e95
--define_symbol gap_set_random_device_address=0x00007eb1
--define_symbol gap_start_ccm=0x00007ee1
--define_symbol gap_test_end=0x00007f35
--define_symbol gap_tx_test_v2=0x00007f41
--define_symbol gap_tx_test_v4=0x00007f59
--define_symbol gap_update_connection_parameters=0x00007f7d
--define_symbol gap_vendor_tx_continuous_wave=0x00007fc1
--define_symbol gatt_client_cancel_write=0x000084e9
--define_symbol gatt_client_discover_characteristic_descriptors=0x0000850f
--define_symbol gatt_client_discover_characteristics_for_handle_range_by_uuid128=0x0000854f
--define_symbol gatt_client_discover_characteristics_for_handle_range_by_uuid16=0x0000859f
--define_symbol gatt_client_discover_characteristics_for_service=0x000085ef
--define_symbol gatt_client_discover_primary_services=0x00008625
--define_symbol gatt_client_discover_primary_services_by_uuid128=0x00008657
--define_symbol gatt_client_discover_primary_services_by_uuid16=0x0000869b
--define_symbol gatt_client_execute_write=0x000086d7
--define_symbol gatt_client_find_included_services_for_service=0x000086fd
--define_symbol gatt_client_get_mtu=0x0000872b
--define_symbol gatt_client_is_ready=0x000087cd
--define_symbol gatt_client_listen_for_characteristic_value_updates=0x000087e3
--define_symbol gatt_client_prepare_write=0x00008805
--define_symbol gatt_client_read_characteristic_descriptor_using_descriptor_handle=0x00008841
--define_symbol gatt_client_read_long_characteristic_descriptor_using_descriptor_handle=0x0000886b
--define_symbol gatt_client_read_long_characteristic_descriptor_using_descriptor_handle_with_offset=0x00008871
--define_symbol gatt_client_read_long_value_of_characteristic_using_value_handle=0x0000889f
--define_symbol gatt_client_read_long_value_of_characteristic_using_value_handle_with_offset=0x000088a5
--define_symbol gatt_client_read_multiple_characteristic_values=0x000088d3
--define_symbol gatt_client_read_value_of_characteristic_using_value_handle=0x00008903
--define_symbol gatt_client_read_value_of_characteristics_by_uuid128=0x00008931
--define_symbol gatt_client_read_value_of_characteristics_by_uuid16=0x0000897d
--define_symbol gatt_client_register_handler=0x000089c9
--define_symbol gatt_client_reliable_write_long_value_of_characteristic=0x000089d5
--define_symbol gatt_client_signed_write_without_response=0x00008e05
--define_symbol gatt_client_write_characteristic_descriptor_using_descriptor_handle=0x00008ec9
--define_symbol gatt_client_write_client_characteristic_configuration=0x00008f03
--define_symbol gatt_client_write_long_characteristic_descriptor_using_descriptor_handle=0x00008f55
--define_symbol gatt_client_write_long_characteristic_descriptor_using_descriptor_handle_with_offset=0x00008f65
--define_symbol gatt_client_write_long_value_of_characteristic=0x00008fa1
--define_symbol gatt_client_write_long_value_of_characteristic_with_offset=0x00008fb1
--define_symbol gatt_client_write_value_of_characteristic=0x00008fed
--define_symbol gatt_client_write_value_of_characteristic_without_response=0x00009023
--define_symbol hci_add_event_handler=0x0000a565
--define_symbol hci_power_control=0x0000ad01
--define_symbol hci_register_acl_packet_handler=0x0000aeb5
--define_symbol kv_commit=0x0000b495
--define_symbol kv_get=0x0000b4ed
--define_symbol kv_init=0x0000b505
--define_symbol kv_put=0x0000b56d
--define_symbol kv_remove=0x0000b5e5
--define_symbol kv_remove_all=0x0000b621
--define_symbol kv_value_modified=0x0000b665
--define_symbol kv_visit=0x0000b669
--define_symbol l2cap_add_event_handler=0x0000b71d
--define_symbol l2cap_can_send_packet_now=0x0000b72d
--define_symbol l2cap_create_le_credit_based_connection_request=0x0000b8e9
--define_symbol l2cap_credit_based_send=0x0000ba2d
--define_symbol l2cap_credit_based_send_continue=0x0000ba59
--define_symbol l2cap_disconnect=0x0000bad5
--define_symbol l2cap_get_le_credit_based_connection_credits=0x0000bd25
--define_symbol l2cap_get_peer_mtu_for_local_cid=0x0000bd41
--define_symbol l2cap_init=0x0000c115
--define_symbol l2cap_le_send_flow_control_credit=0x0000c20b
--define_symbol l2cap_max_le_mtu=0x0000c515
--define_symbol l2cap_register_packet_handler=0x0000c63d
--define_symbol l2cap_register_service=0x0000c649
--define_symbol l2cap_request_can_send_now_event=0x0000c759
--define_symbol l2cap_request_connection_parameter_update=0x0000c773
--define_symbol l2cap_send_echo_request=0x0000cc45
--define_symbol l2cap_unregister_service=0x0000cd05
--define_symbol le_device_db_add=0x0000cd5d
--define_symbol le_device_db_find=0x0000ce31
--define_symbol le_device_db_from_key=0x0000ce5d
--define_symbol le_device_db_iter_cur=0x0000ce65
--define_symbol le_device_db_iter_cur_key=0x0000ce69
--define_symbol le_device_db_iter_init=0x0000ce6d
--define_symbol le_device_db_iter_next=0x0000ce75
--define_symbol le_device_db_remove_key=0x0000ce9b
--define_symbol ll_aes_encrypt=0x0000cec9
--define_symbol ll_free=0x0000cf45
--define_symbol ll_hint_on_ce_len=0x0000cf51
--define_symbol ll_legacy_adv_set_interval=0x0000cf89
--define_symbol ll_malloc=0x0000cf99
--define_symbol ll_query_timing_info=0x0000d0d1
--define_symbol ll_scan_set_fixed_channel=0x0000d175
--define_symbol ll_set_adv_access_address=0x0000d289
--define_symbol ll_set_adv_coded_scheme=0x0000d295
--define_symbol ll_set_conn_coded_scheme=0x0000d2c5
--define_symbol ll_set_conn_latency=0x0000d2f1
--define_symbol ll_set_conn_tx_power=0x0000d321
--define_symbol ll_set_def_antenna=0x0000d369
--define_symbol ll_set_initiating_coded_scheme=0x0000d385
--define_symbol ll_set_max_conn_number=0x0000d391
--define_symbol nibble_for_char=0x0001e1cd
--define_symbol platform_32k_rc_auto_tune=0x0001e279
--define_symbol platform_32k_rc_tune=0x0001e2f5
--define_symbol platform_calibrate_32k=0x0001e309
--define_symbol platform_config=0x0001e30d
--define_symbol platform_enable_irq=0x0001e435
--define_symbol platform_get_current_task=0x0001e46d
--define_symbol platform_get_gen_os_driver=0x0001e491
--define_symbol platform_get_heap_status=0x0001e499
--define_symbol platform_get_task_handle=0x0001e4b1
--define_symbol platform_get_us_time=0x0001e4d1
--define_symbol platform_get_version=0x0001e4d5
--define_symbol platform_hrng=0x0001e4dd
--define_symbol platform_install_isr_stack=0x0001e4e5
--define_symbol platform_install_task_stack=0x0001e4f1
--define_symbol platform_patch_rf_init_data=0x0001e529
--define_symbol platform_printf=0x0001e535
--define_symbol platform_raise_assertion=0x0001e549
--define_symbol platform_rand=0x0001e55d
--define_symbol platform_read_info=0x0001e561
--define_symbol platform_read_persistent_reg=0x0001e591
--define_symbol platform_reset=0x0001e5a1
--define_symbol platform_set_evt_callback=0x0001e5c5
--define_symbol platform_set_evt_callback_table=0x0001e5d9
--define_symbol platform_set_irq_callback=0x0001e5e5
--define_symbol platform_set_irq_callback_table=0x0001e601
--define_symbol platform_set_rf_clk_source=0x0001e60d
--define_symbol platform_set_rf_init_data=0x0001e619
--define_symbol platform_set_rf_power_mapping=0x0001e625
--define_symbol platform_set_timer=0x0001e631
--define_symbol platform_shutdown=0x0001e635
--define_symbol platform_switch_app=0x0001e639
--define_symbol platform_trace_raw=0x0001e665
--define_symbol platform_write_persistent_reg=0x0001e67d
--define_symbol printf_hexdump=0x0001e831
--define_symbol pvPortMalloc=0x0001f321
--define_symbol pvTaskIncrementMutexHeldCount=0x0001f409
--define_symbol pvTimerGetTimerID=0x0001f421
--define_symbol pxPortInitialiseStack=0x0001f44d
--define_symbol reverse_128=0x0001f62d
--define_symbol reverse_24=0x0001f633
--define_symbol reverse_48=0x0001f639
--define_symbol reverse_56=0x0001f63f
--define_symbol reverse_64=0x0001f645
--define_symbol reverse_bd_addr=0x0001f64b
--define_symbol reverse_bytes=0x0001f651
--define_symbol sm_add_event_handler=0x0001f911
--define_symbol sm_address_resolution_lookup=0x0001fa69
--define_symbol sm_authenticated=0x0001fdc9
--define_symbol sm_authorization_decline=0x0001fdd7
--define_symbol sm_authorization_grant=0x0001fdf7
--define_symbol sm_authorization_state=0x0001fe17
--define_symbol sm_bonding_decline=0x0001fe31
--define_symbol sm_config=0x00020251
--define_symbol sm_config_conn=0x00020269
--define_symbol sm_encryption_key_size=0x0002041f
--define_symbol sm_just_works_confirm=0x00020959
--define_symbol sm_le_device_key=0x00020c95
--define_symbol sm_passkey_input=0x00020d2b
--define_symbol sm_private_random_address_generation_get=0x000210d9
--define_symbol sm_private_random_address_generation_get_mode=0x000210e1
--define_symbol sm_private_random_address_generation_set_mode=0x000210ed
--define_symbol sm_private_random_address_generation_set_update_period=0x00021115
--define_symbol sm_register_oob_data_callback=0x00021251
--define_symbol sm_request_pairing=0x0002125d
--define_symbol sm_send_security_request=0x00021c97
--define_symbol sm_set_accepted_stk_generation_methods=0x00021cbd
--define_symbol sm_set_authentication_requirements=0x00021cc9
--define_symbol sm_set_encryption_key_size_range=0x00021cd5
--define_symbol sscanf_bd_addr=0x00022031
--define_symbol sysSetPublicDeviceAddr=0x000223e5
--define_symbol uuid128_to_str=0x00022b59
--define_symbol uuid_add_bluetooth_prefix=0x00022bb1
--define_symbol uuid_has_bluetooth_prefix=0x00022bd1
--define_symbol uxListRemove=0x00022bed
--define_symbol uxQueueMessagesWaiting=0x00022c15
--define_symbol uxQueueMessagesWaitingFromISR=0x00022c3d
--define_symbol uxQueueSpacesAvailable=0x00022c59
--define_symbol uxTaskGetStackHighWaterMark=0x00022c85
--define_symbol uxTaskPriorityGet=0x00022ca5
--define_symbol uxTaskPriorityGetFromISR=0x00022cc1
--define_symbol vListInitialise=0x00022d7b
--define_symbol vListInitialiseItem=0x00022d91
--define_symbol vListInsert=0x00022d97
--define_symbol vListInsertEnd=0x00022dc7
--define_symbol vPortEndScheduler=0x00022de1
--define_symbol vPortEnterCritical=0x00022e0d
--define_symbol vPortExitCritical=0x00022e51
--define_symbol vPortFree=0x00022e85
--define_symbol vPortSuppressTicksAndSleep=0x00022f19
--define_symbol vPortValidateInterruptPriority=0x00023035
--define_symbol vQueueDelete=0x00023091
--define_symbol vQueueWaitForMessageRestricted=0x000230bd
--define_symbol vTaskDelay=0x00023105
--define_symbol vTaskInternalSetTimeOutState=0x00023151
--define_symbol vTaskMissedYield=0x00023161
--define_symbol vTaskPlaceOnEventList=0x0002316d
--define_symbol vTaskPlaceOnEventListRestricted=0x000231a5
--define_symbol vTaskPriorityDisinheritAfterTimeout=0x000231e5
--define_symbol vTaskPrioritySet=0x00023291
--define_symbol vTaskResume=0x00023359
--define_symbol vTaskStartScheduler=0x000233dd
--define_symbol vTaskStepTick=0x0002346d
--define_symbol vTaskSuspend=0x0002349d
--define_symbol vTaskSuspendAll=0x00023559
--define_symbol vTaskSwitchContext=0x00023569
--define_symbol xPortStartScheduler=0x00023611
--define_symbol xQueueAddToSet=0x000236d9
--define_symbol xQueueCreateCountingSemaphore=0x000236fd
--define_symbol xQueueCreateCountingSemaphoreStatic=0x00023739
--define_symbol xQueueCreateMutex=0x0002377d
--define_symbol xQueueCreateMutexStatic=0x00023793
--define_symbol xQueueCreateSet=0x000237ad
--define_symbol xQueueGenericCreate=0x000237b5
--define_symbol xQueueGenericCreateStatic=0x00023801
--define_symbol xQueueGenericReset=0x00023869
--define_symbol xQueueGenericSend=0x000238f5
--define_symbol xQueueGenericSendFromISR=0x00023a61
--define_symbol xQueueGiveFromISR=0x00023b21
--define_symbol xQueueGiveMutexRecursive=0x00023bc5
--define_symbol xQueueIsQueueEmptyFromISR=0x00023c05
--define_symbol xQueueIsQueueFullFromISR=0x00023c29
--define_symbol xQueuePeek=0x00023c51
--define_symbol xQueuePeekFromISR=0x00023d79
--define_symbol xQueueReceive=0x00023de5
--define_symbol xQueueReceiveFromISR=0x00023f11
--define_symbol xQueueRemoveFromSet=0x00023fa5
--define_symbol xQueueSelectFromSet=0x00023fc7
--define_symbol xQueueSelectFromSetFromISR=0x00023fd9
--define_symbol xQueueSemaphoreTake=0x00023fed
--define_symbol xQueueTakeMutexRecursive=0x00024159
--define_symbol xTaskCheckForTimeOut=0x0002419d
--define_symbol xTaskCreate=0x0002420d
--define_symbol xTaskCreateStatic=0x00024269
--define_symbol xTaskGetCurrentTaskHandle=0x000242d9
--define_symbol xTaskGetSchedulerState=0x000242e5
--define_symbol xTaskGetTickCount=0x00024301
--define_symbol xTaskGetTickCountFromISR=0x0002430d
--define_symbol xTaskIncrementTick=0x0002431d
--define_symbol xTaskPriorityDisinherit=0x000243e9
--define_symbol xTaskPriorityInherit=0x0002447d
--define_symbol xTaskRemoveFromEventList=0x00024511
--define_symbol xTaskResumeAll=0x00024591
--define_symbol xTaskResumeFromISR=0x00024659
--define_symbol xTimerCreate=0x000246e5
--define_symbol xTimerCreateStatic=0x00024719
--define_symbol xTimerCreateTimerTask=0x00024751
--define_symbol xTimerGenericCommand=0x000247bd
--define_symbol xTimerGetExpiryTime=0x0002482d
--define_symbol xTimerGetTimerDaemonTaskHandle=0x0002484d
