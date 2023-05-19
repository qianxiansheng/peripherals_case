--define_symbol att_dispatch_client_can_send_now=0x0000565d
--define_symbol att_dispatch_client_request_can_send_now_event=0x00005663
--define_symbol att_dispatch_register_client=0x00005669
--define_symbol att_dispatch_register_server=0x0000567d
--define_symbol att_dispatch_server_can_send_now=0x00005691
--define_symbol att_dispatch_server_request_can_send_now_event=0x00005697
--define_symbol att_emit_general_event=0x00005749
--define_symbol att_server_can_send_packet_now=0x00005e5d
--define_symbol att_server_deferred_read_response=0x00005e61
--define_symbol att_server_get_mtu=0x00005e79
--define_symbol att_server_indicate=0x00005ef1
--define_symbol att_server_init=0x00005f75
--define_symbol att_server_notify=0x00005fb1
--define_symbol att_server_register_packet_handler=0x000060c9
--define_symbol att_server_request_can_send_now_event=0x000060d5
--define_symbol att_set_db=0x000060f1
--define_symbol att_set_read_callback=0x00006105
--define_symbol att_set_write_callback=0x00006111
--define_symbol bd_addr_cmp=0x00006281
--define_symbol bd_addr_copy=0x00006287
--define_symbol bd_addr_to_str=0x00006291
--define_symbol big_endian_read_16=0x000062c9
--define_symbol big_endian_read_32=0x000062d1
--define_symbol big_endian_store_16=0x000062e5
--define_symbol big_endian_store_32=0x000062f1
--define_symbol btstack_config=0x00006429
--define_symbol btstack_memory_pool_create=0x00006577
--define_symbol btstack_memory_pool_free=0x000065a1
--define_symbol btstack_memory_pool_get=0x00006601
--define_symbol btstack_push_user_msg=0x00006649
--define_symbol btstack_push_user_runnable=0x00006655
--define_symbol char_for_nibble=0x000068c5
--define_symbol gap_add_dev_to_periodic_list=0x00007185
--define_symbol gap_add_whitelist=0x00007195
--define_symbol gap_aes_encrypt=0x000071a1
--define_symbol gap_clear_white_lists=0x000071e5
--define_symbol gap_clr_adv_set=0x000071f1
--define_symbol gap_clr_periodic_adv_list=0x000071fd
--define_symbol gap_create_connection_cancel=0x00007209
--define_symbol gap_disconnect=0x00007215
--define_symbol gap_disconnect_all=0x00007241
--define_symbol gap_ext_create_connection=0x00007281
--define_symbol gap_get_connection_parameter_range=0x00007371
--define_symbol gap_le_read_channel_map=0x000073ad
--define_symbol gap_periodic_adv_create_sync=0x00007419
--define_symbol gap_periodic_adv_create_sync_cancel=0x0000743d
--define_symbol gap_periodic_adv_term_sync=0x00007449
--define_symbol gap_read_periodic_adv_list_size=0x000074d1
--define_symbol gap_read_phy=0x000074dd
--define_symbol gap_read_remote_used_features=0x000074e9
--define_symbol gap_read_remote_version=0x000074f5
--define_symbol gap_read_rssi=0x00007501
--define_symbol gap_remove_whitelist=0x0000750d
--define_symbol gap_rmv_adv_set=0x00007589
--define_symbol gap_rmv_dev_from_periodic_list=0x00007595
--define_symbol gap_rx_test_v2=0x000075a5
--define_symbol gap_set_adv_set_random_addr=0x000075dd
--define_symbol gap_set_connection_parameter_range=0x00007625
--define_symbol gap_set_data_length=0x0000763d
--define_symbol gap_set_def_phy=0x00007655
--define_symbol gap_set_ext_adv_data=0x00007665
--define_symbol gap_set_ext_adv_enable=0x0000767d
--define_symbol gap_set_ext_adv_para=0x000076f9
--define_symbol gap_set_ext_scan_enable=0x000077d1
--define_symbol gap_set_ext_scan_para=0x000077e9
--define_symbol gap_set_ext_scan_response_data=0x00007891
--define_symbol gap_set_host_channel_classification=0x000078a9
--define_symbol gap_set_periodic_adv_data=0x000078b9
--define_symbol gap_set_periodic_adv_enable=0x00007929
--define_symbol gap_set_periodic_adv_para=0x00007939
--define_symbol gap_set_phy=0x00007951
--define_symbol gap_set_random_device_address=0x0000796d
--define_symbol gap_start_ccm=0x0000799d
--define_symbol gap_test_end=0x000079f1
--define_symbol gap_tx_test_v2=0x000079fd
--define_symbol gap_tx_test_v4=0x00007a15
--define_symbol gap_update_connection_parameters=0x00007a39
--define_symbol gap_vendor_tx_continuous_wave=0x00007a7d
--define_symbol gatt_client_cancel_write=0x00007fa5
--define_symbol gatt_client_discover_characteristic_descriptors=0x00007fcb
--define_symbol gatt_client_discover_characteristics_for_handle_range_by_uuid128=0x0000800b
--define_symbol gatt_client_discover_characteristics_for_handle_range_by_uuid16=0x0000805b
--define_symbol gatt_client_discover_characteristics_for_service=0x000080ab
--define_symbol gatt_client_discover_primary_services=0x000080e1
--define_symbol gatt_client_discover_primary_services_by_uuid128=0x00008113
--define_symbol gatt_client_discover_primary_services_by_uuid16=0x00008157
--define_symbol gatt_client_execute_write=0x00008193
--define_symbol gatt_client_find_included_services_for_service=0x000081b9
--define_symbol gatt_client_get_mtu=0x000081e7
--define_symbol gatt_client_is_ready=0x00008289
--define_symbol gatt_client_listen_for_characteristic_value_updates=0x0000829f
--define_symbol gatt_client_prepare_write=0x000082c1
--define_symbol gatt_client_read_characteristic_descriptor_using_descriptor_handle=0x000082fd
--define_symbol gatt_client_read_long_characteristic_descriptor_using_descriptor_handle=0x00008327
--define_symbol gatt_client_read_long_characteristic_descriptor_using_descriptor_handle_with_offset=0x0000832d
--define_symbol gatt_client_read_long_value_of_characteristic_using_value_handle=0x0000835b
--define_symbol gatt_client_read_long_value_of_characteristic_using_value_handle_with_offset=0x00008361
--define_symbol gatt_client_read_multiple_characteristic_values=0x0000838f
--define_symbol gatt_client_read_value_of_characteristic_using_value_handle=0x000083bf
--define_symbol gatt_client_read_value_of_characteristics_by_uuid128=0x000083ed
--define_symbol gatt_client_read_value_of_characteristics_by_uuid16=0x00008439
--define_symbol gatt_client_register_handler=0x00008485
--define_symbol gatt_client_reliable_write_long_value_of_characteristic=0x00008491
--define_symbol gatt_client_signed_write_without_response=0x000088c1
--define_symbol gatt_client_write_characteristic_descriptor_using_descriptor_handle=0x00008985
--define_symbol gatt_client_write_client_characteristic_configuration=0x000089bf
--define_symbol gatt_client_write_long_characteristic_descriptor_using_descriptor_handle=0x00008a11
--define_symbol gatt_client_write_long_characteristic_descriptor_using_descriptor_handle_with_offset=0x00008a21
--define_symbol gatt_client_write_long_value_of_characteristic=0x00008a5d
--define_symbol gatt_client_write_long_value_of_characteristic_with_offset=0x00008a6d
--define_symbol gatt_client_write_value_of_characteristic=0x00008aa9
--define_symbol gatt_client_write_value_of_characteristic_without_response=0x00008adf
--define_symbol hci_add_event_handler=0x0000a005
--define_symbol hci_power_control=0x0000a7f9
--define_symbol hci_register_acl_packet_handler=0x0000a9ad
--define_symbol kv_commit=0x0000af45
--define_symbol kv_get=0x0000af9d
--define_symbol kv_init=0x0000afb5
--define_symbol kv_put=0x0000b01d
--define_symbol kv_remove=0x0000b095
--define_symbol kv_remove_all=0x0000b0d1
--define_symbol kv_value_modified=0x0000b115
--define_symbol kv_visit=0x0000b119
--define_symbol l2cap_add_event_handler=0x0000b1cd
--define_symbol l2cap_can_send_packet_now=0x0000b1dd
--define_symbol l2cap_create_le_credit_based_connection_request=0x0000b399
--define_symbol l2cap_credit_based_send=0x0000b4dd
--define_symbol l2cap_credit_based_send_continue=0x0000b509
--define_symbol l2cap_disconnect=0x0000b585
--define_symbol l2cap_get_le_credit_based_connection_credits=0x0000b7d5
--define_symbol l2cap_get_peer_mtu_for_local_cid=0x0000b7f1
--define_symbol l2cap_init=0x0000bbc5
--define_symbol l2cap_le_send_flow_control_credit=0x0000bcbb
--define_symbol l2cap_max_le_mtu=0x0000bfc5
--define_symbol l2cap_register_packet_handler=0x0000c0ed
--define_symbol l2cap_register_service=0x0000c0f9
--define_symbol l2cap_request_can_send_now_event=0x0000c209
--define_symbol l2cap_request_connection_parameter_update=0x0000c223
--define_symbol l2cap_send_echo_request=0x0000c6f5
--define_symbol l2cap_unregister_service=0x0000c7b5
--define_symbol le_device_db_add=0x0000c80d
--define_symbol le_device_db_find=0x0000c8e1
--define_symbol le_device_db_from_key=0x0000c90d
--define_symbol le_device_db_iter_cur=0x0000c915
--define_symbol le_device_db_iter_cur_key=0x0000c919
--define_symbol le_device_db_iter_init=0x0000c91d
--define_symbol le_device_db_iter_next=0x0000c925
--define_symbol le_device_db_remove_key=0x0000c94b
--define_symbol ll_aes_encrypt=0x0000c979
--define_symbol ll_free=0x0000c9f5
--define_symbol ll_hint_on_ce_len=0x0000ca01
--define_symbol ll_legacy_adv_set_interval=0x0000ca39
--define_symbol ll_malloc=0x0000ca49
--define_symbol ll_query_timing_info=0x0000cb81
--define_symbol ll_scan_set_fixed_channel=0x0000cc25
--define_symbol ll_set_adv_access_address=0x0000cd39
--define_symbol ll_set_adv_coded_scheme=0x0000cd45
--define_symbol ll_set_conn_coded_scheme=0x0000cd75
--define_symbol ll_set_conn_latency=0x0000cda1
--define_symbol ll_set_conn_tx_power=0x0000cdd1
--define_symbol ll_set_def_antenna=0x0000ce19
--define_symbol ll_set_initiating_coded_scheme=0x0000ce35
--define_symbol ll_set_max_conn_number=0x0000ce41
--define_symbol nibble_for_char=0x0001cd49
--define_symbol platform_32k_rc_auto_tune=0x0001cde5
--define_symbol platform_32k_rc_tune=0x0001ce61
--define_symbol platform_calibrate_32k=0x0001ce75
--define_symbol platform_config=0x0001ce79
--define_symbol platform_controller_run=0x0001cf9d
--define_symbol platform_enable_irq=0x0001cfd5
--define_symbol platform_get_gen_os_driver=0x0001d00d
--define_symbol platform_get_task_handle=0x0001d019
--define_symbol platform_get_us_time=0x0001d031
--define_symbol platform_get_version=0x0001d035
--define_symbol platform_hrng=0x0001d03d
--define_symbol platform_init_controller=0x0001d045
--define_symbol platform_os_idle_resumed_hook=0x0001d061
--define_symbol platform_patch_rf_init_data=0x0001d065
--define_symbol platform_post_sleep_processing=0x0001d071
--define_symbol platform_pre_sleep_processing=0x0001d077
--define_symbol platform_pre_suppress_ticks_and_sleep_processing=0x0001d07d
--define_symbol platform_printf=0x0001d081
--define_symbol platform_raise_assertion=0x0001d095
--define_symbol platform_rand=0x0001d0a9
--define_symbol platform_read_info=0x0001d0ad
--define_symbol platform_read_persistent_reg=0x0001d0dd
--define_symbol platform_reset=0x0001d0ed
--define_symbol platform_set_evt_callback=0x0001d111
--define_symbol platform_set_evt_callback_table=0x0001d125
--define_symbol platform_set_irq_callback=0x0001d131
--define_symbol platform_set_irq_callback_table=0x0001d14d
--define_symbol platform_set_rf_clk_source=0x0001d159
--define_symbol platform_set_rf_init_data=0x0001d165
--define_symbol platform_set_rf_power_mapping=0x0001d171
--define_symbol platform_set_timer=0x0001d17d
--define_symbol platform_shutdown=0x0001d181
--define_symbol platform_switch_app=0x0001d185
--define_symbol platform_trace_raw=0x0001d1b1
--define_symbol platform_write_persistent_reg=0x0001d1c9
--define_symbol printf_hexdump=0x0001d1d9
--define_symbol reverse_128=0x0001d515
--define_symbol reverse_24=0x0001d51b
--define_symbol reverse_48=0x0001d521
--define_symbol reverse_56=0x0001d527
--define_symbol reverse_64=0x0001d52d
--define_symbol reverse_bd_addr=0x0001d533
--define_symbol reverse_bytes=0x0001d539
--define_symbol sm_add_event_handler=0x0001d6a5
--define_symbol sm_address_resolution_lookup=0x0001d7fd
--define_symbol sm_authenticated=0x0001db5d
--define_symbol sm_authorization_decline=0x0001db6b
--define_symbol sm_authorization_grant=0x0001db8b
--define_symbol sm_authorization_state=0x0001dbab
--define_symbol sm_bonding_decline=0x0001dbc5
--define_symbol sm_config=0x0001dfe5
--define_symbol sm_config_conn=0x0001dffd
--define_symbol sm_encryption_key_size=0x0001e1b3
--define_symbol sm_just_works_confirm=0x0001e6ed
--define_symbol sm_le_device_key=0x0001ea29
--define_symbol sm_passkey_input=0x0001eabf
--define_symbol sm_private_random_address_generation_get=0x0001ee6d
--define_symbol sm_private_random_address_generation_get_mode=0x0001ee75
--define_symbol sm_private_random_address_generation_set_mode=0x0001ee81
--define_symbol sm_private_random_address_generation_set_update_period=0x0001eea9
--define_symbol sm_register_oob_data_callback=0x0001efe5
--define_symbol sm_request_pairing=0x0001eff1
--define_symbol sm_send_security_request=0x0001fa2b
--define_symbol sm_set_accepted_stk_generation_methods=0x0001fa51
--define_symbol sm_set_authentication_requirements=0x0001fa5d
--define_symbol sm_set_encryption_key_size_range=0x0001fa69
--define_symbol sscanf_bd_addr=0x0001fdc5
--define_symbol sysSetPublicDeviceAddr=0x00020179
--define_symbol uuid128_to_str=0x0002075d
--define_symbol uuid_add_bluetooth_prefix=0x000207b5
--define_symbol uuid_has_bluetooth_prefix=0x000207d5
