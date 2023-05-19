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
--define_symbol gap_add_dev_to_periodic_list=0x0000717d
--define_symbol gap_add_whitelist=0x0000718d
--define_symbol gap_aes_encrypt=0x00007199
--define_symbol gap_clear_white_lists=0x000071dd
--define_symbol gap_clr_adv_set=0x000071e9
--define_symbol gap_clr_periodic_adv_list=0x000071f5
--define_symbol gap_create_connection_cancel=0x00007201
--define_symbol gap_disconnect=0x0000720d
--define_symbol gap_disconnect_all=0x00007239
--define_symbol gap_ext_create_connection=0x00007279
--define_symbol gap_get_connection_parameter_range=0x00007369
--define_symbol gap_le_read_channel_map=0x000073a1
--define_symbol gap_periodic_adv_create_sync=0x0000740d
--define_symbol gap_periodic_adv_create_sync_cancel=0x00007431
--define_symbol gap_periodic_adv_term_sync=0x0000743d
--define_symbol gap_read_periodic_adv_list_size=0x000074c5
--define_symbol gap_read_phy=0x000074d1
--define_symbol gap_read_remote_used_features=0x000074dd
--define_symbol gap_read_remote_version=0x000074e9
--define_symbol gap_read_rssi=0x000074f5
--define_symbol gap_remove_whitelist=0x00007501
--define_symbol gap_rmv_adv_set=0x0000757d
--define_symbol gap_rmv_dev_from_periodic_list=0x00007589
--define_symbol gap_rx_test_v2=0x00007599
--define_symbol gap_set_adv_set_random_addr=0x000075d1
--define_symbol gap_set_connection_parameter_range=0x0000761d
--define_symbol gap_set_data_length=0x00007639
--define_symbol gap_set_def_phy=0x00007651
--define_symbol gap_set_ext_adv_data=0x00007661
--define_symbol gap_set_ext_adv_enable=0x00007679
--define_symbol gap_set_ext_adv_para=0x000076f5
--define_symbol gap_set_ext_scan_enable=0x000077cd
--define_symbol gap_set_ext_scan_para=0x000077e5
--define_symbol gap_set_ext_scan_response_data=0x0000788d
--define_symbol gap_set_host_channel_classification=0x000078a5
--define_symbol gap_set_periodic_adv_data=0x000078b5
--define_symbol gap_set_periodic_adv_enable=0x00007925
--define_symbol gap_set_periodic_adv_para=0x00007935
--define_symbol gap_set_phy=0x0000794d
--define_symbol gap_set_random_device_address=0x00007969
--define_symbol gap_start_ccm=0x00007999
--define_symbol gap_test_end=0x000079ed
--define_symbol gap_tx_test_v2=0x000079f9
--define_symbol gap_tx_test_v4=0x00007a11
--define_symbol gap_update_connection_parameters=0x00007a35
--define_symbol gap_vendor_tx_continuous_wave=0x00007a79
--define_symbol gatt_client_cancel_write=0x00007fa1
--define_symbol gatt_client_discover_characteristic_descriptors=0x00007fc7
--define_symbol gatt_client_discover_characteristics_for_handle_range_by_uuid128=0x00008007
--define_symbol gatt_client_discover_characteristics_for_handle_range_by_uuid16=0x00008057
--define_symbol gatt_client_discover_characteristics_for_service=0x000080a7
--define_symbol gatt_client_discover_primary_services=0x000080dd
--define_symbol gatt_client_discover_primary_services_by_uuid128=0x0000810f
--define_symbol gatt_client_discover_primary_services_by_uuid16=0x00008153
--define_symbol gatt_client_execute_write=0x0000818f
--define_symbol gatt_client_find_included_services_for_service=0x000081b5
--define_symbol gatt_client_get_mtu=0x000081e3
--define_symbol gatt_client_is_ready=0x00008285
--define_symbol gatt_client_listen_for_characteristic_value_updates=0x0000829b
--define_symbol gatt_client_prepare_write=0x000082bd
--define_symbol gatt_client_read_characteristic_descriptor_using_descriptor_handle=0x000082f9
--define_symbol gatt_client_read_long_characteristic_descriptor_using_descriptor_handle=0x00008323
--define_symbol gatt_client_read_long_characteristic_descriptor_using_descriptor_handle_with_offset=0x00008329
--define_symbol gatt_client_read_long_value_of_characteristic_using_value_handle=0x00008357
--define_symbol gatt_client_read_long_value_of_characteristic_using_value_handle_with_offset=0x0000835d
--define_symbol gatt_client_read_multiple_characteristic_values=0x0000838b
--define_symbol gatt_client_read_value_of_characteristic_using_value_handle=0x000083bb
--define_symbol gatt_client_read_value_of_characteristics_by_uuid128=0x000083e9
--define_symbol gatt_client_read_value_of_characteristics_by_uuid16=0x00008435
--define_symbol gatt_client_register_handler=0x00008481
--define_symbol gatt_client_reliable_write_long_value_of_characteristic=0x0000848d
--define_symbol gatt_client_signed_write_without_response=0x000088bd
--define_symbol gatt_client_write_characteristic_descriptor_using_descriptor_handle=0x00008981
--define_symbol gatt_client_write_client_characteristic_configuration=0x000089bb
--define_symbol gatt_client_write_long_characteristic_descriptor_using_descriptor_handle=0x00008a0d
--define_symbol gatt_client_write_long_characteristic_descriptor_using_descriptor_handle_with_offset=0x00008a1d
--define_symbol gatt_client_write_long_value_of_characteristic=0x00008a59
--define_symbol gatt_client_write_long_value_of_characteristic_with_offset=0x00008a69
--define_symbol gatt_client_write_value_of_characteristic=0x00008aa5
--define_symbol gatt_client_write_value_of_characteristic_without_response=0x00008adb
--define_symbol hci_add_event_handler=0x0000a001
--define_symbol hci_power_control=0x0000a79d
--define_symbol hci_register_acl_packet_handler=0x0000a951
--define_symbol kv_commit=0x0000af55
--define_symbol kv_get=0x0000afad
--define_symbol kv_init=0x0000afc5
--define_symbol kv_put=0x0000b02d
--define_symbol kv_remove=0x0000b0a5
--define_symbol kv_remove_all=0x0000b0e1
--define_symbol kv_value_modified=0x0000b125
--define_symbol kv_visit=0x0000b129
--define_symbol l2cap_add_event_handler=0x0000b1dd
--define_symbol l2cap_can_send_packet_now=0x0000b1ed
--define_symbol l2cap_create_le_credit_based_connection_request=0x0000b3a9
--define_symbol l2cap_credit_based_send=0x0000b4ed
--define_symbol l2cap_credit_based_send_continue=0x0000b519
--define_symbol l2cap_disconnect=0x0000b595
--define_symbol l2cap_get_le_credit_based_connection_credits=0x0000b7e5
--define_symbol l2cap_get_peer_mtu_for_local_cid=0x0000b801
--define_symbol l2cap_init=0x0000bbd5
--define_symbol l2cap_le_send_flow_control_credit=0x0000bccb
--define_symbol l2cap_max_le_mtu=0x0000bfd5
--define_symbol l2cap_register_packet_handler=0x0000c0fd
--define_symbol l2cap_register_service=0x0000c109
--define_symbol l2cap_request_can_send_now_event=0x0000c219
--define_symbol l2cap_request_connection_parameter_update=0x0000c233
--define_symbol l2cap_send_echo_request=0x0000c705
--define_symbol l2cap_unregister_service=0x0000c7c5
--define_symbol le_device_db_add=0x0000c81d
--define_symbol le_device_db_find=0x0000c8f1
--define_symbol le_device_db_from_key=0x0000c91d
--define_symbol le_device_db_iter_cur=0x0000c925
--define_symbol le_device_db_iter_cur_key=0x0000c929
--define_symbol le_device_db_iter_init=0x0000c92d
--define_symbol le_device_db_iter_next=0x0000c935
--define_symbol le_device_db_remove_key=0x0000c95b
--define_symbol ll_aes_encrypt=0x0000c989
--define_symbol ll_free=0x0000ca05
--define_symbol ll_hint_on_ce_len=0x0000ca11
--define_symbol ll_legacy_adv_set_interval=0x0000ca49
--define_symbol ll_malloc=0x0000ca59
--define_symbol ll_query_timing_info=0x0000cb91
--define_symbol ll_scan_set_fixed_channel=0x0000cc35
--define_symbol ll_set_adv_access_address=0x0000cd49
--define_symbol ll_set_adv_coded_scheme=0x0000cd55
--define_symbol ll_set_conn_coded_scheme=0x0000cd85
--define_symbol ll_set_conn_latency=0x0000cdb1
--define_symbol ll_set_conn_tx_power=0x0000cde1
--define_symbol ll_set_def_antenna=0x0000ce29
--define_symbol ll_set_initiating_coded_scheme=0x0000ce45
--define_symbol ll_set_max_conn_number=0x0000ce51
--define_symbol nibble_for_char=0x0001cd81
--define_symbol platform_32k_rc_auto_tune=0x0001ce1d
--define_symbol platform_32k_rc_tune=0x0001ce99
--define_symbol platform_calibrate_32k=0x0001cead
--define_symbol platform_config=0x0001ceb1
--define_symbol platform_controller_run=0x0001cfd5
--define_symbol platform_enable_irq=0x0001d00d
--define_symbol platform_get_gen_os_driver=0x0001d045
--define_symbol platform_get_task_handle=0x0001d051
--define_symbol platform_get_us_time=0x0001d069
--define_symbol platform_get_version=0x0001d06d
--define_symbol platform_hrng=0x0001d075
--define_symbol platform_init_controller=0x0001d07d
--define_symbol platform_os_idle_resumed_hook=0x0001d099
--define_symbol platform_patch_rf_init_data=0x0001d09d
--define_symbol platform_post_sleep_processing=0x0001d0a9
--define_symbol platform_pre_sleep_processing=0x0001d0af
--define_symbol platform_pre_suppress_ticks_and_sleep_processing=0x0001d0b5
--define_symbol platform_printf=0x0001d0b9
--define_symbol platform_raise_assertion=0x0001d0cd
--define_symbol platform_rand=0x0001d0e1
--define_symbol platform_read_info=0x0001d0e5
--define_symbol platform_read_persistent_reg=0x0001d115
--define_symbol platform_reset=0x0001d125
--define_symbol platform_set_evt_callback=0x0001d149
--define_symbol platform_set_evt_callback_table=0x0001d15d
--define_symbol platform_set_irq_callback=0x0001d169
--define_symbol platform_set_irq_callback_table=0x0001d185
--define_symbol platform_set_rf_clk_source=0x0001d191
--define_symbol platform_set_rf_init_data=0x0001d19d
--define_symbol platform_set_rf_power_mapping=0x0001d1a9
--define_symbol platform_set_timer=0x0001d1b5
--define_symbol platform_shutdown=0x0001d1b9
--define_symbol platform_switch_app=0x0001d1bd
--define_symbol platform_trace_raw=0x0001d1e9
--define_symbol platform_write_persistent_reg=0x0001d201
--define_symbol printf_hexdump=0x0001d211
--define_symbol reverse_128=0x0001d54d
--define_symbol reverse_24=0x0001d553
--define_symbol reverse_48=0x0001d559
--define_symbol reverse_56=0x0001d55f
--define_symbol reverse_64=0x0001d565
--define_symbol reverse_bd_addr=0x0001d56b
--define_symbol reverse_bytes=0x0001d571
--define_symbol sm_add_event_handler=0x0001d6dd
--define_symbol sm_address_resolution_lookup=0x0001d835
--define_symbol sm_authenticated=0x0001db95
--define_symbol sm_authorization_decline=0x0001dba3
--define_symbol sm_authorization_grant=0x0001dbc3
--define_symbol sm_authorization_state=0x0001dbe3
--define_symbol sm_bonding_decline=0x0001dbfd
--define_symbol sm_config=0x0001e01d
--define_symbol sm_config_conn=0x0001e035
--define_symbol sm_encryption_key_size=0x0001e1eb
--define_symbol sm_just_works_confirm=0x0001e725
--define_symbol sm_le_device_key=0x0001ea61
--define_symbol sm_passkey_input=0x0001eaf7
--define_symbol sm_private_random_address_generation_get=0x0001eea5
--define_symbol sm_private_random_address_generation_get_mode=0x0001eead
--define_symbol sm_private_random_address_generation_set_mode=0x0001eeb9
--define_symbol sm_private_random_address_generation_set_update_period=0x0001eee1
--define_symbol sm_register_oob_data_callback=0x0001f01d
--define_symbol sm_request_pairing=0x0001f029
--define_symbol sm_send_security_request=0x0001fa63
--define_symbol sm_set_accepted_stk_generation_methods=0x0001fa89
--define_symbol sm_set_authentication_requirements=0x0001fa95
--define_symbol sm_set_encryption_key_size_range=0x0001faa1
--define_symbol sscanf_bd_addr=0x0001fdfd
--define_symbol sysSetPublicDeviceAddr=0x000201b1
--define_symbol uuid128_to_str=0x00020795
--define_symbol uuid_add_bluetooth_prefix=0x000207ed
--define_symbol uuid_has_bluetooth_prefix=0x0002080d
