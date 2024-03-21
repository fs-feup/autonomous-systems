var page_canlib =
[
    [ "Introduction", "page_user_guide_intro.html", [
      [ "Hello, CAN!", "page_user_guide_intro.html#section_user_guide_intro_hello", null ],
      [ "Error checking", "page_user_guide_intro.html#section_user_guide_canstatus", null ],
      [ "CANlib Core API Calls", "page_user_guide_intro.html#section_core_api_calls", null ]
    ] ],
    [ "Initialization", "page_user_guide_init.html", [
      [ "Library Initialization", "page_user_guide_init.html#section_user_guide_init_lib_init", null ],
      [ "Library Deinitialization and Cleanup", "page_user_guide_init.html#section_user_guide_init_lib_deinit", null ],
      [ "Manually Enumerating CAN channels", "page_user_guide_init.html#section_user_guide_enumerate_hw", null ]
    ] ],
    [ "Devices and Channels", "page_user_guide_device_and_channel.html", [
      [ "Identifying Devices and Channels", "page_user_guide_device_and_channel.html#section_user_guide_identifying_devices", null ],
      [ "Channel Information", "page_user_guide_device_and_channel.html#section_user_guide_unique_device", null ],
      [ "Customized Channel Name", "page_user_guide_device_and_channel.html#section_user_guide_cust_channel_name", null ],
      [ "Virtual Channels", "page_user_guide_device_and_channel.html#section_user_guide_virtual", null ]
    ] ],
    [ "Open Channel", "page_user_guide_chips_channels.html", [
      [ "Open as CAN", "page_user_guide_chips_channels.html#section_user_guide_init_sel_channel_can", null ],
      [ "Open as CAN FD", "page_user_guide_chips_channels.html#section_user_guide_init_sel_channel_canfd", null ],
      [ "Close Channel", "page_user_guide_chips_channels.html#section_user_guide_init_sel_channel_close", null ],
      [ "Check Channel Capabilities", "page_user_guide_chips_channels.html#section_user_guide_check_channel_capabilities", null ],
      [ "Set CAN Bitrate", "page_user_guide_chips_channels.html#section_user_guide_init_bit_rate_can_tq", [
        [ "Specifying Bit Timing Parameters", "page_user_guide_chips_channels.html#section_user_guide_init_bit_rate_can_specify_tq", null ],
        [ "Using Predefined Bitrate Constants", "page_user_guide_chips_channels.html#section_user_guide_init_bit_rate_can_predefined", null ]
      ] ],
      [ "Set CAN FD Bitrate", "page_user_guide_chips_channels.html#section_user_guide_init_bit_rate_canfd_tq", [
        [ "Specifying Bit Timing Parameters", "page_user_guide_chips_channels.html#section_user_guide_init_bit_rate_canfd_specify_tq", null ],
        [ "Using Predefined Bitrate Constants", "page_user_guide_chips_channels.html#section_user_guide_init_bit_rate_canfd_predefined", null ]
      ] ],
      [ "CAN Driver Modes", "page_user_guide_chips_channels.html#section_user_guide_init_driver_modes", null ],
      [ "Code Sample", "page_user_guide_chips_channels.html#code_sample", null ],
      [ "Legacy Functions", "page_user_guide_chips_channels.html#section_user_guide_init_bit_rate_legacy", [
        [ "Set CAN Bitrate", "page_user_guide_chips_channels.html#section_user_guide_init_bit_rate_can", null ],
        [ "Set CAN FD Bitrate", "page_user_guide_chips_channels.html#section_user_guide_init_bit_rate_canfd", null ]
      ] ]
    ] ],
    [ "CAN Frame Types", "page_user_guide_can_frame_types_types.html", [
      [ "CAN Data Frames", "page_user_guide_can_frame_types_types.html#section_user_guide_can_frame_types_data_frames", null ],
      [ "CAN FD Data Frames", "page_user_guide_can_frame_types_types.html#section_user_guide_can_frame_types_fd_data_frames", null ],
      [ "Error Frames", "page_user_guide_can_frame_types_types.html#section_user_guide_can_frame_types_error_frame", null ],
      [ "Remote Requests", "page_user_guide_can_frame_types_types.html#section_user_guide_can_frame_types_remote_request", null ],
      [ "Overload Frames", "page_user_guide_can_frame_types_types.html#section_user_guide_can_frame_types_overload_frames", null ],
      [ "Other frame features of interest", "page_user_guide_can_frame_types_types.html#section_user_guide_can_frame_types_other", null ]
    ] ],
    [ "Send and Receive", "page_user_guide_send_recv.html", [
      [ "Bus On / Bus Off", "page_user_guide_send_recv.html#section_user_guide_send_recv_bus_on_off", null ],
      [ "Reading Messages", "page_user_guide_send_recv.html#section_user_guide_send_recv_reading", null ],
      [ "Acceptance Filters", "page_user_guide_send_recv.html#section_user_guide_send_recv_filters", [
        [ "Code and Mask Format", "page_user_guide_send_recv.html#section_user_guide_misc_code_and_mask", null ]
      ] ],
      [ "Sending Messages", "page_user_guide_send_recv.html#section_user_guide_send_recv_sending", null ],
      [ "Object Buffers", "page_user_guide_send_recv.html#section_user_guide_send_recv_obj_buf", null ]
    ] ],
    [ "Bus Errors", "page_user_guide_bus_errors.html", [
      [ "Obtaining Bus Status Information", "page_user_guide_bus_errors.html#section_user_guide_dev_info_status", null ],
      [ "Overruns", "page_user_guide_bus_errors.html#section_user_guide_send_recv_overruns", null ],
      [ "Error Frames", "page_user_guide_bus_errors.html#section_user_guide_bus_errors_error_frames", null ],
      [ "SJA1000 Error Codes", "page_user_guide_bus_errors.html#section_user_guide_bus_errors_sja1000_error_codes", null ]
    ] ],
    [ "Time Measurement", "page_user_guide_time.html", [
      [ "Accuracy", "page_user_guide_time.html#section_user_guide_time_accuracy_and_resolution_accuracy", null ],
      [ "Resolution", "page_user_guide_time.html#section_user_guide_time_accuracy_and_resolution_resolution", null ],
      [ "Time Domain", "page_user_guide_time.html#section_user_guide_time_domain", null ]
    ] ],
    [ "Version Checking", "page_user_guide_version.html", null ],
    [ "Using Threads", "page_user_guide_threads.html", [
      [ "Threaded Applications", "page_user_guide_threads.html#section_user_guide_threads_applications", null ]
    ] ],
    [ "Asynchronous Notification", "page_user_guide_send_recv_asynch_not.html", [
      [ "Asynchronous Notifications", "page_user_guide_send_recv_asynch_not.html#section_user_guide_send_recv_asynch", [
        [ "Receive Events", "page_user_guide_send_recv_asynch_not.html#section_user_guide_send_recv_asynch_receive", null ],
        [ "Transmit Events", "page_user_guide_send_recv_asynch_not.html#section_user_guide_send_recv_asynch_transmit", null ],
        [ "Status Events", "page_user_guide_send_recv_asynch_not.html#section_user_guide_send_recv_asynch_status", null ],
        [ "Error Events", "page_user_guide_send_recv_asynch_not.html#section_user_guide_send_recv_asynch_error", null ]
      ] ],
      [ "Receiving Using Callback Function", "page_user_guide_send_recv_asynch_not.html#section_user_guide_send_recv_asynch_callback", null ]
    ] ],
    [ "t Programming", "page_user_guide_kvscript.html", [
      [ "Overview", "page_user_guide_kvscript.html#section_user_guide_kvscript_overview", null ],
      [ "Load and Unload Script", "page_user_guide_kvscript.html#section_user_guide_kvscript_loading", null ],
      [ "Start and Stop script", "page_user_guide_kvscript.html#section_user_guide_kvscript_start_stop", null ],
      [ "Environment Variable", "page_user_guide_kvscript.html#section_user_guide_kvscript_envvar", null ],
      [ "Send Event", "page_user_guide_kvscript.html#section_user_guide_kvscript_send_event", null ]
    ] ],
    [ "File handling", "page_user_guide_kvfile.html", [
      [ "Overview", "page_user_guide_kvfile.html#section_user_guide_kvfile_overview", null ],
      [ "Enumerate files", "page_user_guide_kvfile.html#section_user_guide_kvfile_enumerate_files", null ],
      [ "Copying files from / to device", "page_user_guide_kvfile.html#section_user_guide_kvfile_copying_files_to_device", null ],
      [ "Deleting files", "page_user_guide_kvfile.html#section_user_guide_kvfile_deleting_files", null ],
      [ "Disk formatting", "page_user_guide_kvfile.html#section_user_guide_kvfile_disk_formatting", null ]
    ] ],
    [ "I/O Pin Handling", "page_user_guide_kviopin.html", [
      [ "Initialize", "page_user_guide_kviopin.html#section_user_guide_kviopin_init", null ],
      [ "Pin information", "page_user_guide_kviopin.html#section_user_guide_kviopin_info", null ],
      [ "IO pin types", "page_user_guide_kviopin.html#section_user_guide_kviopin_type", [
        [ "Analog Pins", "page_user_guide_kviopin.html#section_user_guide_kviopin_type_analog", null ],
        [ "Digital Pins", "page_user_guide_kviopin.html#section_user_guide_kviopin_type_digital", null ],
        [ "Relay Pins", "page_user_guide_kviopin.html#section_user_guide_kviopin_type_relay", null ]
      ] ]
    ] ],
    [ "CANtegrity API (kvDiag)", "page_user_guide_kvdiag.html", [
      [ "Setting up an analyzer", "page_user_guide_kvdiag.html#section_uset_guide_kvdiag_setup", null ],
      [ "Samples", "page_user_guide_kvdiag.html#section_user_guide_kvdiag_samples", null ]
    ] ],
    [ "Message Mailboxes", "page_user_guide_send_recv_mailboxes.html", [
      [ "Message Queue and Buffer Sizes", "page_user_guide_send_recv_mailboxes.html#section_user_guide_send_recv_queue_and_buf_sizes", null ]
    ] ],
    [ "User Data in Kvaser Devices", "page_user_guide_userdata.html", [
      [ "Trying out the concept", "page_user_guide_userdata.html#section_user_guide_userdata_trying", null ],
      [ "Writing user data", "page_user_guide_userdata.html#section_example_c_customerdata_writing", null ],
      [ "Obtaining your own password", "page_user_guide_userdata.html#section_example_c_customerdata_obtaining", null ],
      [ "Reading User Data", "page_user_guide_userdata.html#section_user_guide_userdata_reading", [
        [ "Program to read User Data", "page_user_guide_userdata.html#section_example_c_read_customerdata", null ]
      ] ]
    ] ],
    [ "Windows Advanced Topics", "page_user_guide_install.html", null ],
    [ "CANlib API Calls Grouped by Function", "page_canlib_api_calls_grouped_by_function.html", [
      [ "Information Services", "page_canlib_api_calls_grouped_by_function.html#section_information", null ],
      [ "Channel Open and Close", "page_canlib_api_calls_grouped_by_function.html#section_open", null ],
      [ "Channel Parameters", "page_canlib_api_calls_grouped_by_function.html#section_parameters", null ],
      [ "Receiving Messages", "page_canlib_api_calls_grouped_by_function.html#section_receiving", null ],
      [ "Sending Messages", "page_canlib_api_calls_grouped_by_function.html#section_sending", null ],
      [ "Notification and Waiting", "page_canlib_api_calls_grouped_by_function.html#section_notification", null ],
      [ "Object Buffers", "page_canlib_api_calls_grouped_by_function.html#section_object", null ],
      [ "Miscellaneous", "page_canlib_api_calls_grouped_by_function.html#section_miscellaneous", null ],
      [ "Time Domain Handling", "page_canlib_api_calls_grouped_by_function.html#section_time", null ]
    ] ],
    [ "Sample Programs (CANlib)", "page_user_guide_canlib_samples.html", "page_user_guide_canlib_samples" ]
];