# Install script for directory: /home/wkd/BBR_ICC/NS3/src/dqc

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "default")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libns3.27-dqc-default.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libns3.27-dqc-default.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libns3.27-dqc-default.so"
         RPATH "/usr/local/lib:$ORIGIN/:$ORIGIN/../lib:/usr/local/lib64:$ORIGIN/:$ORIGIN/../lib64")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/wkd/BBR_ICC/NS3/build/lib/libns3.27-dqc-default.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libns3.27-dqc-default.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libns3.27-dqc-default.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libns3.27-dqc-default.so"
         OLD_RPATH "/home/wkd/BBR_ICC/NS3/build/lib::::::::::::::::::::::::::::::::::::::::::::::::::"
         NEW_RPATH "/usr/local/lib:$ORIGIN/:$ORIGIN/../lib:/usr/local/lib64:$ORIGIN/:$ORIGIN/../lib64")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libns3.27-dqc-default.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ns3" TYPE FILE FILES
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/dqc_delay_ack_receiver.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/dqc_sender.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/dqc_receiver.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/dqc_clock.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/time_tag.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/dqc_trace.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/include/ack_frame.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/include/alarm.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/include/basic_constants.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/include/basic_macro.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/include/byte_codec.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/include/byte_order.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/include/cf_platform.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/include/commandlineflags.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/include/flag_config.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/include/flag_util_impl.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/include/interval.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/include/ip_address.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/include/linked_hash_map.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/include/memslice.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/include/netutils.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/include/ns3_platform.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/include/optional.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/include/packet_number.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/include/packet_number_indexed_queue.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/include/process_alarm_factory.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/include/proto_bandwidth.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/include/proto_circular_deque.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/include/proto_comm.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/include/proto_con.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/include/proto_con_visitor.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/include/proto_connection_stats.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/include/proto_constants.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/include/proto_error_codes.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/include/proto_framer.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/include/proto_packets.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/include/proto_socket.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/include/proto_stream.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/include/proto_stream_data_producer.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/include/proto_stream_sequencer.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/include/proto_time.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/include/proto_types.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/include/proto_utils.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/include/quic_export.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/include/quic_logging.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/include/random.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/include/received_packet_manager.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/include/rtt_monitor.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/include/rtt_stats.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/include/send_packet_manager.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/include/send_receive.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/include/socket_address.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/include/string_utils.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/include/unacked_packet_map.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/congestion/couple_cc_manager.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/congestion/couple_cc_source.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/congestion/dqc_cc_param_config.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/congestion/proto_pacing_sender.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/congestion/proto_send_algorithm_interface.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/congestion/unknow_random.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/logging/base.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/logging/build_config.h"
    "/home/wkd/BBR_ICC/NS3/src/dqc/model/thirdparty/logging/logging.h"
    "/home/wkd/BBR_ICC/NS3/build/include/ns3/dqc-module.h"
    )
endif()

