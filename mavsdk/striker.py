# -*- coding: utf-8 -*-
# DO NOT EDIT! This file is auto-generated from
# https://github.com/mavlink/MAVSDK-Python/tree/main/other/templates/py
from ._base import AsyncBase
from . import striker_pb2, striker_pb2_grpc
from enum import Enum


class Heartbeat:
    """
     Heartbeat type.

     Parameters
     ----------
     custom_mode : uint32_t
          A bitfield for use for autopilot-specific flags

     type : uint32_t
          Vehicle or component type. Use MAV_TYPE_* constants

     autopilot : uint32_t
          Autopilot type/class. Use MAV_AUTOPILOT_* constants

     base_mode : uint32_t
          System mode bitmap. Use MAV_MODE_FLAG_* constants

     system_status : uint32_t
          System status flag. Use MAV_STATE_* constants

     mavlink_version : uint32_t
          MAVLink version, set by protocol

     """

    

    def __init__(
            self,
            custom_mode,
            type,
            autopilot,
            base_mode,
            system_status,
            mavlink_version):
        """ Initializes the Heartbeat object """
        self.custom_mode = custom_mode
        self.type = type
        self.autopilot = autopilot
        self.base_mode = base_mode
        self.system_status = system_status
        self.mavlink_version = mavlink_version

    def __eq__(self, to_compare):
        """ Checks if two Heartbeat are the same """
        try:
            # Try to compare - this likely fails when it is compared to a non
            # Heartbeat object
            return \
                (self.custom_mode == to_compare.custom_mode) and \
                (self.type == to_compare.type) and \
                (self.autopilot == to_compare.autopilot) and \
                (self.base_mode == to_compare.base_mode) and \
                (self.system_status == to_compare.system_status) and \
                (self.mavlink_version == to_compare.mavlink_version)

        except AttributeError:
            return False

    def __str__(self):
        """ Heartbeat in string representation """
        struct_repr = ", ".join([
                "custom_mode: " + str(self.custom_mode),
                "type: " + str(self.type),
                "autopilot: " + str(self.autopilot),
                "base_mode: " + str(self.base_mode),
                "system_status: " + str(self.system_status),
                "mavlink_version: " + str(self.mavlink_version)
                ])

        return f"Heartbeat: [{struct_repr}]"

    @staticmethod
    def translate_from_rpc(rpcHeartbeat):
        """ Translates a gRPC struct to the SDK equivalent """
        return Heartbeat(
                
                rpcHeartbeat.custom_mode,
                
                
                rpcHeartbeat.type,
                
                
                rpcHeartbeat.autopilot,
                
                
                rpcHeartbeat.base_mode,
                
                
                rpcHeartbeat.system_status,
                
                
                rpcHeartbeat.mavlink_version
                )

    def translate_to_rpc(self, rpcHeartbeat):
        """ Translates this SDK object into its gRPC equivalent """

        
        
            
        rpcHeartbeat.custom_mode = self.custom_mode
            
        
        
        
            
        rpcHeartbeat.type = self.type
            
        
        
        
            
        rpcHeartbeat.autopilot = self.autopilot
            
        
        
        
            
        rpcHeartbeat.base_mode = self.base_mode
            
        
        
        
            
        rpcHeartbeat.system_status = self.system_status
            
        
        
        
            
        rpcHeartbeat.mavlink_version = self.mavlink_version
            
        
        


class SysStatus:
    """
     System status type.

     Parameters
     ----------
     onboard_control_sensors_present : uint32_t
          Bitmask of onboard controllers and sensors present

     onboard_control_sensors_enabled : uint32_t
          Bitmask of enabled controllers/sensors

     onboard_control_sensors_health : uint32_t
          Bitmask of sensors with errors (0 = error, 1 = healthy)

     load : uint32_t
          [0-1000] Maximum loop load percentage (1000 = 100%)

     voltage_battery : uint32_t
          [mV] Battery voltage, UINT16_MAX if not sent

     current_battery : int32_t
          [cA] Battery current, -1 if not sent

     drop_rate_comm : uint32_t
          [%] Communication drop rate

     errors_comm : uint32_t
          Communication errors count

     errors_count1 : uint32_t
          Autopilot-specific errors

     errors_count2 : uint32_t
          Autopilot-specific errors

     errors_count3 : uint32_t
          Autopilot-specific errors

     errors_count4 : uint32_t
          Autopilot-specific errors

     battery_remaining : int32_t
          [%] Battery energy remaining, -1 if not sent

     onboard_control_sensors_present_extended : uint32_t
          Extended bitmask for present sensors

     onboard_control_sensors_enabled_extended : uint32_t
          Extended bitmask for enabled sensors

     onboard_control_sensors_health_extended : uint32_t
          Extended bitmask for sensor health

     """

    

    def __init__(
            self,
            onboard_control_sensors_present,
            onboard_control_sensors_enabled,
            onboard_control_sensors_health,
            load,
            voltage_battery,
            current_battery,
            drop_rate_comm,
            errors_comm,
            errors_count1,
            errors_count2,
            errors_count3,
            errors_count4,
            battery_remaining,
            onboard_control_sensors_present_extended,
            onboard_control_sensors_enabled_extended,
            onboard_control_sensors_health_extended):
        """ Initializes the SysStatus object """
        self.onboard_control_sensors_present = onboard_control_sensors_present
        self.onboard_control_sensors_enabled = onboard_control_sensors_enabled
        self.onboard_control_sensors_health = onboard_control_sensors_health
        self.load = load
        self.voltage_battery = voltage_battery
        self.current_battery = current_battery
        self.drop_rate_comm = drop_rate_comm
        self.errors_comm = errors_comm
        self.errors_count1 = errors_count1
        self.errors_count2 = errors_count2
        self.errors_count3 = errors_count3
        self.errors_count4 = errors_count4
        self.battery_remaining = battery_remaining
        self.onboard_control_sensors_present_extended = onboard_control_sensors_present_extended
        self.onboard_control_sensors_enabled_extended = onboard_control_sensors_enabled_extended
        self.onboard_control_sensors_health_extended = onboard_control_sensors_health_extended

    def __eq__(self, to_compare):
        """ Checks if two SysStatus are the same """
        try:
            # Try to compare - this likely fails when it is compared to a non
            # SysStatus object
            return \
                (self.onboard_control_sensors_present == to_compare.onboard_control_sensors_present) and \
                (self.onboard_control_sensors_enabled == to_compare.onboard_control_sensors_enabled) and \
                (self.onboard_control_sensors_health == to_compare.onboard_control_sensors_health) and \
                (self.load == to_compare.load) and \
                (self.voltage_battery == to_compare.voltage_battery) and \
                (self.current_battery == to_compare.current_battery) and \
                (self.drop_rate_comm == to_compare.drop_rate_comm) and \
                (self.errors_comm == to_compare.errors_comm) and \
                (self.errors_count1 == to_compare.errors_count1) and \
                (self.errors_count2 == to_compare.errors_count2) and \
                (self.errors_count3 == to_compare.errors_count3) and \
                (self.errors_count4 == to_compare.errors_count4) and \
                (self.battery_remaining == to_compare.battery_remaining) and \
                (self.onboard_control_sensors_present_extended == to_compare.onboard_control_sensors_present_extended) and \
                (self.onboard_control_sensors_enabled_extended == to_compare.onboard_control_sensors_enabled_extended) and \
                (self.onboard_control_sensors_health_extended == to_compare.onboard_control_sensors_health_extended)

        except AttributeError:
            return False

    def __str__(self):
        """ SysStatus in string representation """
        struct_repr = ", ".join([
                "onboard_control_sensors_present: " + str(self.onboard_control_sensors_present),
                "onboard_control_sensors_enabled: " + str(self.onboard_control_sensors_enabled),
                "onboard_control_sensors_health: " + str(self.onboard_control_sensors_health),
                "load: " + str(self.load),
                "voltage_battery: " + str(self.voltage_battery),
                "current_battery: " + str(self.current_battery),
                "drop_rate_comm: " + str(self.drop_rate_comm),
                "errors_comm: " + str(self.errors_comm),
                "errors_count1: " + str(self.errors_count1),
                "errors_count2: " + str(self.errors_count2),
                "errors_count3: " + str(self.errors_count3),
                "errors_count4: " + str(self.errors_count4),
                "battery_remaining: " + str(self.battery_remaining),
                "onboard_control_sensors_present_extended: " + str(self.onboard_control_sensors_present_extended),
                "onboard_control_sensors_enabled_extended: " + str(self.onboard_control_sensors_enabled_extended),
                "onboard_control_sensors_health_extended: " + str(self.onboard_control_sensors_health_extended)
                ])

        return f"SysStatus: [{struct_repr}]"

    @staticmethod
    def translate_from_rpc(rpcSysStatus):
        """ Translates a gRPC struct to the SDK equivalent """
        return SysStatus(
                
                rpcSysStatus.onboard_control_sensors_present,
                
                
                rpcSysStatus.onboard_control_sensors_enabled,
                
                
                rpcSysStatus.onboard_control_sensors_health,
                
                
                rpcSysStatus.load,
                
                
                rpcSysStatus.voltage_battery,
                
                
                rpcSysStatus.current_battery,
                
                
                rpcSysStatus.drop_rate_comm,
                
                
                rpcSysStatus.errors_comm,
                
                
                rpcSysStatus.errors_count1,
                
                
                rpcSysStatus.errors_count2,
                
                
                rpcSysStatus.errors_count3,
                
                
                rpcSysStatus.errors_count4,
                
                
                rpcSysStatus.battery_remaining,
                
                
                rpcSysStatus.onboard_control_sensors_present_extended,
                
                
                rpcSysStatus.onboard_control_sensors_enabled_extended,
                
                
                rpcSysStatus.onboard_control_sensors_health_extended
                )

    def translate_to_rpc(self, rpcSysStatus):
        """ Translates this SDK object into its gRPC equivalent """

        
        
            
        rpcSysStatus.onboard_control_sensors_present = self.onboard_control_sensors_present
            
        
        
        
            
        rpcSysStatus.onboard_control_sensors_enabled = self.onboard_control_sensors_enabled
            
        
        
        
            
        rpcSysStatus.onboard_control_sensors_health = self.onboard_control_sensors_health
            
        
        
        
            
        rpcSysStatus.load = self.load
            
        
        
        
            
        rpcSysStatus.voltage_battery = self.voltage_battery
            
        
        
        
            
        rpcSysStatus.current_battery = self.current_battery
            
        
        
        
            
        rpcSysStatus.drop_rate_comm = self.drop_rate_comm
            
        
        
        
            
        rpcSysStatus.errors_comm = self.errors_comm
            
        
        
        
            
        rpcSysStatus.errors_count1 = self.errors_count1
            
        
        
        
            
        rpcSysStatus.errors_count2 = self.errors_count2
            
        
        
        
            
        rpcSysStatus.errors_count3 = self.errors_count3
            
        
        
        
            
        rpcSysStatus.errors_count4 = self.errors_count4
            
        
        
        
            
        rpcSysStatus.battery_remaining = self.battery_remaining
            
        
        
        
            
        rpcSysStatus.onboard_control_sensors_present_extended = self.onboard_control_sensors_present_extended
            
        
        
        
            
        rpcSysStatus.onboard_control_sensors_enabled_extended = self.onboard_control_sensors_enabled_extended
            
        
        
        
            
        rpcSysStatus.onboard_control_sensors_health_extended = self.onboard_control_sensors_health_extended
            
        
        




class Striker(AsyncBase):
    """
     Allow users to get vehicle telemetry and state information

     Generated by dcsdkgen - MAVSDK Striker API
    """

    # Plugin name
    name = "Striker"

    def _setup_stub(self, channel):
        """ Setups the api stub """
        self._stub = striker_pb2_grpc.StrikerServiceStub(channel)

    

    async def heartbeat(self):
        """
         Subscribe to 'Heartbeat' updates.

         Yields
         -------
         heartbeat : Heartbeat
              Heartbeat data

         
        """

        request = striker_pb2.SubscribeHeartbeatRequest()
        heartbeat_stream = self._stub.SubscribeHeartbeat(request)

        try:
            async for response in heartbeat_stream:
                

            
                yield Heartbeat.translate_from_rpc(response.heartbeat)
        finally:
            heartbeat_stream.cancel()

    async def sys_status(self):
        """
         Subscribe to 'Sys Status' updates.

         Yields
         -------
         sys_status : SysStatus
              System status data

         
        """

        request = striker_pb2.SubscribeSysStatusRequest()
        sys_status_stream = self._stub.SubscribeSysStatus(request)

        try:
            async for response in sys_status_stream:
                

            
                yield SysStatus.translate_from_rpc(response.sys_status)
        finally:
            sys_status_stream.cancel()