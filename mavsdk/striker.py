# -*- coding: utf-8 -*-
# DO NOT EDIT! This file is auto-generated from
# https://github.com/mavlink/MAVSDK-Python/tree/main/other/templates/py
from ._base import AsyncBase
from . import striker_pb2, striker_pb2_grpc
from enum import Enum


class ActuatorServosStatus:
    """
     Actuator Servos Status type.

     Parameters
     ----------
     time_usec : uint64_t
         
     control : [float]
          Need size 8 [-1..1]

     """

    

    def __init__(
            self,
            time_usec,
            control):
        """ Initializes the ActuatorServosStatus object """
        self.time_usec = time_usec
        self.control = control

    def __eq__(self, to_compare):
        """ Checks if two ActuatorServosStatus are the same """
        try:
            # Try to compare - this likely fails when it is compared to a non
            # ActuatorServosStatus object
            return \
                (self.time_usec == to_compare.time_usec) and \
                (self.control == to_compare.control)

        except AttributeError:
            return False

    def __str__(self):
        """ ActuatorServosStatus in string representation """
        struct_repr = ", ".join([
                "time_usec: " + str(self.time_usec),
                "control: " + str(self.control)
                ])

        return f"ActuatorServosStatus: [{struct_repr}]"

    @staticmethod
    def translate_from_rpc(rpcActuatorServosStatus):
        """ Translates a gRPC struct to the SDK equivalent """
        return ActuatorServosStatus(
                
                rpcActuatorServosStatus.time_usec,
                
                
                rpcActuatorServosStatus.control
                )

    def translate_to_rpc(self, rpcActuatorServosStatus):
        """ Translates this SDK object into its gRPC equivalent """

        
        
            
        rpcActuatorServosStatus.time_usec = self.time_usec
            
        
        
        
            
        for elem in self.control:
          rpcActuatorServosStatus.control.append(elem)
            
        
        


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
            
        
        


class RcChannel:
    """
     RC_Channel type.

     Parameters
     ----------
     time_boot_ms : uint32_t
         
     chan1_raw : uint32_t
         
     chan2_raw : uint32_t
         
     chan3_raw : uint32_t
         
     chan4_raw : uint32_t
         
     chan5_raw : uint32_t
         
     chan6_raw : uint32_t
         
     chan7_raw : uint32_t
         
     chan8_raw : uint32_t
         
     chan9_raw : uint32_t
         
     chan10_raw : uint32_t
         
     chan11_raw : uint32_t
         
     chan12_raw : uint32_t
         
     chan13_raw : uint32_t
         
     chan14_raw : uint32_t
         
     chan15_raw : uint32_t
         
     chan16_raw : uint32_t
         
     chan17_raw : uint32_t
         
     chan18_raw : uint32_t
         
     chancount : uint32_t
         
     rssi : uint32_t
         
     """

    

    def __init__(
            self,
            time_boot_ms,
            chan1_raw,
            chan2_raw,
            chan3_raw,
            chan4_raw,
            chan5_raw,
            chan6_raw,
            chan7_raw,
            chan8_raw,
            chan9_raw,
            chan10_raw,
            chan11_raw,
            chan12_raw,
            chan13_raw,
            chan14_raw,
            chan15_raw,
            chan16_raw,
            chan17_raw,
            chan18_raw,
            chancount,
            rssi):
        """ Initializes the RcChannel object """
        self.time_boot_ms = time_boot_ms
        self.chan1_raw = chan1_raw
        self.chan2_raw = chan2_raw
        self.chan3_raw = chan3_raw
        self.chan4_raw = chan4_raw
        self.chan5_raw = chan5_raw
        self.chan6_raw = chan6_raw
        self.chan7_raw = chan7_raw
        self.chan8_raw = chan8_raw
        self.chan9_raw = chan9_raw
        self.chan10_raw = chan10_raw
        self.chan11_raw = chan11_raw
        self.chan12_raw = chan12_raw
        self.chan13_raw = chan13_raw
        self.chan14_raw = chan14_raw
        self.chan15_raw = chan15_raw
        self.chan16_raw = chan16_raw
        self.chan17_raw = chan17_raw
        self.chan18_raw = chan18_raw
        self.chancount = chancount
        self.rssi = rssi

    def __eq__(self, to_compare):
        """ Checks if two RcChannel are the same """
        try:
            # Try to compare - this likely fails when it is compared to a non
            # RcChannel object
            return \
                (self.time_boot_ms == to_compare.time_boot_ms) and \
                (self.chan1_raw == to_compare.chan1_raw) and \
                (self.chan2_raw == to_compare.chan2_raw) and \
                (self.chan3_raw == to_compare.chan3_raw) and \
                (self.chan4_raw == to_compare.chan4_raw) and \
                (self.chan5_raw == to_compare.chan5_raw) and \
                (self.chan6_raw == to_compare.chan6_raw) and \
                (self.chan7_raw == to_compare.chan7_raw) and \
                (self.chan8_raw == to_compare.chan8_raw) and \
                (self.chan9_raw == to_compare.chan9_raw) and \
                (self.chan10_raw == to_compare.chan10_raw) and \
                (self.chan11_raw == to_compare.chan11_raw) and \
                (self.chan12_raw == to_compare.chan12_raw) and \
                (self.chan13_raw == to_compare.chan13_raw) and \
                (self.chan14_raw == to_compare.chan14_raw) and \
                (self.chan15_raw == to_compare.chan15_raw) and \
                (self.chan16_raw == to_compare.chan16_raw) and \
                (self.chan17_raw == to_compare.chan17_raw) and \
                (self.chan18_raw == to_compare.chan18_raw) and \
                (self.chancount == to_compare.chancount) and \
                (self.rssi == to_compare.rssi)

        except AttributeError:
            return False

    def __str__(self):
        """ RcChannel in string representation """
        struct_repr = ", ".join([
                "time_boot_ms: " + str(self.time_boot_ms),
                "chan1_raw: " + str(self.chan1_raw),
                "chan2_raw: " + str(self.chan2_raw),
                "chan3_raw: " + str(self.chan3_raw),
                "chan4_raw: " + str(self.chan4_raw),
                "chan5_raw: " + str(self.chan5_raw),
                "chan6_raw: " + str(self.chan6_raw),
                "chan7_raw: " + str(self.chan7_raw),
                "chan8_raw: " + str(self.chan8_raw),
                "chan9_raw: " + str(self.chan9_raw),
                "chan10_raw: " + str(self.chan10_raw),
                "chan11_raw: " + str(self.chan11_raw),
                "chan12_raw: " + str(self.chan12_raw),
                "chan13_raw: " + str(self.chan13_raw),
                "chan14_raw: " + str(self.chan14_raw),
                "chan15_raw: " + str(self.chan15_raw),
                "chan16_raw: " + str(self.chan16_raw),
                "chan17_raw: " + str(self.chan17_raw),
                "chan18_raw: " + str(self.chan18_raw),
                "chancount: " + str(self.chancount),
                "rssi: " + str(self.rssi)
                ])

        return f"RcChannel: [{struct_repr}]"

    @staticmethod
    def translate_from_rpc(rpcRcChannel):
        """ Translates a gRPC struct to the SDK equivalent """
        return RcChannel(
                
                rpcRcChannel.time_boot_ms,
                
                
                rpcRcChannel.chan1_raw,
                
                
                rpcRcChannel.chan2_raw,
                
                
                rpcRcChannel.chan3_raw,
                
                
                rpcRcChannel.chan4_raw,
                
                
                rpcRcChannel.chan5_raw,
                
                
                rpcRcChannel.chan6_raw,
                
                
                rpcRcChannel.chan7_raw,
                
                
                rpcRcChannel.chan8_raw,
                
                
                rpcRcChannel.chan9_raw,
                
                
                rpcRcChannel.chan10_raw,
                
                
                rpcRcChannel.chan11_raw,
                
                
                rpcRcChannel.chan12_raw,
                
                
                rpcRcChannel.chan13_raw,
                
                
                rpcRcChannel.chan14_raw,
                
                
                rpcRcChannel.chan15_raw,
                
                
                rpcRcChannel.chan16_raw,
                
                
                rpcRcChannel.chan17_raw,
                
                
                rpcRcChannel.chan18_raw,
                
                
                rpcRcChannel.chancount,
                
                
                rpcRcChannel.rssi
                )

    def translate_to_rpc(self, rpcRcChannel):
        """ Translates this SDK object into its gRPC equivalent """

        
        
            
        rpcRcChannel.time_boot_ms = self.time_boot_ms
            
        
        
        
            
        rpcRcChannel.chan1_raw = self.chan1_raw
            
        
        
        
            
        rpcRcChannel.chan2_raw = self.chan2_raw
            
        
        
        
            
        rpcRcChannel.chan3_raw = self.chan3_raw
            
        
        
        
            
        rpcRcChannel.chan4_raw = self.chan4_raw
            
        
        
        
            
        rpcRcChannel.chan5_raw = self.chan5_raw
            
        
        
        
            
        rpcRcChannel.chan6_raw = self.chan6_raw
            
        
        
        
            
        rpcRcChannel.chan7_raw = self.chan7_raw
            
        
        
        
            
        rpcRcChannel.chan8_raw = self.chan8_raw
            
        
        
        
            
        rpcRcChannel.chan9_raw = self.chan9_raw
            
        
        
        
            
        rpcRcChannel.chan10_raw = self.chan10_raw
            
        
        
        
            
        rpcRcChannel.chan11_raw = self.chan11_raw
            
        
        
        
            
        rpcRcChannel.chan12_raw = self.chan12_raw
            
        
        
        
            
        rpcRcChannel.chan13_raw = self.chan13_raw
            
        
        
        
            
        rpcRcChannel.chan14_raw = self.chan14_raw
            
        
        
        
            
        rpcRcChannel.chan15_raw = self.chan15_raw
            
        
        
        
            
        rpcRcChannel.chan16_raw = self.chan16_raw
            
        
        
        
            
        rpcRcChannel.chan17_raw = self.chan17_raw
            
        
        
        
            
        rpcRcChannel.chan18_raw = self.chan18_raw
            
        
        
        
            
        rpcRcChannel.chancount = self.chancount
            
        
        
        
            
        rpcRcChannel.rssi = self.rssi
            
        
        


class Magnitometer:
    """
 

     Parameters
     ----------
     x : double
         
     y : double
         
     z : double
         
     magnetic_heading : double
         
     """

    

    def __init__(
            self,
            x,
            y,
            z,
            magnetic_heading):
        """ Initializes the Magnitometer object """
        self.x = x
        self.y = y
        self.z = z
        self.magnetic_heading = magnetic_heading

    def __eq__(self, to_compare):
        """ Checks if two Magnitometer are the same """
        try:
            # Try to compare - this likely fails when it is compared to a non
            # Magnitometer object
            return \
                (self.x == to_compare.x) and \
                (self.y == to_compare.y) and \
                (self.z == to_compare.z) and \
                (self.magnetic_heading == to_compare.magnetic_heading)

        except AttributeError:
            return False

    def __str__(self):
        """ Magnitometer in string representation """
        struct_repr = ", ".join([
                "x: " + str(self.x),
                "y: " + str(self.y),
                "z: " + str(self.z),
                "magnetic_heading: " + str(self.magnetic_heading)
                ])

        return f"Magnitometer: [{struct_repr}]"

    @staticmethod
    def translate_from_rpc(rpcMagnitometer):
        """ Translates a gRPC struct to the SDK equivalent """
        return Magnitometer(
                
                rpcMagnitometer.x,
                
                
                rpcMagnitometer.y,
                
                
                rpcMagnitometer.z,
                
                
                rpcMagnitometer.magnetic_heading
                )

    def translate_to_rpc(self, rpcMagnitometer):
        """ Translates this SDK object into its gRPC equivalent """

        
        
            
        rpcMagnitometer.x = self.x
            
        
        
        
            
        rpcMagnitometer.y = self.y
            
        
        
        
            
        rpcMagnitometer.z = self.z
            
        
        
        
            
        rpcMagnitometer.magnetic_heading = self.magnetic_heading
            
        
        


class BatteryVoltages:
    """
 

     Parameters
     ----------
     voltages : [uint32_t]
         < [mV] Battery voltage of cells 1 to 10 (see voltages_ext for cells 11-14). Cells in this field above the valid cell count for this battery should have the UINT16_MAX value. If individual cell voltages are unknown or not measured for this battery, then the overall battery voltage should be filled in cell 0, with all others set to UINT16_MAX. If the voltage of the battery is greater than (UINT16_MAX - 1), then cell 0 should be set to (UINT16_MAX - 1), and cell 1 to the remaining voltage. This can be extended to multiple cells if the total voltage is greater than 2 * (UINT16_MAX - 1).
     ext_voltages : [uint32_t]
         < [mV] Battery voltages for cells 11 to 14. Cells above the valid cell count for this battery should have a value of 0, where zero indicates not supported (note, this is different than for the voltages field and allows empty byte truncation). If the measured value is 0 then 1 should be sent instead.
     """

    

    def __init__(
            self,
            voltages,
            ext_voltages):
        """ Initializes the BatteryVoltages object """
        self.voltages = voltages
        self.ext_voltages = ext_voltages

    def __eq__(self, to_compare):
        """ Checks if two BatteryVoltages are the same """
        try:
            # Try to compare - this likely fails when it is compared to a non
            # BatteryVoltages object
            return \
                (self.voltages == to_compare.voltages) and \
                (self.ext_voltages == to_compare.ext_voltages)

        except AttributeError:
            return False

    def __str__(self):
        """ BatteryVoltages in string representation """
        struct_repr = ", ".join([
                "voltages: " + str(self.voltages),
                "ext_voltages: " + str(self.ext_voltages)
                ])

        return f"BatteryVoltages: [{struct_repr}]"

    @staticmethod
    def translate_from_rpc(rpcBatteryVoltages):
        """ Translates a gRPC struct to the SDK equivalent """
        return BatteryVoltages(
                
                rpcBatteryVoltages.voltages,
                
                
                rpcBatteryVoltages.ext_voltages
                )

    def translate_to_rpc(self, rpcBatteryVoltages):
        """ Translates this SDK object into its gRPC equivalent """

        
        
            
        for elem in self.voltages:
          rpcBatteryVoltages.voltages.append(elem)
            
        
        
        
            
        for elem in self.ext_voltages:
          rpcBatteryVoltages.ext_voltages.append(elem)
            
        
        


class AvailableMode:
    """
 

     Parameters
     ----------
     number_modes : uint32_t
          Total number of available modes

     mode_index : uint32_t
          Index of this mode (1-based)

     standard_mode : uint32_t
          Standard MAV_STANDARD_MODE enum

     custom_mode : uint32_t
          Custom mode (autopilot-specific)

     properties : uint32_t
          Bitmask of mode properties

     mode_name : std::string
          Human-readable mode name

     """

    

    def __init__(
            self,
            number_modes,
            mode_index,
            standard_mode,
            custom_mode,
            properties,
            mode_name):
        """ Initializes the AvailableMode object """
        self.number_modes = number_modes
        self.mode_index = mode_index
        self.standard_mode = standard_mode
        self.custom_mode = custom_mode
        self.properties = properties
        self.mode_name = mode_name

    def __eq__(self, to_compare):
        """ Checks if two AvailableMode are the same """
        try:
            # Try to compare - this likely fails when it is compared to a non
            # AvailableMode object
            return \
                (self.number_modes == to_compare.number_modes) and \
                (self.mode_index == to_compare.mode_index) and \
                (self.standard_mode == to_compare.standard_mode) and \
                (self.custom_mode == to_compare.custom_mode) and \
                (self.properties == to_compare.properties) and \
                (self.mode_name == to_compare.mode_name)

        except AttributeError:
            return False

    def __str__(self):
        """ AvailableMode in string representation """
        struct_repr = ", ".join([
                "number_modes: " + str(self.number_modes),
                "mode_index: " + str(self.mode_index),
                "standard_mode: " + str(self.standard_mode),
                "custom_mode: " + str(self.custom_mode),
                "properties: " + str(self.properties),
                "mode_name: " + str(self.mode_name)
                ])

        return f"AvailableMode: [{struct_repr}]"

    @staticmethod
    def translate_from_rpc(rpcAvailableMode):
        """ Translates a gRPC struct to the SDK equivalent """
        return AvailableMode(
                
                rpcAvailableMode.number_modes,
                
                
                rpcAvailableMode.mode_index,
                
                
                rpcAvailableMode.standard_mode,
                
                
                rpcAvailableMode.custom_mode,
                
                
                rpcAvailableMode.properties,
                
                
                rpcAvailableMode.mode_name
                )

    def translate_to_rpc(self, rpcAvailableMode):
        """ Translates this SDK object into its gRPC equivalent """

        
        
            
        rpcAvailableMode.number_modes = self.number_modes
            
        
        
        
            
        rpcAvailableMode.mode_index = self.mode_index
            
        
        
        
            
        rpcAvailableMode.standard_mode = self.standard_mode
            
        
        
        
            
        rpcAvailableMode.custom_mode = self.custom_mode
            
        
        
        
            
        rpcAvailableMode.properties = self.properties
            
        
        
        
            
        rpcAvailableMode.mode_name = self.mode_name
            
        
        


class StrikerResult:
    """
 

     Parameters
     ----------
     result : Result
          ModeResult enum value

     result_str : std::string
          Human-readable English string describing the result

     """

    
    
    class Result(Enum):
        """
         Possible results returned for action requests.

         Values
         ------
         UNKNOWN
              Unknown result

         SUCCESS
              Request was successful

         NO_SYSTEM
              No system is connected

         CONNECTION_ERROR
              Connection error

         BUSY
              Vehicle is busy

         COMMAND_DENIED
              Command refused by vehicle

         COMMAND_DENIED_LANDED_STATE_UNKNOWN
              Command refused because landed state is unknown

         COMMAND_DENIED_NOT_LANDED
              Command refused because vehicle not landed

         TIMEOUT
              Request timed out

         VTOL_TRANSITION_SUPPORT_UNKNOWN
              Hybrid/VTOL transition support is unknown

         NO_VTOL_TRANSITION_SUPPORT
              Vehicle does not support hybrid/VTOL transitions

         PARAMETER_ERROR
              Error getting or setting parameter

         UNSUPPORTED
              Action not supported

         FAILED
              Action failed

         INVALID_ARGUMENT
              Invalid argument

         """

        
        UNKNOWN = 0
        SUCCESS = 1
        NO_SYSTEM = 2
        CONNECTION_ERROR = 3
        BUSY = 4
        COMMAND_DENIED = 5
        COMMAND_DENIED_LANDED_STATE_UNKNOWN = 6
        COMMAND_DENIED_NOT_LANDED = 7
        TIMEOUT = 8
        VTOL_TRANSITION_SUPPORT_UNKNOWN = 9
        NO_VTOL_TRANSITION_SUPPORT = 10
        PARAMETER_ERROR = 11
        UNSUPPORTED = 12
        FAILED = 13
        INVALID_ARGUMENT = 14

        def translate_to_rpc(self):
            if self == StrikerResult.Result.UNKNOWN:
                return striker_pb2.StrikerResult.RESULT_UNKNOWN
            if self == StrikerResult.Result.SUCCESS:
                return striker_pb2.StrikerResult.RESULT_SUCCESS
            if self == StrikerResult.Result.NO_SYSTEM:
                return striker_pb2.StrikerResult.RESULT_NO_SYSTEM
            if self == StrikerResult.Result.CONNECTION_ERROR:
                return striker_pb2.StrikerResult.RESULT_CONNECTION_ERROR
            if self == StrikerResult.Result.BUSY:
                return striker_pb2.StrikerResult.RESULT_BUSY
            if self == StrikerResult.Result.COMMAND_DENIED:
                return striker_pb2.StrikerResult.RESULT_COMMAND_DENIED
            if self == StrikerResult.Result.COMMAND_DENIED_LANDED_STATE_UNKNOWN:
                return striker_pb2.StrikerResult.RESULT_COMMAND_DENIED_LANDED_STATE_UNKNOWN
            if self == StrikerResult.Result.COMMAND_DENIED_NOT_LANDED:
                return striker_pb2.StrikerResult.RESULT_COMMAND_DENIED_NOT_LANDED
            if self == StrikerResult.Result.TIMEOUT:
                return striker_pb2.StrikerResult.RESULT_TIMEOUT
            if self == StrikerResult.Result.VTOL_TRANSITION_SUPPORT_UNKNOWN:
                return striker_pb2.StrikerResult.RESULT_VTOL_TRANSITION_SUPPORT_UNKNOWN
            if self == StrikerResult.Result.NO_VTOL_TRANSITION_SUPPORT:
                return striker_pb2.StrikerResult.RESULT_NO_VTOL_TRANSITION_SUPPORT
            if self == StrikerResult.Result.PARAMETER_ERROR:
                return striker_pb2.StrikerResult.RESULT_PARAMETER_ERROR
            if self == StrikerResult.Result.UNSUPPORTED:
                return striker_pb2.StrikerResult.RESULT_UNSUPPORTED
            if self == StrikerResult.Result.FAILED:
                return striker_pb2.StrikerResult.RESULT_FAILED
            if self == StrikerResult.Result.INVALID_ARGUMENT:
                return striker_pb2.StrikerResult.RESULT_INVALID_ARGUMENT

        @staticmethod
        def translate_from_rpc(rpc_enum_value):
            """ Parses a gRPC response """
            if rpc_enum_value == striker_pb2.StrikerResult.RESULT_UNKNOWN:
                return StrikerResult.Result.UNKNOWN
            if rpc_enum_value == striker_pb2.StrikerResult.RESULT_SUCCESS:
                return StrikerResult.Result.SUCCESS
            if rpc_enum_value == striker_pb2.StrikerResult.RESULT_NO_SYSTEM:
                return StrikerResult.Result.NO_SYSTEM
            if rpc_enum_value == striker_pb2.StrikerResult.RESULT_CONNECTION_ERROR:
                return StrikerResult.Result.CONNECTION_ERROR
            if rpc_enum_value == striker_pb2.StrikerResult.RESULT_BUSY:
                return StrikerResult.Result.BUSY
            if rpc_enum_value == striker_pb2.StrikerResult.RESULT_COMMAND_DENIED:
                return StrikerResult.Result.COMMAND_DENIED
            if rpc_enum_value == striker_pb2.StrikerResult.RESULT_COMMAND_DENIED_LANDED_STATE_UNKNOWN:
                return StrikerResult.Result.COMMAND_DENIED_LANDED_STATE_UNKNOWN
            if rpc_enum_value == striker_pb2.StrikerResult.RESULT_COMMAND_DENIED_NOT_LANDED:
                return StrikerResult.Result.COMMAND_DENIED_NOT_LANDED
            if rpc_enum_value == striker_pb2.StrikerResult.RESULT_TIMEOUT:
                return StrikerResult.Result.TIMEOUT
            if rpc_enum_value == striker_pb2.StrikerResult.RESULT_VTOL_TRANSITION_SUPPORT_UNKNOWN:
                return StrikerResult.Result.VTOL_TRANSITION_SUPPORT_UNKNOWN
            if rpc_enum_value == striker_pb2.StrikerResult.RESULT_NO_VTOL_TRANSITION_SUPPORT:
                return StrikerResult.Result.NO_VTOL_TRANSITION_SUPPORT
            if rpc_enum_value == striker_pb2.StrikerResult.RESULT_PARAMETER_ERROR:
                return StrikerResult.Result.PARAMETER_ERROR
            if rpc_enum_value == striker_pb2.StrikerResult.RESULT_UNSUPPORTED:
                return StrikerResult.Result.UNSUPPORTED
            if rpc_enum_value == striker_pb2.StrikerResult.RESULT_FAILED:
                return StrikerResult.Result.FAILED
            if rpc_enum_value == striker_pb2.StrikerResult.RESULT_INVALID_ARGUMENT:
                return StrikerResult.Result.INVALID_ARGUMENT

        def __str__(self):
            return self.name
    

    def __init__(
            self,
            result,
            result_str):
        """ Initializes the StrikerResult object """
        self.result = result
        self.result_str = result_str

    def __eq__(self, to_compare):
        """ Checks if two StrikerResult are the same """
        try:
            # Try to compare - this likely fails when it is compared to a non
            # StrikerResult object
            return \
                (self.result == to_compare.result) and \
                (self.result_str == to_compare.result_str)

        except AttributeError:
            return False

    def __str__(self):
        """ StrikerResult in string representation """
        struct_repr = ", ".join([
                "result: " + str(self.result),
                "result_str: " + str(self.result_str)
                ])

        return f"StrikerResult: [{struct_repr}]"

    @staticmethod
    def translate_from_rpc(rpcStrikerResult):
        """ Translates a gRPC struct to the SDK equivalent """
        return StrikerResult(
                
                StrikerResult.Result.translate_from_rpc(rpcStrikerResult.result),
                
                
                rpcStrikerResult.result_str
                )

    def translate_to_rpc(self, rpcStrikerResult):
        """ Translates this SDK object into its gRPC equivalent """

        
        
            
        rpcStrikerResult.result = self.result.translate_to_rpc()
            
        
        
        
            
        rpcStrikerResult.result_str = self.result_str
            
        
        



class StrikerError(Exception):
    """ Raised when a StrikerResult is a fail code """

    def __init__(self, result, origin, *params):
        self._result = result
        self._origin = origin
        self._params = params

    def __str__(self):
        return f"{self._result.result}: '{self._result.result_str}'; origin: {self._origin}; params: {self._params}"


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

    
    def _extract_result(self, response):
        """ Returns the response status and description """
        return StrikerResult.translate_from_rpc(response.result)
    

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

    async def rc_channel(self):
        """
         Subscribe to 'RC channel' updates.

         Yields
         -------
         rc_channel : RcChannel
              RC_Channel data

         
        """

        request = striker_pb2.SubscribeRcChannelRequest()
        rc_channel_stream = self._stub.SubscribeRcChannel(request)

        try:
            async for response in rc_channel_stream:
                

            
                yield RcChannel.translate_from_rpc(response.rc_channel)
        finally:
            rc_channel_stream.cancel()

    async def magnitometer(self):
        """
         Subscribe to 'Magnitometer' updates.

         Yields
         -------
         magnitometer : Magnitometer
              Magnitometer data

         
        """

        request = striker_pb2.SubscribeMagnitometerRequest()
        magnitometer_stream = self._stub.SubscribeMagnitometer(request)

        try:
            async for response in magnitometer_stream:
                

            
                yield Magnitometer.translate_from_rpc(response.magnitometer)
        finally:
            magnitometer_stream.cancel()

    async def battery_voltages(self):
        """
         Subscribe to 'Battery voltage' updates.

         Yields
         -------
         battery_voltages : BatteryVoltages
              BatteryVoltages data

         
        """

        request = striker_pb2.SubscribeBatteryVoltagesRequest()
        battery_voltages_stream = self._stub.SubscribeBatteryVoltages(request)

        try:
            async for response in battery_voltages_stream:
                

            
                yield BatteryVoltages.translate_from_rpc(response.battery_voltages)
        finally:
            battery_voltages_stream.cancel()

    async def available_modes(self):
        """
         Subscribe to 'Available Modes' updates.

         Yields
         -------
         available_modes : [AvailableMode]
             
         
        """

        request = striker_pb2.SubscribeAvailableModesRequest()
        available_modes_stream = self._stub.SubscribeAvailableModes(request)

        try:
            async for response in available_modes_stream:
                

            
                yield list(map(lambda x : AvailableMode.translate_from_rpc(x), response.available_modes))
        finally:
            available_modes_stream.cancel()

    async def set_manual_flight_mode(self, mode, custom_mode, custom_sub_mode):
        """
         Set the vehicle mode.

         Parameters
         ----------
         mode : uint32_t
              Standard MAV_STANDARD_MODE enum

         custom_mode : uint32_t
              Custom mode (autopilot-specific)

         custom_sub_mode : uint32_t
              Custom sub-mode (autopilot-specific)

         Raises
         ------
         StrikerError
             If the request fails. The error contains the reason for the failure.
        """

        request = striker_pb2.SetManualFlightModeRequest()
        request.mode = mode
        request.custom_mode = custom_mode
        request.custom_sub_mode = custom_sub_mode
        response = await self._stub.SetManualFlightMode(request)

        
        result = self._extract_result(response)

        if result.result != StrikerResult.Result.SUCCESS:
            raise StrikerError(result, "set_manual_flight_mode()", mode, custom_mode, custom_sub_mode)
        

    async def actuator_servos_status(self):
        """
         Subscribe to 'Actuator Servos Status' updates.

         Yields
         -------
         actuator_servos_status : ActuatorServosStatus
             
         
        """

        request = striker_pb2.SubscribeActuatorServosStatusRequest()
        actuator_servos_status_stream = self._stub.SubscribeActuatorServosStatus(request)

        try:
            async for response in actuator_servos_status_stream:
                

            
                yield ActuatorServosStatus.translate_from_rpc(response.actuator_servos_status)
        finally:
            actuator_servos_status_stream.cancel()

    async def set_rate_actuator_servos_status(self, rate_hz):
        """
         Set the actuator control target.

         Parameters
         ----------
         rate_hz : double
              The requested rate (in Hertz)

         Raises
         ------
         StrikerError
             If the request fails. The error contains the reason for the failure.
        """

        request = striker_pb2.SetRateActuatorServosStatusRequest()
        request.rate_hz = rate_hz
        response = await self._stub.SetRateActuatorServosStatus(request)

        
        result = self._extract_result(response)

        if result.result != StrikerResult.Result.SUCCESS:
            raise StrikerError(result, "set_rate_actuator_servos_status()", rate_hz)
        

    async def request_available_modes(self):
        """
         Request available modes.

         Raises
         ------
         StrikerError
             If the request fails. The error contains the reason for the failure.
        """

        request = striker_pb2.RequestAvailableModesRequest()
        response = await self._stub.RequestAvailableModes(request)

        
        result = self._extract_result(response)

        if result.result != StrikerResult.Result.SUCCESS:
            raise StrikerError(result, "request_available_modes()")
        