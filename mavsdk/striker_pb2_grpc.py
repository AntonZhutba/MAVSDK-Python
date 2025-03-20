# Generated by the gRPC Python protocol compiler plugin. DO NOT EDIT!
"""Client and server classes corresponding to protobuf-defined services."""
import grpc
import warnings

from . import striker_pb2 as striker_dot_striker__pb2

GRPC_GENERATED_VERSION = '1.70.0'
GRPC_VERSION = grpc.__version__
_version_not_supported = False

try:
    from grpc._utilities import first_version_is_lower
    _version_not_supported = first_version_is_lower(GRPC_VERSION, GRPC_GENERATED_VERSION)
except ImportError:
    _version_not_supported = True

if _version_not_supported:
    raise RuntimeError(
        f'The grpc package installed is at version {GRPC_VERSION},'
        + f' but the generated code in striker/striker_pb2_grpc.py depends on'
        + f' grpcio>={GRPC_GENERATED_VERSION}.'
        + f' Please upgrade your grpc module to grpcio>={GRPC_GENERATED_VERSION}'
        + f' or downgrade your generated code using grpcio-tools<={GRPC_VERSION}.'
    )


class StrikerServiceStub(object):
    """
    Allow users to get vehicle telemetry and state information
    """

    def __init__(self, channel):
        """Constructor.

        Args:
            channel: A grpc.Channel.
        """
        self.SubscribeHeartbeat = channel.unary_stream(
                '/mavsdk.rpc.striker.StrikerService/SubscribeHeartbeat',
                request_serializer=striker_dot_striker__pb2.SubscribeHeartbeatRequest.SerializeToString,
                response_deserializer=striker_dot_striker__pb2.HeartbeatResponse.FromString,
                _registered_method=True)
        self.SubscribeSysStatus = channel.unary_stream(
                '/mavsdk.rpc.striker.StrikerService/SubscribeSysStatus',
                request_serializer=striker_dot_striker__pb2.SubscribeSysStatusRequest.SerializeToString,
                response_deserializer=striker_dot_striker__pb2.SysStatusResponse.FromString,
                _registered_method=True)
        self.SubscribeRcChannel = channel.unary_stream(
                '/mavsdk.rpc.striker.StrikerService/SubscribeRcChannel',
                request_serializer=striker_dot_striker__pb2.SubscribeRcChannelRequest.SerializeToString,
                response_deserializer=striker_dot_striker__pb2.RcChannelResponse.FromString,
                _registered_method=True)
        self.SubscribeMagnitometer = channel.unary_stream(
                '/mavsdk.rpc.striker.StrikerService/SubscribeMagnitometer',
                request_serializer=striker_dot_striker__pb2.SubscribeMagnitometerRequest.SerializeToString,
                response_deserializer=striker_dot_striker__pb2.MagnitometerResponse.FromString,
                _registered_method=True)
        self.SubscribeBatteryVoltages = channel.unary_stream(
                '/mavsdk.rpc.striker.StrikerService/SubscribeBatteryVoltages',
                request_serializer=striker_dot_striker__pb2.SubscribeBatteryVoltagesRequest.SerializeToString,
                response_deserializer=striker_dot_striker__pb2.BatteryVoltagesResponse.FromString,
                _registered_method=True)


class StrikerServiceServicer(object):
    """
    Allow users to get vehicle telemetry and state information
    """

    def SubscribeHeartbeat(self, request, context):
        """Subscribe to 'Heartbeat' updates.
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def SubscribeSysStatus(self, request, context):
        """Subscribe to 'Sys Status' updates.
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def SubscribeRcChannel(self, request, context):
        """Subscribe to 'RC channel' updates.
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def SubscribeMagnitometer(self, request, context):
        """Subscribe to 'Magnitometer' updates.
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def SubscribeBatteryVoltages(self, request, context):
        """Subscribe to 'Battery voltage' updates.
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')


def add_StrikerServiceServicer_to_server(servicer, server):
    rpc_method_handlers = {
            'SubscribeHeartbeat': grpc.unary_stream_rpc_method_handler(
                    servicer.SubscribeHeartbeat,
                    request_deserializer=striker_dot_striker__pb2.SubscribeHeartbeatRequest.FromString,
                    response_serializer=striker_dot_striker__pb2.HeartbeatResponse.SerializeToString,
            ),
            'SubscribeSysStatus': grpc.unary_stream_rpc_method_handler(
                    servicer.SubscribeSysStatus,
                    request_deserializer=striker_dot_striker__pb2.SubscribeSysStatusRequest.FromString,
                    response_serializer=striker_dot_striker__pb2.SysStatusResponse.SerializeToString,
            ),
            'SubscribeRcChannel': grpc.unary_stream_rpc_method_handler(
                    servicer.SubscribeRcChannel,
                    request_deserializer=striker_dot_striker__pb2.SubscribeRcChannelRequest.FromString,
                    response_serializer=striker_dot_striker__pb2.RcChannelResponse.SerializeToString,
            ),
            'SubscribeMagnitometer': grpc.unary_stream_rpc_method_handler(
                    servicer.SubscribeMagnitometer,
                    request_deserializer=striker_dot_striker__pb2.SubscribeMagnitometerRequest.FromString,
                    response_serializer=striker_dot_striker__pb2.MagnitometerResponse.SerializeToString,
            ),
            'SubscribeBatteryVoltages': grpc.unary_stream_rpc_method_handler(
                    servicer.SubscribeBatteryVoltages,
                    request_deserializer=striker_dot_striker__pb2.SubscribeBatteryVoltagesRequest.FromString,
                    response_serializer=striker_dot_striker__pb2.BatteryVoltagesResponse.SerializeToString,
            ),
    }
    generic_handler = grpc.method_handlers_generic_handler(
            'mavsdk.rpc.striker.StrikerService', rpc_method_handlers)
    server.add_generic_rpc_handlers((generic_handler,))
    server.add_registered_method_handlers('mavsdk.rpc.striker.StrikerService', rpc_method_handlers)


 # This class is part of an EXPERIMENTAL API.
class StrikerService(object):
    """
    Allow users to get vehicle telemetry and state information
    """

    @staticmethod
    def SubscribeHeartbeat(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_stream(
            request,
            target,
            '/mavsdk.rpc.striker.StrikerService/SubscribeHeartbeat',
            striker_dot_striker__pb2.SubscribeHeartbeatRequest.SerializeToString,
            striker_dot_striker__pb2.HeartbeatResponse.FromString,
            options,
            channel_credentials,
            insecure,
            call_credentials,
            compression,
            wait_for_ready,
            timeout,
            metadata,
            _registered_method=True)

    @staticmethod
    def SubscribeSysStatus(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_stream(
            request,
            target,
            '/mavsdk.rpc.striker.StrikerService/SubscribeSysStatus',
            striker_dot_striker__pb2.SubscribeSysStatusRequest.SerializeToString,
            striker_dot_striker__pb2.SysStatusResponse.FromString,
            options,
            channel_credentials,
            insecure,
            call_credentials,
            compression,
            wait_for_ready,
            timeout,
            metadata,
            _registered_method=True)

    @staticmethod
    def SubscribeRcChannel(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_stream(
            request,
            target,
            '/mavsdk.rpc.striker.StrikerService/SubscribeRcChannel',
            striker_dot_striker__pb2.SubscribeRcChannelRequest.SerializeToString,
            striker_dot_striker__pb2.RcChannelResponse.FromString,
            options,
            channel_credentials,
            insecure,
            call_credentials,
            compression,
            wait_for_ready,
            timeout,
            metadata,
            _registered_method=True)

    @staticmethod
    def SubscribeMagnitometer(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_stream(
            request,
            target,
            '/mavsdk.rpc.striker.StrikerService/SubscribeMagnitometer',
            striker_dot_striker__pb2.SubscribeMagnitometerRequest.SerializeToString,
            striker_dot_striker__pb2.MagnitometerResponse.FromString,
            options,
            channel_credentials,
            insecure,
            call_credentials,
            compression,
            wait_for_ready,
            timeout,
            metadata,
            _registered_method=True)

    @staticmethod
    def SubscribeBatteryVoltages(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_stream(
            request,
            target,
            '/mavsdk.rpc.striker.StrikerService/SubscribeBatteryVoltages',
            striker_dot_striker__pb2.SubscribeBatteryVoltagesRequest.SerializeToString,
            striker_dot_striker__pb2.BatteryVoltagesResponse.FromString,
            options,
            channel_credentials,
            insecure,
            call_credentials,
            compression,
            wait_for_ready,
            timeout,
            metadata,
            _registered_method=True)
