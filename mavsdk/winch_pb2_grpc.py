# Generated by the gRPC Python protocol compiler plugin. DO NOT EDIT!
"""Client and server classes corresponding to protobuf-defined services."""
import grpc
import warnings

from . import winch_pb2 as winch_dot_winch__pb2

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
        + f' but the generated code in winch/winch_pb2_grpc.py depends on'
        + f' grpcio>={GRPC_GENERATED_VERSION}.'
        + f' Please upgrade your grpc module to grpcio>={GRPC_GENERATED_VERSION}'
        + f' or downgrade your generated code using grpcio-tools<={GRPC_VERSION}.'
    )


class WinchServiceStub(object):
    """
    Allows users to send winch actions, as well as receive status information from winch systems.

    """

    def __init__(self, channel):
        """Constructor.

        Args:
            channel: A grpc.Channel.
        """
        self.SubscribeStatus = channel.unary_stream(
                '/mavsdk.rpc.winch.WinchService/SubscribeStatus',
                request_serializer=winch_dot_winch__pb2.SubscribeStatusRequest.SerializeToString,
                response_deserializer=winch_dot_winch__pb2.StatusResponse.FromString,
                _registered_method=True)
        self.Relax = channel.unary_unary(
                '/mavsdk.rpc.winch.WinchService/Relax',
                request_serializer=winch_dot_winch__pb2.RelaxRequest.SerializeToString,
                response_deserializer=winch_dot_winch__pb2.RelaxResponse.FromString,
                _registered_method=True)
        self.RelativeLengthControl = channel.unary_unary(
                '/mavsdk.rpc.winch.WinchService/RelativeLengthControl',
                request_serializer=winch_dot_winch__pb2.RelativeLengthControlRequest.SerializeToString,
                response_deserializer=winch_dot_winch__pb2.RelativeLengthControlResponse.FromString,
                _registered_method=True)
        self.RateControl = channel.unary_unary(
                '/mavsdk.rpc.winch.WinchService/RateControl',
                request_serializer=winch_dot_winch__pb2.RateControlRequest.SerializeToString,
                response_deserializer=winch_dot_winch__pb2.RateControlResponse.FromString,
                _registered_method=True)
        self.Lock = channel.unary_unary(
                '/mavsdk.rpc.winch.WinchService/Lock',
                request_serializer=winch_dot_winch__pb2.LockRequest.SerializeToString,
                response_deserializer=winch_dot_winch__pb2.LockResponse.FromString,
                _registered_method=True)
        self.Deliver = channel.unary_unary(
                '/mavsdk.rpc.winch.WinchService/Deliver',
                request_serializer=winch_dot_winch__pb2.DeliverRequest.SerializeToString,
                response_deserializer=winch_dot_winch__pb2.DeliverResponse.FromString,
                _registered_method=True)
        self.Hold = channel.unary_unary(
                '/mavsdk.rpc.winch.WinchService/Hold',
                request_serializer=winch_dot_winch__pb2.HoldRequest.SerializeToString,
                response_deserializer=winch_dot_winch__pb2.HoldResponse.FromString,
                _registered_method=True)
        self.Retract = channel.unary_unary(
                '/mavsdk.rpc.winch.WinchService/Retract',
                request_serializer=winch_dot_winch__pb2.RetractRequest.SerializeToString,
                response_deserializer=winch_dot_winch__pb2.RetractResponse.FromString,
                _registered_method=True)
        self.LoadLine = channel.unary_unary(
                '/mavsdk.rpc.winch.WinchService/LoadLine',
                request_serializer=winch_dot_winch__pb2.LoadLineRequest.SerializeToString,
                response_deserializer=winch_dot_winch__pb2.LoadLineResponse.FromString,
                _registered_method=True)
        self.AbandonLine = channel.unary_unary(
                '/mavsdk.rpc.winch.WinchService/AbandonLine',
                request_serializer=winch_dot_winch__pb2.AbandonLineRequest.SerializeToString,
                response_deserializer=winch_dot_winch__pb2.AbandonLineResponse.FromString,
                _registered_method=True)
        self.LoadPayload = channel.unary_unary(
                '/mavsdk.rpc.winch.WinchService/LoadPayload',
                request_serializer=winch_dot_winch__pb2.LoadPayloadRequest.SerializeToString,
                response_deserializer=winch_dot_winch__pb2.LoadPayloadResponse.FromString,
                _registered_method=True)


class WinchServiceServicer(object):
    """
    Allows users to send winch actions, as well as receive status information from winch systems.

    """

    def SubscribeStatus(self, request, context):
        """Subscribe to 'winch status' updates.
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def Relax(self, request, context):
        """
        Allow motor to freewheel.
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def RelativeLengthControl(self, request, context):
        """
        Wind or unwind specified length of line, optionally using specified rate.
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def RateControl(self, request, context):
        """
        Wind or unwind line at specified rate.
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def Lock(self, request, context):
        """
        Perform the locking sequence to relieve motor while in the fully retracted position.
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def Deliver(self, request, context):
        """
        Sequence of drop, slow down, touch down, reel up, lock.
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def Hold(self, request, context):
        """
        Engage motor and hold current position.
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def Retract(self, request, context):
        """
        Return the reel to the fully retracted position.
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def LoadLine(self, request, context):
        """
        Load the reel with line.

        The winch will calculate the total loaded length and stop when the tension exceeds a threshold.
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def AbandonLine(self, request, context):
        """
        Spool out the entire length of the line.
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def LoadPayload(self, request, context):
        """
        Spools out just enough to present the hook to the user to load the payload.
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')


def add_WinchServiceServicer_to_server(servicer, server):
    rpc_method_handlers = {
            'SubscribeStatus': grpc.unary_stream_rpc_method_handler(
                    servicer.SubscribeStatus,
                    request_deserializer=winch_dot_winch__pb2.SubscribeStatusRequest.FromString,
                    response_serializer=winch_dot_winch__pb2.StatusResponse.SerializeToString,
            ),
            'Relax': grpc.unary_unary_rpc_method_handler(
                    servicer.Relax,
                    request_deserializer=winch_dot_winch__pb2.RelaxRequest.FromString,
                    response_serializer=winch_dot_winch__pb2.RelaxResponse.SerializeToString,
            ),
            'RelativeLengthControl': grpc.unary_unary_rpc_method_handler(
                    servicer.RelativeLengthControl,
                    request_deserializer=winch_dot_winch__pb2.RelativeLengthControlRequest.FromString,
                    response_serializer=winch_dot_winch__pb2.RelativeLengthControlResponse.SerializeToString,
            ),
            'RateControl': grpc.unary_unary_rpc_method_handler(
                    servicer.RateControl,
                    request_deserializer=winch_dot_winch__pb2.RateControlRequest.FromString,
                    response_serializer=winch_dot_winch__pb2.RateControlResponse.SerializeToString,
            ),
            'Lock': grpc.unary_unary_rpc_method_handler(
                    servicer.Lock,
                    request_deserializer=winch_dot_winch__pb2.LockRequest.FromString,
                    response_serializer=winch_dot_winch__pb2.LockResponse.SerializeToString,
            ),
            'Deliver': grpc.unary_unary_rpc_method_handler(
                    servicer.Deliver,
                    request_deserializer=winch_dot_winch__pb2.DeliverRequest.FromString,
                    response_serializer=winch_dot_winch__pb2.DeliverResponse.SerializeToString,
            ),
            'Hold': grpc.unary_unary_rpc_method_handler(
                    servicer.Hold,
                    request_deserializer=winch_dot_winch__pb2.HoldRequest.FromString,
                    response_serializer=winch_dot_winch__pb2.HoldResponse.SerializeToString,
            ),
            'Retract': grpc.unary_unary_rpc_method_handler(
                    servicer.Retract,
                    request_deserializer=winch_dot_winch__pb2.RetractRequest.FromString,
                    response_serializer=winch_dot_winch__pb2.RetractResponse.SerializeToString,
            ),
            'LoadLine': grpc.unary_unary_rpc_method_handler(
                    servicer.LoadLine,
                    request_deserializer=winch_dot_winch__pb2.LoadLineRequest.FromString,
                    response_serializer=winch_dot_winch__pb2.LoadLineResponse.SerializeToString,
            ),
            'AbandonLine': grpc.unary_unary_rpc_method_handler(
                    servicer.AbandonLine,
                    request_deserializer=winch_dot_winch__pb2.AbandonLineRequest.FromString,
                    response_serializer=winch_dot_winch__pb2.AbandonLineResponse.SerializeToString,
            ),
            'LoadPayload': grpc.unary_unary_rpc_method_handler(
                    servicer.LoadPayload,
                    request_deserializer=winch_dot_winch__pb2.LoadPayloadRequest.FromString,
                    response_serializer=winch_dot_winch__pb2.LoadPayloadResponse.SerializeToString,
            ),
    }
    generic_handler = grpc.method_handlers_generic_handler(
            'mavsdk.rpc.winch.WinchService', rpc_method_handlers)
    server.add_generic_rpc_handlers((generic_handler,))
    server.add_registered_method_handlers('mavsdk.rpc.winch.WinchService', rpc_method_handlers)


 # This class is part of an EXPERIMENTAL API.
class WinchService(object):
    """
    Allows users to send winch actions, as well as receive status information from winch systems.

    """

    @staticmethod
    def SubscribeStatus(request,
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
            '/mavsdk.rpc.winch.WinchService/SubscribeStatus',
            winch_dot_winch__pb2.SubscribeStatusRequest.SerializeToString,
            winch_dot_winch__pb2.StatusResponse.FromString,
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
    def Relax(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(
            request,
            target,
            '/mavsdk.rpc.winch.WinchService/Relax',
            winch_dot_winch__pb2.RelaxRequest.SerializeToString,
            winch_dot_winch__pb2.RelaxResponse.FromString,
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
    def RelativeLengthControl(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(
            request,
            target,
            '/mavsdk.rpc.winch.WinchService/RelativeLengthControl',
            winch_dot_winch__pb2.RelativeLengthControlRequest.SerializeToString,
            winch_dot_winch__pb2.RelativeLengthControlResponse.FromString,
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
    def RateControl(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(
            request,
            target,
            '/mavsdk.rpc.winch.WinchService/RateControl',
            winch_dot_winch__pb2.RateControlRequest.SerializeToString,
            winch_dot_winch__pb2.RateControlResponse.FromString,
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
    def Lock(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(
            request,
            target,
            '/mavsdk.rpc.winch.WinchService/Lock',
            winch_dot_winch__pb2.LockRequest.SerializeToString,
            winch_dot_winch__pb2.LockResponse.FromString,
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
    def Deliver(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(
            request,
            target,
            '/mavsdk.rpc.winch.WinchService/Deliver',
            winch_dot_winch__pb2.DeliverRequest.SerializeToString,
            winch_dot_winch__pb2.DeliverResponse.FromString,
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
    def Hold(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(
            request,
            target,
            '/mavsdk.rpc.winch.WinchService/Hold',
            winch_dot_winch__pb2.HoldRequest.SerializeToString,
            winch_dot_winch__pb2.HoldResponse.FromString,
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
    def Retract(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(
            request,
            target,
            '/mavsdk.rpc.winch.WinchService/Retract',
            winch_dot_winch__pb2.RetractRequest.SerializeToString,
            winch_dot_winch__pb2.RetractResponse.FromString,
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
    def LoadLine(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(
            request,
            target,
            '/mavsdk.rpc.winch.WinchService/LoadLine',
            winch_dot_winch__pb2.LoadLineRequest.SerializeToString,
            winch_dot_winch__pb2.LoadLineResponse.FromString,
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
    def AbandonLine(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(
            request,
            target,
            '/mavsdk.rpc.winch.WinchService/AbandonLine',
            winch_dot_winch__pb2.AbandonLineRequest.SerializeToString,
            winch_dot_winch__pb2.AbandonLineResponse.FromString,
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
    def LoadPayload(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(
            request,
            target,
            '/mavsdk.rpc.winch.WinchService/LoadPayload',
            winch_dot_winch__pb2.LoadPayloadRequest.SerializeToString,
            winch_dot_winch__pb2.LoadPayloadResponse.FromString,
            options,
            channel_credentials,
            insecure,
            call_credentials,
            compression,
            wait_for_ready,
            timeout,
            metadata,
            _registered_method=True)
