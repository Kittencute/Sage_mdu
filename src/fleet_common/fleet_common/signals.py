import signal
import threading

import rclpy
from rclpy.signals import SignalHandlerOptions

STOP_SIGNALS = {signal.SIGINT, signal.SIGTERM}


def init(args=None) -> None:
    signal.pthread_sigmask(signal.SIG_BLOCK, STOP_SIGNALS)
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    threading.Thread(target=_shutdown_on_stop_signal, daemon=True).start()


def _shutdown_on_stop_signal() -> None:
    signal.sigwait(STOP_SIGNALS)
    rclpy.try_shutdown()
