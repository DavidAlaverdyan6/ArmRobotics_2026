"""Keep Telemetrix reporter thread alive on spurious OneWire report packets.

Some UART sessions produce a packet tagged ONE_WIRE_REPORT although the robot
does not enable OneWire. Telemetrix's default _onewire_report unconditionally
calls self.onewire_callback (None), crashing the reporter thread and stopping
all future ADC callbacks. Only ignore packets if NO OneWire callback exists.
Valid OneWire users retain the original implementation.
"""

import functools
import logging
import time

_LOG = logging.getLogger(__name__)
_REPORT_INTERVAL_SECONDS = 10.0


def install_unexpected_onewire_guard(telemetrix_class):
    """Patch the Telemetrix class once, before its first board is constructed.

    No serial protocol, pin configuration or ADC callback semantics are changed.
    """
    original = telemetrix_class._onewire_report
    if getattr(original, "_postintegral_onewire_guard", False):
        return

    @functools.wraps(original)
    def guarded(self, report):
        callback = getattr(self, "onewire_callback", None)
        if callback is not None:
            return original(self, report)

        # An unexpected report suggests UART framing/protocol trouble, not an
        # actual OneWire sensor. Do not let it kill the only reporter thread.
        total = getattr(self, "_postintegral_unexpected_onewire", 0) + 1
        self._postintegral_unexpected_onewire = total
        now = time.monotonic()
        last = getattr(self, "_postintegral_last_onewire_warning", -1e12)
        if total == 1 or now - last >= _REPORT_INTERVAL_SECONDS:
            self._postintegral_last_onewire_warning = now
            # A warning is deliberately retained in terminal scrollback.
            print(
                "TELEMETRIX WARNING: discarded unexpected OneWire report "
                f"without callback (count={total}, payload={report!r}). "
                "Possible UART framing/protocol corruption.",
                flush=True,
            )
        return None

    guarded._postintegral_onewire_guard = True
    telemetrix_class._onewire_report = guarded
