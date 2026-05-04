#!/usr/bin/env python3
"""
NovaROS BLE pairing agent.

Persistent BlueZ Agent1 with NoInputNoOutput capability. Auto-accepts
JustWorks pairing requests so the drone Pi can bond with new BLE
centrals (the ground bridge, the phone) without operator interaction.

Why we need this:
  - Stock bluez 5.82 ships with a built-in BLE MIDI service (handle
    0x0015) whose characteristic requires encryption per the BLE-MIDI
    spec. Bleak's GATT discovery on the central side reads every
    READ-property characteristic during enumeration; without an
    encrypted link it gets `Insufficient Encryption (0x0F)` from the
    drone-side bluez and tears down with Authentication Failure.
  - The clean fix is link-layer encryption, which BLE does
    automatically once the two sides are bonded. JustWorks pairing
    (no PIN, accept first-come) is consistent with PROMPT.md §4
    ("for now the drone Pi advertises only novadrone-pi and pairs
    first-come") and matches normal phone-to-peripheral UX.

Run as a systemd service (root, system DBus). On startup it registers
itself as the system default agent and stays alive for the lifetime of
bluetoothd. If bluetoothd restarts (DBus name owner changes), the
mainloop re-registers.

Wire format / required methods follow the BlueZ Agent API:
  https://git.kernel.org/pub/scm/bluetooth/bluez.git/tree/doc/agent-api.txt
"""

from __future__ import annotations

import logging
import sys

import dbus
import dbus.mainloop.glib
import dbus.service
from gi.repository import GLib


AGENT_PATH       = "/novaros/agent"
AGENT_CAPABILITY = "NoInputNoOutput"   # JustWorks: no display, no input

BLUEZ_BUS         = "org.bluez"
AGENT_MGR_PATH    = "/org/bluez"
AGENT_MGR_IFACE   = "org.bluez.AgentManager1"
AGENT_IFACE       = "org.bluez.Agent1"


logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s novaros-bt-agent %(message)s",
    stream=sys.stderr,
)
log = logging.getLogger("agent")


class JustWorksAgent(dbus.service.Object):
    """NoInputNoOutput agent — accepts every pairing request without prompting.

    With NoInputNoOutput capability the BlueZ pairing FSM uses
    JustWorks (no MITM protection, no PIN exchange) and only invokes
    AuthorizeService / RequestAuthorization for application-level
    authorization. Both auto-pass."""

    @dbus.service.method(AGENT_IFACE, in_signature="", out_signature="")
    def Release(self):
        log.info("Release()")

    @dbus.service.method(AGENT_IFACE, in_signature="os", out_signature="")
    def AuthorizeService(self, device, uuid):
        log.info("AuthorizeService(%s, %s) → ALLOW", device, uuid)

    @dbus.service.method(AGENT_IFACE, in_signature="o", out_signature="s")
    def RequestPinCode(self, device):
        # NoInputNoOutput shouldn't be asked for a PIN, but BlueZ may
        # call this on legacy paths. Return a placeholder; pairing will
        # likely succeed without real PIN entry on JustWorks anyway.
        log.warning("RequestPinCode(%s) — unexpected for NoInputNoOutput", device)
        return "0000"

    @dbus.service.method(AGENT_IFACE, in_signature="o", out_signature="u")
    def RequestPasskey(self, device):
        log.warning("RequestPasskey(%s) — unexpected for NoInputNoOutput", device)
        return dbus.UInt32(0)

    @dbus.service.method(AGENT_IFACE, in_signature="ouq", out_signature="")
    def DisplayPasskey(self, device, passkey, entered):
        log.info("DisplayPasskey(%s, %06d, %d entered)", device, passkey, entered)

    @dbus.service.method(AGENT_IFACE, in_signature="os", out_signature="")
    def DisplayPinCode(self, device, pincode):
        log.info("DisplayPinCode(%s, %s)", device, pincode)

    @dbus.service.method(AGENT_IFACE, in_signature="ou", out_signature="")
    def RequestConfirmation(self, device, passkey):
        # JustWorks numeric-comparison; no display so we cannot really
        # confirm. Auto-accept matches the "pairs first-come" policy.
        log.info("RequestConfirmation(%s, %06d) → ACCEPT", device, passkey)

    @dbus.service.method(AGENT_IFACE, in_signature="o", out_signature="")
    def RequestAuthorization(self, device):
        log.info("RequestAuthorization(%s) → ALLOW", device)

    @dbus.service.method(AGENT_IFACE, in_signature="", out_signature="")
    def Cancel(self):
        log.info("Cancel()")


def register_agent(bus: dbus.SystemBus) -> None:
    """Register our agent and ask BlueZ to make it the default."""
    mgr = dbus.Interface(bus.get_object(BLUEZ_BUS, AGENT_MGR_PATH), AGENT_MGR_IFACE)
    try:
        mgr.UnregisterAgent(AGENT_PATH)
    except dbus.exceptions.DBusException:
        pass  # not yet registered
    mgr.RegisterAgent(AGENT_PATH, AGENT_CAPABILITY)
    mgr.RequestDefaultAgent(AGENT_PATH)
    log.info("agent registered as default (%s capability)", AGENT_CAPABILITY)


def on_bluez_owner_changed(name: str, old: str, new: str,
                           bus: dbus.SystemBus, agent: JustWorksAgent) -> None:
    """If bluetoothd restarts (`new` becomes non-empty after being empty),
    re-register our agent so we don't drop out silently."""
    if name != BLUEZ_BUS:
        return
    if old and not new:
        log.warning("bluetoothd departed the bus")
    elif new and not old:
        log.info("bluetoothd appeared on the bus — re-registering agent")
        try:
            register_agent(bus)
        except Exception as e:
            log.exception("agent re-register failed: %s", e)


def main() -> int:
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    bus = dbus.SystemBus()

    agent = JustWorksAgent(bus, AGENT_PATH)

    # Initial registration.
    try:
        register_agent(bus)
    except dbus.exceptions.DBusException as e:
        log.error("initial agent register failed: %s", e)
        return 1

    # Watch for bluetoothd restart.
    bus.add_signal_receiver(
        lambda name, old, new: on_bluez_owner_changed(name, old, new, bus, agent),
        signal_name="NameOwnerChanged",
        dbus_interface="org.freedesktop.DBus",
        arg0=BLUEZ_BUS,
    )

    log.info("event loop running; ready to handle pairing requests")
    GLib.MainLoop().run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
