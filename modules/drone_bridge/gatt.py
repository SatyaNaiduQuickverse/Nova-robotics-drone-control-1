"""BlueZ GATT object implementations (direct DBus, no bless).

This module exports the four DBus interfaces BlueZ needs to manage a
GATT peripheral application:

  * `GattService`         org.bluez.GattService1
  * `GattCharacteristic`  org.bluez.GattCharacteristic1
  * `GattDescriptor`      org.bluez.GattDescriptor1   (used here only for CCCD)
  * `Advertisement`       org.bluez.LEAdvertisement1
  * `GattApplication`     org.freedesktop.DBus.ObjectManager
                          (the root that BlueZ introspects to discover the rest)

Why direct DBus (replacing bless): bless silently de-registers the
GATT application AND advertisement when a central disconnects, with
no callback or log line. We need explicit ownership of state so a
disconnect → re-advertise round-trip is visible, deterministic, and
can be re-tried without restarting the process.

Wire-level conventions:
  * Object-path layout is a tree under `APP_PATH`. BlueZ traverses it
    via ObjectManager.GetManagedObjects.
  * Property `Value` on a characteristic carries the latest payload
    for reads; updates emit PropertiesChanged so subscribed centrals
    receive notifications (BlueZ handles the actual ATT notify PDU).
"""

from __future__ import annotations

import logging
from typing import Any, Awaitable, Callable, Optional

from dbus_fast import Variant
from dbus_fast.constants import PropertyAccess
from dbus_fast.service import ServiceInterface, dbus_property, method


log = logging.getLogger("drone_bridge.gatt")


# --- Object-path layout (rooted at APP_PATH; BlueZ enumerates from here) ---

APP_PATH       = "/com/novaros/dronebridge"
SERVICE_PATH   = f"{APP_PATH}/service0"
CHAR_REQ_PATH  = f"{SERVICE_PATH}/char0"
CHAR_RSP_PATH  = f"{SERVICE_PATH}/char1"
DESC_CCCD_PATH = f"{CHAR_RSP_PATH}/desc0"
ADV_PATH       = f"{APP_PATH}/advertisement0"


# --- Type aliases for clarity ---

WriteCallback     = Callable[[bytes], Awaitable[None]]
SubscribeCallback = Callable[[bool], None]


# ---------------------------------------------------------------------------
# org.bluez.GattService1
# ---------------------------------------------------------------------------

class GattService(ServiceInterface):
    """A BLE GATT service. Properties only — no methods.

    BlueZ uses the UUID + Primary properties to advertise this service
    in the GATT tree it exposes to centrals. The service's
    characteristics live at child paths under `SERVICE_PATH`.
    """

    def __init__(self, uuid: str, primary: bool = True):
        super().__init__("org.bluez.GattService1")
        self._uuid = uuid
        self._primary = primary

    @dbus_property(access=PropertyAccess.READ)
    def UUID(self) -> "s":  # noqa: N802 — DBus property name, must match
        return self._uuid

    @dbus_property(access=PropertyAccess.READ)
    def Primary(self) -> "b":  # noqa: N802
        return self._primary


# ---------------------------------------------------------------------------
# org.bluez.GattCharacteristic1
# ---------------------------------------------------------------------------

class GattCharacteristic(ServiceInterface):
    """A characteristic in a GATT service.

    Behavior per use-case:

      * **CHAR_REQUEST** (write-only). `_write_cb` is invoked when the
        central writes. BlueZ assembles Prepared Writes (Write Long
        Value) internally and calls our WriteValue once with the full
        body; we forward as one chunk to the application.

      * **CHAR_RESPONSE** (read+notify). The application calls
        `update_value()` to push a fragment; we set the Value property
        and emit PropertiesChanged, which BlueZ converts into an ATT
        notify PDU for every central that has called StartNotify.
        Notify subscription state is tracked in `_notifying` so the
        application can decide whether sending is worth the radio time.
    """

    def __init__(
        self,
        uuid: str,
        flags: list[str],
        service_path: str,
        write_cb: Optional[WriteCallback] = None,
        on_subscribe: Optional[SubscribeCallback] = None,
    ):
        super().__init__("org.bluez.GattCharacteristic1")
        self._uuid = uuid
        self._flags = flags
        self._service_path = service_path
        self._value = bytearray()
        self._notifying = False
        self._write_cb = write_cb
        self._on_subscribe = on_subscribe

    # --- Properties ------------------------------------------------------

    @dbus_property(access=PropertyAccess.READ)
    def UUID(self) -> "s":  # noqa: N802
        return self._uuid

    @dbus_property(access=PropertyAccess.READ)
    def Service(self) -> "o":  # noqa: N802
        return self._service_path

    @dbus_property(access=PropertyAccess.READ)
    def Flags(self) -> "as":  # noqa: N802
        return self._flags

    @dbus_property(access=PropertyAccess.READ)
    def Value(self) -> "ay":  # noqa: N802
        return bytes(self._value)

    @dbus_property(access=PropertyAccess.READ)
    def Notifying(self) -> "b":  # noqa: N802
        return self._notifying

    # --- Methods ---------------------------------------------------------

    @method()
    def ReadValue(self, options: "a{sv}") -> "ay":  # noqa: N802
        # Most write-only chars never get read; for notify chars BlueZ
        # may read once on connect to seed the value. Either is harmless.
        return bytes(self._value)

    @method()
    async def WriteValue(self, value: "ay", options: "a{sv}"):  # noqa: N802
        # `options` may include `offset` for legacy Prepared Writes, but
        # in practice BlueZ on Linux assembles internally and delivers
        # the full body in one call. Forward as-is to the app.
        if self._write_cb is None:
            log.warning("WriteValue on %s but no callback registered", self._uuid)
            return
        try:
            await self._write_cb(bytes(value))
        except Exception:
            # Don't let an app-level exception kill the DBus thread —
            # BlueZ will surface as a generic GATT error to the central.
            log.exception("write callback raised on char %s", self._uuid)

    @method()
    def StartNotify(self):  # noqa: N802
        if self._notifying:
            return
        self._notifying = True
        log.info("notify subscribed on %s", self._uuid)
        if self._on_subscribe:
            try:
                self._on_subscribe(True)
            except Exception:
                log.exception("on_subscribe callback raised")

    @method()
    def StopNotify(self):  # noqa: N802
        if not self._notifying:
            return
        self._notifying = False
        log.info("notify unsubscribed on %s", self._uuid)
        if self._on_subscribe:
            try:
                self._on_subscribe(False)
            except Exception:
                log.exception("on_subscribe callback raised")

    # --- App-facing helper -----------------------------------------------

    def update_value(self, new_value: bytes) -> None:
        """Set the characteristic value and notify any subscriber.

        Setting `_value` then emitting PropertiesChanged is what BlueZ
        translates into an ATT Notify PDU. Calling this with the
        characteristic's `notify` flag is what triggers the radio
        transmit; for non-notify chars it just updates the cache.
        """
        self._value = bytearray(new_value)
        # ServiceInterface.emit_properties_changed wraps values as Variants
        # appropriate for the declared property signature ('ay' here).
        self.emit_properties_changed({"Value": bytes(new_value)})

    @property
    def notifying(self) -> bool:
        """Read-only Python view of the subscription state. Use this to
        skip an expensive serialize+notify when nobody is listening."""
        return self._notifying


# ---------------------------------------------------------------------------
# org.bluez.GattDescriptor1
# ---------------------------------------------------------------------------

class GattDescriptor(ServiceInterface):
    """A GATT descriptor on a characteristic.

    We only need this for the Client Characteristic Configuration
    Descriptor (CCCD, UUID 0x2902) on CHAR_RESPONSE. BlueZ usually
    auto-creates the CCCD when a characteristic has the `notify` flag,
    so explicit registration here is belt-and-suspenders. Harmless if
    BlueZ ignores it.
    """

    def __init__(self, uuid: str, flags: list[str], char_path: str):
        super().__init__("org.bluez.GattDescriptor1")
        self._uuid = uuid
        self._flags = flags
        self._char_path = char_path
        self._value = bytearray([0x00, 0x00])  # CCCD: notifications disabled

    @dbus_property(access=PropertyAccess.READ)
    def UUID(self) -> "s":  # noqa: N802
        return self._uuid

    @dbus_property(access=PropertyAccess.READ)
    def Characteristic(self) -> "o":  # noqa: N802
        return self._char_path

    @dbus_property(access=PropertyAccess.READ)
    def Flags(self) -> "as":  # noqa: N802
        return self._flags

    @dbus_property(access=PropertyAccess.READ)
    def Value(self) -> "ay":  # noqa: N802
        return bytes(self._value)

    @method()
    def ReadValue(self, options: "a{sv}") -> "ay":  # noqa: N802
        return bytes(self._value)

    @method()
    def WriteValue(self, value: "ay", options: "a{sv}"):  # noqa: N802
        # Central writing CCCD: 0x0001 = notifications, 0x0002 = indications.
        # BlueZ also fires StartNotify/StopNotify on the parent char so we
        # don't track this here.
        self._value = bytearray(value)


# ---------------------------------------------------------------------------
# org.bluez.LEAdvertisement1
# ---------------------------------------------------------------------------

class Advertisement(ServiceInterface):
    """The LE undirected advertising packet BlueZ emits on our behalf.

    BlueZ owns the radio scheduling; we just declare what should be in
    the advertising payload. ServiceUUIDs is the critical field — the
    central scans for our service UUID to discover us.
    """

    def __init__(
        self,
        local_name: str,
        service_uuids: list[str],
        adv_type: str = "peripheral",
    ):
        super().__init__("org.bluez.LEAdvertisement1")
        self._type = adv_type
        self._local_name = local_name
        self._service_uuids = service_uuids
        self._released = False

    @dbus_property(access=PropertyAccess.READ)
    def Type(self) -> "s":  # noqa: N802
        return self._type

    @dbus_property(access=PropertyAccess.READ)
    def LocalName(self) -> "s":  # noqa: N802
        return self._local_name

    @dbus_property(access=PropertyAccess.READ)
    def ServiceUUIDs(self) -> "as":  # noqa: N802
        return self._service_uuids

    @method()
    def Release(self):  # noqa: N802
        # BlueZ calls Release when it has unregistered the advertisement
        # (most often after a central connects, since LE peripherals
        # can't advertise during a connection). We re-register from
        # BleServer when the central disconnects.
        self._released = True
        log.info("advertisement released by BlueZ (central connected, or unregister)")

    @property
    def released(self) -> bool:
        """Has BlueZ called Release() on us? Set on connect, cleared on
        re-register."""
        return self._released

    def reset_released(self) -> None:
        self._released = False


# ---------------------------------------------------------------------------
# org.freedesktop.DBus.ObjectManager — root of the GATT application tree
# ---------------------------------------------------------------------------

class GattApplication(ServiceInterface):
    """Root object exposing GetManagedObjects to BlueZ.

    BlueZ's GattManager1.RegisterApplication takes an object path; it
    then calls org.freedesktop.DBus.ObjectManager.GetManagedObjects on
    that path to enumerate services / characteristics / descriptors.
    Our implementation walks the registered children and returns the
    interface + property snapshot BlueZ expects.
    """

    def __init__(self):
        super().__init__("org.freedesktop.DBus.ObjectManager")
        # path → {iface_name: ServiceInterface instance}
        self._objects: dict[str, dict[str, ServiceInterface]] = {}

    def add(self, path: str, iface: ServiceInterface) -> None:
        """Register an exported object so it shows up in GetManagedObjects."""
        self._objects.setdefault(path, {})[iface.name] = iface

    @method()
    def GetManagedObjects(self) -> "a{oa{sa{sv}}}":  # noqa: N802
        out: dict[str, dict[str, dict[str, Variant]]] = {}
        for path, ifaces in self._objects.items():
            ifaces_dict: dict[str, dict[str, Variant]] = {}
            for iface_name, iface in ifaces.items():
                # Pull the current property values for every dbus_property
                # the ServiceInterface class declares. dbus-fast stores the
                # property descriptors on the class, not the instance.
                props: dict[str, Variant] = {}
                for prop_name, prop in _iface_properties(iface).items():
                    try:
                        value = prop.prop_getter(iface)
                    except Exception:
                        log.exception("get %s.%s failed", iface_name, prop_name)
                        continue
                    props[prop_name] = Variant(prop.signature, value)
                ifaces_dict[iface_name] = props
            out[path] = ifaces_dict
        return out


# ---------------------------------------------------------------------------
# Helper: introspect a ServiceInterface's declared dbus_property descriptors
# ---------------------------------------------------------------------------

def _iface_properties(iface: ServiceInterface) -> dict[str, Any]:
    """Pull the property descriptors dbus-fast attached to the class.

    dbus-fast stores DBusProperty descriptors as class attributes named
    after the original method (e.g., `UUID`). We walk the MRO so that
    properties inherited from a base ServiceInterface subclass also
    surface.
    """
    from dbus_fast.service import DBusProperty
    out: dict[str, Any] = {}
    for klass in type(iface).__mro__:
        for name, attr in vars(klass).items():
            if isinstance(attr, DBusProperty) and name not in out:
                out[name] = attr
    return out
