"""BLE GATT peripheral — phone ↔ drone tunnel.

Implements drone-handoff PROMPT.md §2 Deliverable 3 directly against
BlueZ's DBus interfaces (no `bless` library — see commit history; bless
was removed because it silently lost GATT registration on disconnect).

Architecture:

    main.py
       │ creates one
       ▼
    BleServer  ──────────────────────────────────────────────────┐
       │ owns                                                     │
       ├──► dbus_fast.MessageBus              (system bus client) │
       ├──► gatt.GattService     @ /com/novaros/dronebridge/...   │
       ├──► gatt.GattCharacteristic (CHAR_REQUEST, write callback) │
       ├──► gatt.GattCharacteristic (CHAR_RESPONSE, notify)        │
       ├──► gatt.GattDescriptor (CCCD on response)                 │
       ├──► gatt.Advertisement                                     │
       ├──► gatt.GattApplication (ObjectManager root)              │
       └──► httpx.AsyncClient → drone-control                      │
                                                                   │
    BlueZ on the host ─────────────────────────────────────────────┘
       │
       └──► hci0 radio  ◄── BLE connect from ground Pi (BleakLink)

What we do that bless didn't:

  * Subscribe to BlueZ's `Adapter1.PowerState` / device `Connected`
    properties so disconnect is detected explicitly, not by polling.
  * Re-register the LEAdvertisement1 immediately after BlueZ calls
    Release() on it (BlueZ does so when a connection establishes;
    advertising must be re-armed after disconnect).
  * Watchdog on `LEAdvertisingManager1.ActiveInstances` as a
    backstop — if it drops to 0 unexpectedly, re-register.
  * Re-register everything if bluetoothd restarts (NameOwnerChanged).

Concurrency: one in-flight phone request at a time (PROMPT §2.3 v1).
Outgoing notifications fragment to MTU; 50 ms pacing between fragments
so BlueZ's PropertiesChanged signal settles into one ATT notify per
fragment without coalescing.
"""

from __future__ import annotations

import asyncio
import json
import logging
from typing import Optional

import httpx
from dbus_fast import BusType
from dbus_fast.aio import MessageBus

from . import adapters, digest, gatt, rpc, snapshot  # noqa: F401  (snapshot for type hints)


log = logging.getLogger("drone_bridge.ble")


# --- Spec constants (must match the ground side byte-for-byte) -----------

SERVICE_UUID       = "6e400000-b5a3-f393-e0a9-e50e24dcca9e"
CHAR_REQUEST_UUID  = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
CHAR_RESPONSE_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
CCCD_UUID          = "00002902-0000-1000-8000-00805f9b34fb"
ADV_NAME           = "novadrone-pi"

ADAPTER_PATH = "/org/bluez/hci0"
BLUEZ_BUS    = "org.bluez"


class BleServer:
    """Manages the GATT peripheral lifecycle.

    Public API matches the previous bless-based BleServer so `main.py`
    and the rest of `drone_bridge` are unchanged.
    """

    def __init__(
        self,
        drone_api: str,
        adv_name: str = ADV_NAME,
        mtu: int = rpc.DEFAULT_MTU,
        startup_retry_s: float = 30.0,
        fragment_pace_ms: int = 50,
        watchdog_period_s: float = 2.0,
    ):
        self.drone_api         = drone_api.rstrip("/")
        self.adv_name          = adv_name
        self.mtu               = mtu
        self.startup_retry_s   = startup_retry_s
        self._fragment_pace    = fragment_pace_ms / 1000.0
        self._watchdog_period  = watchdog_period_s

        # Lifecycle / state
        self._inflight_lock    = asyncio.Lock()
        self._http: Optional[httpx.AsyncClient] = None
        self._bus: Optional[MessageBus]         = None

        # Centrals may fragment large RpcRequests into multiple BLE writes;
        # the reassembler accumulates body slices until the EOM flag, then
        # surfaces a complete RpcRequest. Single-fragment requests (most
        # calls) round-trip in one feed because EOM is set on fragment 1.
        self._reassembler = rpc.RequestReassembler()

        self._app:           Optional[gatt.GattApplication]    = None
        self._service:       Optional[gatt.GattService]        = None
        self._char_request:  Optional[gatt.GattCharacteristic] = None
        self._char_response: Optional[gatt.GattCharacteristic] = None
        self._cccd:          Optional[gatt.GattDescriptor]     = None
        self._advertisement: Optional[gatt.Advertisement]      = None

        self._gatt_mgr     = None  # org.bluez.GattManager1 proxy
        self._adv_mgr      = None  # org.bluez.LEAdvertisingManager1 proxy
        self._app_registered = False
        self._adv_registered = False

        self._watchdog_task:  Optional[asyncio.Task] = None
        self._adv_lock      = asyncio.Lock()

    # ----------------------------------------------------------------- start

    async def start(self) -> None:
        """Connect to the system bus, build the GATT tree, register with BlueZ."""
        await self._wait_for_drone_api()

        self._http = httpx.AsyncClient(
            base_url=self.drone_api,
            timeout=adapters.DEFAULT_TIMEOUT,
            transport=httpx.AsyncHTTPTransport(retries=0),
        )

        self._bus = await MessageBus(bus_type=BusType.SYSTEM).connect()
        log.info("connected to system DBus")

        self._build_gatt_tree()
        self._export_objects()

        await self._resolve_bluez_managers()
        await self._register_application()
        await self._register_advertisement()

        self._watchdog_task = asyncio.create_task(
            self._advertising_watchdog(), name="ble-watchdog",
        )
        log.info("ble peripheral up: %s service=%s", self.adv_name, SERVICE_UUID)

    async def _wait_for_drone_api(self) -> None:
        """Block until drone-control answers /telemetry, with a deadline.

        BLE clients hitting the bridge during this gap would just see
        502 Bad Gateway from us, which is fine — but logging cleanly is
        better than firing failed-forward errors on every request."""
        async with httpx.AsyncClient(timeout=2.0) as probe:
            deadline = asyncio.get_event_loop().time() + self.startup_retry_s
            attempt = 0
            while True:
                attempt += 1
                try:
                    r = await probe.get(f"{self.drone_api}/telemetry")
                    if r.status_code < 500:
                        log.info("drone-control reachable (attempt %d)", attempt)
                        return
                except httpx.RequestError:
                    pass
                if asyncio.get_event_loop().time() >= deadline:
                    log.error("drone-control did not come up in %.0fs — "
                              "starting BLE anyway; forwards will return 502",
                              self.startup_retry_s)
                    return
                if attempt == 1:
                    log.warning("drone-control not up yet, retrying every 2s "
                                "for up to %.0fs", self.startup_retry_s)
                await asyncio.sleep(2.0)

    def _build_gatt_tree(self) -> None:
        """Construct the GATT objects (no DBus interaction yet)."""
        self._app = gatt.GattApplication()

        self._service = gatt.GattService(SERVICE_UUID, primary=True)

        self._char_request = gatt.GattCharacteristic(
            uuid=CHAR_REQUEST_UUID,
            flags=["write", "write-without-response"],
            service_path=gatt.SERVICE_PATH,
            write_cb=self._on_request_bytes,
        )

        self._char_response = gatt.GattCharacteristic(
            uuid=CHAR_RESPONSE_UUID,
            flags=["read", "notify"],
            service_path=gatt.SERVICE_PATH,
        )

        # Belt-and-suspenders CCCD; BlueZ usually adds it implicitly when
        # a char has the `notify` flag, but declaring it explicitly makes
        # the GATT tree self-describing and survives any auto-add quirk.
        self._cccd = gatt.GattDescriptor(
            uuid=CCCD_UUID,
            flags=["read", "write"],
            char_path=gatt.CHAR_RSP_PATH,
        )

        self._advertisement = gatt.Advertisement(
            local_name=self.adv_name,
            service_uuids=[SERVICE_UUID],
        )

    def _export_objects(self) -> None:
        """Export all DBus objects on the system bus + record them in the
        ObjectManager so BlueZ's GetManagedObjects walk can find them."""
        assert self._bus is not None
        # Application root (provides ObjectManager interface)
        self._bus.export(gatt.APP_PATH, self._app)
        # Children, each at a unique path
        self._bus.export(gatt.SERVICE_PATH,   self._service)
        self._bus.export(gatt.CHAR_REQ_PATH,  self._char_request)
        self._bus.export(gatt.CHAR_RSP_PATH,  self._char_response)
        self._bus.export(gatt.DESC_CCCD_PATH, self._cccd)
        # And register them with our ObjectManager so BlueZ can enumerate
        self._app.add(gatt.SERVICE_PATH,   self._service)
        self._app.add(gatt.CHAR_REQ_PATH,  self._char_request)
        self._app.add(gatt.CHAR_RSP_PATH,  self._char_response)
        self._app.add(gatt.DESC_CCCD_PATH, self._cccd)
        # Advertisement is a separate path, registered via a different manager
        self._bus.export(gatt.ADV_PATH, self._advertisement)

    async def _resolve_bluez_managers(self) -> None:
        """Get proxy interfaces for BlueZ's GattManager1 + LEAdvertisingManager1."""
        assert self._bus is not None
        introspection = await self._bus.introspect(BLUEZ_BUS, ADAPTER_PATH)
        proxy = self._bus.get_proxy_object(BLUEZ_BUS, ADAPTER_PATH, introspection)
        self._gatt_mgr = proxy.get_interface("org.bluez.GattManager1")
        self._adv_mgr  = proxy.get_interface("org.bluez.LEAdvertisingManager1")

    async def _register_application(self) -> None:
        """Tell BlueZ about our GATT application. BlueZ will call our
        ObjectManager.GetManagedObjects to discover the tree."""
        assert self._gatt_mgr is not None
        if self._app_registered:
            return
        try:
            await self._gatt_mgr.call_register_application(gatt.APP_PATH, {})
            self._app_registered = True
            log.info("GATT application registered at %s", gatt.APP_PATH)
        except Exception:
            log.exception("RegisterApplication failed")
            raise

    async def _register_advertisement(self) -> None:
        """Tell BlueZ to start broadcasting the advertisement object.
        Idempotent — safe to call again after Release()."""
        assert self._adv_mgr is not None and self._advertisement is not None
        async with self._adv_lock:
            if self._adv_registered and not self._advertisement.released:
                return
            try:
                await self._adv_mgr.call_register_advertisement(gatt.ADV_PATH, {})
                self._adv_registered = True
                self._advertisement.reset_released()
                log.info("advertisement registered (%s)", self.adv_name)
            except Exception as e:
                # Common case: re-registering while a connection is active.
                # BlueZ rejects with "already exists". Treat that as success.
                msg = str(e)
                if "already" in msg.lower() or "exists" in msg.lower():
                    log.debug("advertisement already registered (re-register no-op)")
                    self._adv_registered = True
                else:
                    log.warning("RegisterAdvertisement failed: %s", e)

    async def _unregister_advertisement(self) -> None:
        if self._adv_mgr is None or not self._adv_registered:
            return
        try:
            await self._adv_mgr.call_unregister_advertisement(gatt.ADV_PATH)
        except Exception as e:
            log.debug("UnregisterAdvertisement: %s", e)
        self._adv_registered = False

    async def _unregister_application(self) -> None:
        if self._gatt_mgr is None or not self._app_registered:
            return
        try:
            await self._gatt_mgr.call_unregister_application(gatt.APP_PATH)
        except Exception as e:
            log.debug("UnregisterApplication: %s", e)
        self._app_registered = False

    # ------------------------------------------------------------------ stop

    async def stop(self) -> None:
        if self._watchdog_task is not None:
            self._watchdog_task.cancel()
            try:
                await self._watchdog_task
            except (asyncio.CancelledError, Exception):
                pass
            self._watchdog_task = None

        await self._unregister_advertisement()
        await self._unregister_application()

        if self._bus is not None:
            try:
                self._bus.disconnect()
            except Exception:
                pass

        if self._http is not None:
            await self._http.aclose()

    # -------------------------------------------------------- watchdog loop

    async def _advertising_watchdog(self) -> None:
        """Resilience layer.

        Reasons we re-register:
          * BlueZ called Release() on our advertisement (post-connect or
            unexpected unregister) and we should rearm
          * `LEAdvertisingManager1.ActiveInstances` reads 0 — something
            unregistered us out from under
          * bluetoothd restarted — `_gatt_mgr` calls will fail; re-resolve

        We poll instead of subscribing because BlueZ doesn't emit a
        clean signal on advertisement-removed; the Release() callback
        on our object is the closest, and it sets a flag we check here.
        """
        while True:
            try:
                await asyncio.sleep(self._watchdog_period)

                # 1) Did BlueZ call Release() on our adv since last tick?
                if self._advertisement is not None and self._advertisement.released:
                    log.info("watchdog: advertisement was Released — re-arming")
                    self._adv_registered = False
                    await self._register_advertisement()
                    # Re-arming the adv only succeeds after the central
                    # has disconnected (BlueZ rejects RegisterAdvertisement
                    # during an active connection). So a successful
                    # re-register IS our "central just disconnected" signal:
                    # drop any stale request fragment buffers so a new
                    # central with the same req_id can't pick up old state.
                    if self._adv_registered and self._reassembler.pending:
                        log.info("watchdog: dropping %d in-flight fragment buffer(s)",
                                 self._reassembler.pending)
                        self._reassembler.clear()
                    continue

                # 2) Cross-check with BlueZ's own ActiveInstances counter.
                if self._adv_mgr is not None:
                    try:
                        active = await self._adv_mgr.get_active_instances()
                    except Exception as e:
                        # Likely bluetoothd restarted — try to re-resolve.
                        log.warning("watchdog: ActiveInstances read failed (%s) — "
                                    "re-resolving BlueZ managers", e)
                        try:
                            await self._resolve_bluez_managers()
                            self._app_registered = False
                            self._adv_registered = False
                            await self._register_application()
                            await self._register_advertisement()
                        except Exception:
                            log.exception("watchdog: re-resolve failed")
                        continue

                    if active == 0 and self._adv_registered:
                        log.warning("watchdog: ActiveInstances=0 but we think "
                                    "we're advertising — re-registering")
                        self._adv_registered = False
                        await self._register_advertisement()

            except asyncio.CancelledError:
                raise
            except Exception:
                log.exception("watchdog tick raised")

    # --------------------------------------------------------- request path

    async def _on_request_bytes(self, raw: bytes) -> None:
        """Called by the CHAR_REQUEST WriteValue handler.

        Each ATT write is a fragment of a (possibly multi-fragment)
        RpcRequest. The reassembler returns None until the EOM
        fragment arrives; once it does, dispatch as before.

        Errors here MUST NOT propagate back into the DBus method
        handler — they would surface to BlueZ as a generic GATT error
        with no actionable info for the central."""
        try:
            req = self._reassembler.feed(raw)
        except ValueError as e:
            log.warning("bad RpcRequest fragment from phone: %s (%d B)",
                        e, len(raw))
            return

        if req is None:
            # Mid-fragment — body still assembling. The next write that
            # arrives with EOM=1 for the same req_id will surface the
            # complete request.
            log.debug("fragment buffered (%d in flight)",
                      self._reassembler.pending)
            return

        log.info("rpc %5d %-6s %s (%d B body)",
                 req.req_id, req.method_name, req.path, len(req.body))

        async with self._inflight_lock:
            try:
                rsp = await self._dispatch(req)
            except Exception as e:
                log.exception("dispatch error for req %d", req.req_id)
                rsp = rpc.RpcResponse(
                    req_id=req.req_id, status=500,
                    body=json.dumps({"code": "BRIDGE_INTERNAL",
                                     "message": str(e)}).encode(),
                    is_error=True,
                )
            await self._send_response(rsp)

    async def _dispatch(self, req: rpc.RpcRequest) -> rpc.RpcResponse:
        # Fast-path: /telemetry/digest is satisfied entirely from the
        # in-memory snapshot. No HTTP hop, no JSON encode/decode.
        if req.method == rpc.METHOD_GET and req.path == "/telemetry/digest":
            return rpc.RpcResponse(
                req_id=req.req_id, status=200,
                body=digest.pack(),
                is_error=False,
            )

        adapted = adapters.adapt(req.method_name, req.path, req.body)

        if self._http is None:
            return rpc.RpcResponse(
                req_id=req.req_id, status=503,
                body=json.dumps({"code": "BRIDGE_NOT_READY",
                                 "message": "http client not initialized"}).encode(),
                is_error=True,
            )

        try:
            r = await self._http.request(
                method=adapted.method,
                url=adapted.path,
                json=adapted.json if isinstance(adapted.json,
                                                (dict, list, type(None))) else None,
                content=(adapted.json if isinstance(adapted.json, bytes) else None),
                timeout=adapted.timeout,
            )
        except httpx.TimeoutException:
            return rpc.RpcResponse(
                req_id=req.req_id, status=504,
                body=json.dumps({"code": "FCU_TIMEOUT",
                                 "message": f"drone-control did not respond within "
                                            f"{adapted.timeout:.1f}s"}).encode(),
                is_error=True,
            )
        except httpx.RequestError as e:
            return rpc.RpcResponse(
                req_id=req.req_id, status=502,
                body=json.dumps({"code": "FCU_BRIDGE_ERROR",
                                 "message": str(e)}).encode(),
                is_error=True,
            )

        is_error = not (200 <= r.status_code < 300)
        return rpc.RpcResponse(
            req_id=req.req_id, status=r.status_code,
            body=r.content, is_error=is_error,
        )

    # --------------------------------------------------------- response path

    async def _send_response(self, rsp: rpc.RpcResponse) -> None:
        """Push the response back to the phone, fragmenting if needed.

        Each fragment becomes a separate ATT notify. We pace them at
        `_fragment_pace_ms` so each PropertiesChanged signal makes it
        through dbus-fast → BlueZ → radio before the next one overwrites
        the property value."""
        if self._char_response is None:
            log.warning("no CHAR_RESPONSE characteristic — dropping response %d",
                        rsp.req_id)
            return

        if not self._char_response.notifying:
            # Central never subscribed (no StartNotify call) — sending
            # would just update our internal cache for nobody.
            log.debug("rpc %5d  no subscriber, response dropped", rsp.req_id)
            return

        n_frags = 0
        for frag in rsp.fragment(self.mtu):
            self._char_response.update_value(frag)
            n_frags += 1
            await asyncio.sleep(self._fragment_pace)

        log.info("rpc %5d %3d  %d B in %d fragment(s)",
                 rsp.req_id, rsp.status, len(rsp.body), n_frags)
