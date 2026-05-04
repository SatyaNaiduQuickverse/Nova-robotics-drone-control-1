"""BLE GATT RPC wire format.

Mirrors `novabridge/ble/rpc.py` on the ground side. Layout is fixed by
drone-handoff PROMPT.md §2 Deliverable 3 and must NOT be reordered:

    Request  (CHAR_REQUEST, 10-byte header):
      req_id    u16_le
      flags     u8         (bit 0 EOM)
      method    u8         (1 GET, 2 POST, 3 DELETE, 4 PUT)
      path_len  u16_le
      body_len  u32_le
      [path  utf-8  path_len  bytes]
      [body  raw    body_len  bytes]

    Response (CHAR_RESPONSE, 10-byte header):
      req_id    u16_le
      status    u16_le
      flags     u8         (bit 0 EOM, bit 1 is_error)
      reserved  u8
      body_len  u32_le     (size of THIS fragment's body slice; receiver
                            accumulates slices keyed by req_id until EOM)
      [body  raw  body_len bytes]
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Iterator, Optional


# --- Wire constants -------------------------------------------------------

REQ_HDR = "<HBBHI"   # req_id, flags, method, path_len, body_len  → 10 B
RSP_HDR = "<HHBBI"   # req_id, status, flags, reserved, body_len  → 10 B
REQ_HDR_SIZE = struct.calcsize(REQ_HDR)
RSP_HDR_SIZE = struct.calcsize(RSP_HDR)
assert REQ_HDR_SIZE == 10 and RSP_HDR_SIZE == 10

METHOD_GET, METHOD_POST, METHOD_DELETE, METHOD_PUT = 1, 2, 3, 4
METHOD_NAME = {1: "GET", 2: "POST", 3: "DELETE", 4: "PUT"}

FLAG_EOM   = 0x01
FLAG_ERROR = 0x02

# GATT MTU after negotiation is typically 247 B; usable payload (after
# 3-byte ATT opcode + handle overhead) is ~244 B. We size fragments
# conservatively so they fit even when MTU isn't negotiated up.
DEFAULT_MTU         = 247
ATT_OVERHEAD_BYTES  = 3      # 1 opcode + 2 handle


# --- Decode / encode primitives -------------------------------------------

@dataclass
class RpcRequest:
    req_id:  int
    flags:   int
    method:  int
    path:    str
    body:    bytes

    @classmethod
    def decode(cls, buf: bytes) -> "RpcRequest":
        if len(buf) < REQ_HDR_SIZE:
            raise ValueError(f"request too short ({len(buf)} B)")
        req_id, flags, method, path_len, body_len = struct.unpack_from(
            REQ_HDR, buf, 0,
        )
        expected = REQ_HDR_SIZE + path_len + body_len
        if len(buf) < expected:
            raise ValueError(
                f"request truncated: header says {expected} B, got {len(buf)} B"
            )
        path  = buf[REQ_HDR_SIZE : REQ_HDR_SIZE + path_len].decode("utf-8")
        body  = bytes(buf[REQ_HDR_SIZE + path_len :
                          REQ_HDR_SIZE + path_len + body_len])
        return cls(req_id=req_id, flags=flags, method=method,
                   path=path, body=body)

    def encode(self) -> bytes:
        """Serialize as `[REQ_HDR][path utf-8][body]`. Default flags
        include EOM=1 unless the caller cleared it (i.e., this is a
        non-final fragment produced by `fragment()`)."""
        path_b = self.path.encode("utf-8")
        # If no flags set explicitly, treat this as a self-contained
        # request (EOM=1). Multi-fragment encoders override this.
        flags = self.flags if self.flags else FLAG_EOM
        hdr = struct.pack(REQ_HDR, self.req_id, flags, self.method,
                          len(path_b), len(self.body))
        return hdr + path_b + self.body

    def fragment(self, mtu: int = DEFAULT_MTU) -> Iterator[bytes]:
        """Yield one or more BLE-write payloads that together carry this
        request.

        Symmetric with `RpcResponse.fragment`. Each fragment is itself
        a complete, valid RpcRequest (decodable by `decode()`):
          * same req_id, method, and path repeated in every fragment
            (path is small — typically < 32 B — so the duplication is
            negligible vs the body slice it accompanies)
          * body split across fragments; each fragment's body field
            carries that fragment's slice
          * EOM (flag bit 0) cleared on all but the last fragment

        The receiver feeds each fragment into a `RequestReassembler`
        which accumulates body slices keyed by req_id and emits the
        complete RpcRequest when EOM arrives. Single-fragment requests
        (most calls) round-trip in one feed.
        """
        path_b = self.path.encode("utf-8")
        per_fragment_overhead = REQ_HDR_SIZE + len(path_b) + ATT_OVERHEAD_BYTES
        max_body = max(1, mtu - per_fragment_overhead)

        if len(self.body) <= max_body:
            # Single fragment — straight RpcRequest with EOM set.
            hdr = struct.pack(REQ_HDR, self.req_id, FLAG_EOM, self.method,
                              len(path_b), len(self.body))
            yield hdr + path_b + self.body
            return

        offset = 0
        n = len(self.body)
        while offset < n:
            chunk = self.body[offset : offset + max_body]
            offset += len(chunk)
            eom = offset >= n
            flags = FLAG_EOM if eom else 0
            hdr = struct.pack(REQ_HDR, self.req_id, flags, self.method,
                              len(path_b), len(chunk))
            yield hdr + path_b + chunk

    @property
    def method_name(self) -> str:
        return METHOD_NAME.get(self.method, f"M{self.method}")


@dataclass
class RpcResponse:
    req_id:   int
    status:   int
    body:     bytes = b""
    is_error: bool  = False

    def _flags(self, eom: bool) -> int:
        f = 0
        if eom:           f |= FLAG_EOM
        if self.is_error: f |= FLAG_ERROR
        return f

    def encode_single(self) -> bytes:
        """Encode as a single non-fragmented notification (EOM set)."""
        hdr = struct.pack(RSP_HDR, self.req_id, self.status,
                          self._flags(eom=True), 0, len(self.body))
        return hdr + self.body

    def fragment(self, mtu: int = DEFAULT_MTU) -> Iterator[bytes]:
        """Yield one or more notification payloads that together carry
        this response. Each fragment has its own 10-byte header; the
        body_len field carries the size of THAT fragment's body slice.
        EOM is set only on the last fragment.

        Slicing is byte-deterministic — fragments are produced in order
        and the receiver concatenates body slices in arrival order.
        """
        max_payload = max(1, mtu - ATT_OVERHEAD_BYTES - RSP_HDR_SIZE)
        if not self.body:
            # Header-only response (e.g., 204 No Content). Single empty fragment.
            yield struct.pack(RSP_HDR, self.req_id, self.status,
                              self._flags(eom=True), 0, 0)
            return

        offset = 0
        n = len(self.body)
        while offset < n:
            chunk = self.body[offset : offset + max_payload]
            offset += len(chunk)
            eom = offset >= n
            hdr = struct.pack(RSP_HDR, self.req_id, self.status,
                              self._flags(eom=eom), 0, len(chunk))
            yield hdr + chunk


# --- Reassembly (used by self-tests; the phone has its own) --------------

class ResponseReassembler:
    """Accumulates fragmented response notifications keyed by req_id.

    Returns a fully-decoded RpcResponse when the EOM fragment arrives.
    Designed to match the encoder above byte-for-byte — each incoming
    fragment is `[10-byte header][body slice]` and the assembled body
    is the concatenation of slices.
    """

    def __init__(self):
        self._buffers: dict[int, bytearray] = {}
        self._heads:   dict[int, tuple[int, int, bool]] = {}   # req_id → (status, flags, is_error)

    def feed(self, payload: bytes):
        if len(payload) < RSP_HDR_SIZE:
            raise ValueError(f"fragment too short ({len(payload)} B)")
        req_id, status, flags, _resv, body_len = struct.unpack_from(
            RSP_HDR, payload, 0,
        )
        slice_ = payload[RSP_HDR_SIZE : RSP_HDR_SIZE + body_len]

        buf = self._buffers.setdefault(req_id, bytearray())
        buf.extend(slice_)
        self._heads[req_id] = (status, flags, bool(flags & FLAG_ERROR))

        if flags & FLAG_EOM:
            body = bytes(buf)
            status, _flags, is_error = self._heads[req_id]
            del self._buffers[req_id]
            del self._heads[req_id]
            return RpcResponse(req_id=req_id, status=status,
                               body=body, is_error=is_error)
        return None


# ---------------------------------------------------------------------------
# Reassembly for incoming fragmented requests (the drone-side mirror of
# what the central does for responses). Each incoming write from the
# central is itself a valid RpcRequest; non-final fragments share the
# req_id, method, and path of the request being assembled, with EOM=0
# until the last fragment.
# ---------------------------------------------------------------------------

class RequestReassembler:
    """Accumulates fragmented incoming RpcRequest writes keyed by req_id.

    Returns the fully-decoded RpcRequest when the EOM fragment arrives.
    Single-fragment writes (most calls — anything that fits in MTU - 3
    bytes) round-trip in one feed because the encoder sets EOM=1 on the
    sole fragment.

    Cleanup: call `drop(req_id)` to release a partial buffer when a
    central disconnects mid-fragment, or `clear()` to flush all state
    on full disconnect. Without this, stale buffers leak across
    reconnects with the same req_id.
    """

    def __init__(self):
        # req_id → accumulated body so far
        self._bodies: dict[int, bytearray] = {}
        # req_id → (method, path) captured on the first fragment
        self._heads:  dict[int, tuple[int, str]] = {}

    def feed(self, payload: bytes) -> Optional["RpcRequest"]:
        # Each fragment is a valid RpcRequest packet — decode and pull
        # out the body slice + per-fragment header.
        sub = RpcRequest.decode(payload)
        body_buf = self._bodies.setdefault(sub.req_id, bytearray())
        body_buf.extend(sub.body)
        # First fragment for this req_id locks the method and path.
        # We trust subsequent fragments to repeat the same values; we
        # don't re-validate to keep this hot path cheap.
        if sub.req_id not in self._heads:
            self._heads[sub.req_id] = (sub.method, sub.path)

        if sub.flags & FLAG_EOM:
            method, path = self._heads.pop(sub.req_id)
            body = bytes(self._bodies.pop(sub.req_id))
            return RpcRequest(
                req_id=sub.req_id,
                flags=sub.flags,
                method=method,
                path=path,
                body=body,
            )
        return None

    def drop(self, req_id: int) -> None:
        """Release any in-flight buffer for one req_id."""
        self._bodies.pop(req_id, None)
        self._heads.pop(req_id, None)

    def clear(self) -> None:
        """Flush all in-flight buffers — call on central disconnect."""
        self._bodies.clear()
        self._heads.clear()

    @property
    def pending(self) -> int:
        """Number of req_ids with partially-assembled bodies."""
        return len(self._bodies)
