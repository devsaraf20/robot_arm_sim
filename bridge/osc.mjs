/**
 * osc.mjs — a tiny OSC (Open Sound Control) encoder/decoder, written from scratch.
 *
 * OSC is a simple binary message format used by Unreal, TouchDesigner, Max/MSP,
 * lighting desks… A message is:
 *
 *   address   "/arm/target"   → a string, zero-padded to a multiple of 4 bytes
 *   type tags ",fff"          → one letter per argument (f=float32, i=int32, s=string…)
 *   arguments 3 × float32     → big-endian binary numbers
 *
 * A "bundle" (starts with "#bundle") wraps several messages with a timestamp.
 * Unreal can send either, so we decode both.
 */

const pad4 = (n) => (n + 3) & ~3;

function readString(buf, offset) {
  let end = offset;
  while (end < buf.length && buf[end] !== 0) end++;
  return { value: buf.toString('utf8', offset, end), next: pad4(end + 1) };
}

function writeString(str) {
  const bytes = Buffer.from(str, 'utf8');
  const out = Buffer.alloc(pad4(bytes.length + 1)); // +1 for the terminating zero
  bytes.copy(out);
  return out;
}

/** Decode a UDP packet into a flat list of { address, args } messages. */
export function decode(buf) {
  if (buf.length >= 8 && buf.toString('utf8', 0, 7) === '#bundle') {
    const out = [];
    let offset = 16; // "#bundle\0" (8) + timetag (8)
    while (offset + 4 <= buf.length) {
      const size = buf.readInt32BE(offset);
      offset += 4;
      if (size <= 0 || offset + size > buf.length) break;
      out.push(...decode(buf.subarray(offset, offset + size)));
      offset += size;
    }
    return out;
  }

  const addr = readString(buf, 0);
  if (!addr.value.startsWith('/')) return [];
  if (addr.next >= buf.length) return [{ address: addr.value, args: [] }];
  const tags = readString(buf, addr.next);
  if (!tags.value.startsWith(',')) return [{ address: addr.value, args: [] }];

  const args = [];
  let offset = tags.next;
  for (const t of tags.value.slice(1)) {
    switch (t) {
      case 'f': args.push(buf.readFloatBE(offset)); offset += 4; break;
      case 'i': args.push(buf.readInt32BE(offset)); offset += 4; break;
      case 'd': args.push(buf.readDoubleBE(offset)); offset += 8; break;
      case 'h': args.push(Number(buf.readBigInt64BE(offset))); offset += 8; break;
      case 's': { const s = readString(buf, offset); args.push(s.value); offset = s.next; break; }
      case 'T': args.push(true); break;
      case 'F': args.push(false); break;
      case 'N': args.push(null); break;
      default: return [{ address: addr.value, args }]; // unknown type: stop safely
    }
  }
  return [{ address: addr.value, args }];
}

/** Encode one message. Numbers become float32 (what Unreal's "Add Float" expects). */
export function encode(address, args = []) {
  let tags = ',';
  const parts = [];
  for (const a of args) {
    if (typeof a === 'number') {
      tags += 'f';
      const b = Buffer.alloc(4);
      b.writeFloatBE(a);
      parts.push(b);
    } else if (typeof a === 'boolean') {
      tags += a ? 'T' : 'F';
    } else {
      tags += 's';
      parts.push(writeString(String(a)));
    }
  }
  return Buffer.concat([writeString(address), writeString(tags), ...parts]);
}
