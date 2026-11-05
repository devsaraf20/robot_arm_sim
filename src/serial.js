/**
 * serial.js — a live USB connection to the real arm using the browser's Web Serial API.
 *
 * Web Serial lets a web page talk to a USB device (like an Arduino) after the user
 * picks it from a list. It only works:
 *   • in Chrome, Edge or Opera on desktop (not Safari/Firefox yet),
 *   • on a secure page: https://… (Vercel) or http://localhost (npm run dev).
 *
 * Data flows through STREAMS — think of them as pipes:
 *   port.readable  →  TextDecoderStream (bytes → text)  →  our line splitter  →  onMessage
 *   cmd text       →  TextEncoder (text → bytes)        →  port.writable
 */
import { BAUD_RATE, cmd, parseLine, createLineSplitter } from './protocol.js';

export const serialSupported = () => typeof navigator !== 'undefined' && 'serial' in navigator;

export function createSerialLink({ onMessage = () => {}, onStatus = () => {}, onLine = () => {} } = {}) {
  let port = null;
  let reader = null;
  let writer = null;
  let readLoopDone = null;
  const encoder = new TextEncoder();

  let lastSent = '';
  let lastSentAt = 0;

  async function connect() {
    if (!serialSupported()) throw new Error('Web Serial is not supported in this browser. Use Chrome or Edge on desktop.');
    // Opens the browser's device picker. Must be called from a click (user gesture).
    port = await navigator.serial.requestPort();
    await port.open({ baudRate: BAUD_RATE });
    writer = port.writable.getWriter();
    onStatus('connected');
    readLoopDone = readLoop();
    // Many Arduinos reboot when the port opens; ask "who are you?" a few times.
    for (const delay of [100, 1200, 2500]) setTimeout(() => port && send(cmd.hello()), delay);
  }

  async function readLoop() {
    const split = createLineSplitter();
    const decoder = new TextDecoderStream();
    const pipeDone = port.readable.pipeTo(decoder.writable).catch(() => {});
    reader = decoder.readable.getReader();
    try {
      for (;;) {
        const { value, done } = await reader.read();
        if (done) break;
        for (const line of split(value)) {
          onLine('←', line);
          const msg = parseLine(line);
          if (msg) onMessage(msg);
        }
      }
    } catch (err) {
      onStatus('error', err.message); // e.g. the cable was unplugged
    } finally {
      reader.releaseLock();
      await pipeDone;
    }
  }

  async function disconnect() {
    if (!port) return;
    const p = port;
    port = null;
    try {
      await reader?.cancel();
      await readLoopDone;
      writer?.releaseLock();
      await p.close();
    } catch { /* already gone */ }
    reader = writer = null;
    onStatus('disconnected');
  }

  /** Send one command line. Returns false if not connected. */
  function send(line) {
    if (!writer) return false;
    // Log everything except the high-rate T stream, to keep the console readable.
    if (!line.startsWith('T ')) onLine('→', line);
    writer.write(encoder.encode(line + '\n')).catch((err) => onStatus('error', err.message));
    return true;
  }

  /**
   * Stream joint targets, but politely: at most `hz` times per second and only when
   * something changed. An Arduino Uno's serial buffer is only 64 bytes, so flooding
   * it makes motion jerky. An unchanged pose is still re-sent every 250 ms as a
   * KEEPALIVE: the firmware holds still if it hears nothing for 1 s (its watchdog).
   */
  function sendTarget(servoDeg, hz = 30) {
    const line = cmd.target(servoDeg);
    const now = performance.now();
    if (now - lastSentAt < 1000 / hz) return;
    if (line === lastSent && now - lastSentAt < 250) return;
    lastSent = line;
    lastSentAt = now;
    send(line);
  }

  // If the device is unplugged, the browser fires 'disconnect' on navigator.serial.
  if (serialSupported()) {
    navigator.serial.addEventListener('disconnect', (e) => {
      if (e.target === port) {
        port = null;
        reader = writer = null;
        onStatus('disconnected', 'device unplugged');
      }
    });
  }

  return {
    connect,
    disconnect,
    send,
    sendTarget,
    get connected() { return !!writer; },
  };
}
