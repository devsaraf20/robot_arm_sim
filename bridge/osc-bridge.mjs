#!/usr/bin/env node
/**
 * osc-bridge.mjs — connects Unreal Engine (OSC over UDP) to the browser (WebSocket).
 *
 * WHY A BRIDGE?  Web pages are not allowed to open raw UDP sockets, and OSC uses UDP.
 * So this small Node.js program sits in the middle and translates:
 *
 *   Unreal ──OSC/UDP :8000──▶ bridge ──WebSocket :8080 (JSON)──▶ browser simulator ──USB──▶ real arm
 *   Unreal ◀──OSC/UDP :8001── bridge ◀──WebSocket (JSON)──────── browser (arm state)
 *
 * Run:   npm run bridge
 * Options (all optional):
 *   --osc-port 8000          UDP port we LISTEN on for OSC from Unreal
 *   --osc-host 127.0.0.1     address to listen on (0.0.0.0 = accept from other PCs)
 *   --reply-host 127.0.0.1   where to SEND arm state back (the Unreal machine)
 *   --reply-port 8001        port Unreal's OSC server listens on
 *   --ws-port 8080           WebSocket port the browser connects to
 *   --allow-origin URL       also accept browser connections from this site
 *                            (e.g. https://your-app.vercel.app). localhost is always allowed.
 */
import dgram from 'node:dgram';
import { WebSocketServer } from 'ws';
import { decode, encode } from './osc.mjs';

const args = Object.fromEntries(
  process.argv.slice(2).reduce((acc, a, i, all) => (a.startsWith('--') ? [...acc, [a.slice(2), all[i + 1]]] : acc), [])
);
const cfg = {
  oscPort: Number(args['osc-port'] ?? 8000),
  oscHost: args['osc-host'] ?? '127.0.0.1',
  replyHost: args['reply-host'] ?? '127.0.0.1',
  replyPort: Number(args['reply-port'] ?? 8001),
  wsPort: Number(args['ws-port'] ?? 8080),
  allowOrigins: [args['allow-origin']].filter(Boolean),
};

// SECURITY: any web page you visit could try to open ws://localhost:8080 and move
// your robot. We only accept pages served from this computer (or a site you name).
function originAllowed(origin) {
  if (!origin) return true; // non-browser clients (scripts, tests)
  try {
    const { hostname } = new URL(origin);
    if (['localhost', '127.0.0.1', '[::1]'].includes(hostname)) return true;
  } catch { return false; }
  return cfg.allowOrigins.includes(origin);
}

const wss = new WebSocketServer({ port: cfg.wsPort, host: '127.0.0.1', verifyClient: ({ origin }) => originAllowed(origin) });
const udp = dgram.createSocket('udp4');
let fromUnreal = 0;
let toUnreal = 0;

// Unreal → browser
udp.on('message', (packet) => {
  let messages;
  try { messages = decode(packet); } catch { return; }
  for (const m of messages) {
    fromUnreal++;
    const json = JSON.stringify(m);
    for (const client of wss.clients) if (client.readyState === 1) client.send(json);
  }
});
udp.on('error', (err) => {
  console.error(`UDP error: ${err.message}`);
  process.exit(1);
});
udp.bind(cfg.oscPort, cfg.oscHost, () =>
  console.log(`OSC  in : udp://${cfg.oscHost}:${cfg.oscPort}   (point Unreal's OSC client here)`)
);

// browser → Unreal
wss.on('connection', (ws, req) => {
  console.log(`Browser connected (${req.headers.origin ?? 'no origin'})`);
  ws.on('message', (data) => {
    let msg;
    try { msg = JSON.parse(data); } catch { return; }
    if (typeof msg?.address !== 'string' || !msg.address.startsWith('/')) return;
    const packet = encode(msg.address, Array.isArray(msg.args) ? msg.args : []);
    udp.send(packet, cfg.replyPort, cfg.replyHost);
    toUnreal++;
  });
  ws.on('close', () => console.log('Browser disconnected'));
});
wss.on('listening', () => {
  console.log(`OSC  out: udp://${cfg.replyHost}:${cfg.replyPort}   (Unreal's OSC server listens here)`);
  console.log(`WebSocket: ws://localhost:${cfg.wsPort}            (the simulator connects here)`);
});

setInterval(() => {
  if (fromUnreal || toUnreal) {
    process.stdout.write(`\r${fromUnreal} msgs from Unreal · ${toUnreal} msgs to Unreal · ${wss.clients.size} browser(s)   `);
  }
}, 1000).unref();
