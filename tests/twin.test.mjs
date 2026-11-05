import { test } from 'node:test';
import assert from 'node:assert/strict';
import { stepTowards, createSafety, maxDiffDeg, HOMING_SPEED } from '../src/safety.js';
import { parseLine, createLineSplitter, cmd } from '../src/protocol.js';
import { HOME, LIMITS, simToServo, servoToSim } from '../src/ik.js';
import { unrealToSim, simToUnreal } from '../src/unreal-link.js';
import { encode, decode } from '../bridge/osc.mjs';

const DEG = Math.PI / 180;

test('speed limit caps each joint step', () => {
  const goal = HOME.map((v, i) => (i === 0 ? LIMITS[0][1] : v));
  let q = HOME.slice();
  const dt = 1 / 60;
  for (let i = 0; i < 60; i++) q = stepTowards(q, goal, dt, 45, 0);
  // 1 second at 45°/s → exactly 45° of travel, no more
  assert.ok(Math.abs((q[0] - HOME[0]) / DEG - 45) < 1e-6);
});

test('stepTowards never leaves joint limits', () => {
  const q = stepTowards(HOME, [10, 10, 10, 10], 1, 10000, 0);
  q.forEach((v, i) => assert.ok(v >= LIMITS[i][0] && v <= LIMITS[i][1]));
});

test('e-stop latches and blocks homing until reset', () => {
  const s = createSafety();
  s.estop();
  assert.equal(s.acceptsInput, false);
  assert.equal(s.startHoming(), false);
  s.reset();
  assert.equal(s.acceptsInput, true);
  assert.equal(s.startHoming(), true);
  assert.equal(s.effectiveSpeed, HOMING_SPEED);
  s.updateHoming(HOME);
  assert.equal(s.state, 'normal');
});

test('servo mapping round-trips', () => {
  const servo = simToServo(HOME);
  assert.deepEqual(servo.map(Math.round), [90, 70, 50, 45]);
  assert.ok(maxDiffDeg(servoToSim(servo), HOME) < 1e-9);
});

test('protocol parsing and chunked lines', () => {
  const split = createLineSplitter();
  assert.deepEqual(split('P 90.0 7'), []);
  const lines = split('0.0 50.0 45.0\r\nS HOMED\nHEL');
  assert.deepEqual(parseLine(lines[0]), { type: 'position', angles: [90, 70, 50, 45] });
  assert.deepEqual(parseLine(lines[1]), { type: 'state', state: 'HOMED' });
  assert.equal(split('LO robot-arm-twin 1.0 4 1\n').length, 1);
  assert.equal(parseLine('HELLO robot-arm-twin 1.0 4 1').feedback, true);
  assert.equal(parseLine('P 1 nope 3').type, 'log');
  assert.equal(cmd.target([90, 70.04, 50, 45]), 'T 90.0 70.0 50.0 45.0');
});

test('OSC encode/decode round trip, including bundles', () => {
  const msg = encode('/arm/target', [12.5, -3, 40, 'hi', true]);
  assert.equal(msg.length % 4, 0);
  const [m] = decode(msg);
  assert.equal(m.address, '/arm/target');
  assert.deepEqual(m.args, [12.5, -3, 40, 'hi', true]);
  const head = Buffer.alloc(16);
  head.write('#bundle');
  const size = Buffer.alloc(4);
  size.writeInt32BE(msg.length);
  const bundle = Buffer.concat([head, size, msg, size, msg]);
  assert.equal(decode(bundle).length, 2);
  assert.deepEqual(decode(Buffer.from('garbage')), []);
});

test('Unreal <-> sim coordinates', () => {
  // 100 cm forward, 20 cm right, 50 cm up
  const s = unrealToSim([100, 20, 50]);
  assert.deepEqual(s, { x: 10, y: 5, z: 2 });
  assert.deepEqual(simToUnreal(s), [100, 20, 50]);
});
