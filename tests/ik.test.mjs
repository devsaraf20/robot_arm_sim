import { test } from 'node:test';
import assert from 'node:assert/strict';
import { solveIK, forwardKinematics, LIMITS, HOME, ARM, toRad } from '../src/ik.js';

const d = (a, b) => Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z);
const inLimits = (q) => q.every((v, i) => v >= LIMITS[i][0] - 1e-9 && v <= LIMITS[i][1] + 1e-9);

test('FK of a straight horizontal arm', () => {
  const t = forwardKinematics([0, 0, 0]).tip;
  assert.ok(Math.abs(t.x - (ARM.L1 + ARM.L2)) < 1e-9);
  assert.ok(Math.abs(t.y - ARM.BASE_HEIGHT) < 1e-9);
});

test('IK round-trips random reachable in-limit poses', () => {
  let seed = 7;
  const rnd = () => ((seed = (seed * 16807) % 2147483647) / 2147483647);
  for (let i = 0; i < 500; i++) {
    const q = LIMITS.map(([lo, hi]) => lo + rnd() * (hi - lo));
    const target = forwardKinematics(q).tip;
    const sol = solveIK(target, HOME);
    assert.ok(inLimits(sol.q), 'limits');
    assert.ok(d(forwardKinematics(sol.q).tip, target) < 1e-3, `err ${sol.error} at ${JSON.stringify(q)}`);
    assert.equal(sol.status, 'ok');
  }
});

test('out-of-reach target stretches toward it', () => {
  const sol = solveIK({ x: 10, y: ARM.BASE_HEIGHT, z: 0 }, HOME);
  assert.equal(sol.status, 'out-of-reach');
  const tip = forwardKinematics(sol.q).tip;
  assert.ok(Math.abs(tip.x - (ARM.L1 + ARM.L2)) < 1e-2);
});

test('target behind the robot is reached over the shoulder', () => {
  const target = { x: -1, y: 2.6, z: 0.2 };
  const sol = solveIK(target, HOME);
  assert.ok(inLimits(sol.q));
  assert.ok(sol.error < 1e-3);
});

test('limit-blocked target returns a legal closest pose', () => {
  // very close below the shoulder: needs elbow fold beyond -150°
  const sol = solveIK({ x: 0.2, y: 0.5, z: 0 }, HOME);
  assert.ok(inLimits(sol.q));
  assert.notEqual(sol.status, 'ok');
});
