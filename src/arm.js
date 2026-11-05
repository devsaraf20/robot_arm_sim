/**
 * arm.js — builds the 3D model of the robot arm out of simple shapes.
 *
 * The important trick is the PARENT → CHILD hierarchy (same as bones/parenting in
 * Blender or attaching components in Unreal):
 *
 *   root
 *    └─ baseYaw          rotates around Y   (joint 0)
 *        └─ shoulder     rotates around Z   (joint 1), placed BASE_HEIGHT up
 *            └─ elbow    rotates around Z   (joint 2), placed L1 along the upper arm
 *                └─ tip  placed L2 along the forearm
 *
 * Because each group lives inside its parent, rotating the shoulder automatically
 * carries the elbow and tip with it. Three.js does the forward kinematics for the
 * picture; ik.js does the same math by hand for the solver — and they agree.
 */
import * as THREE from 'three';
import { ARM } from './ik.js';

export const JOINT_COLORS = ['#2dd4bf', '#fb923c', '#a78bfa', '#f472b6']; // base, shoulder, elbow, gripper

/**
 * @param {object}  opts
 * @param {boolean} opts.ghost  build a see-through "ghost" copy. The digital twin uses
 *                              it to show where the REAL arm reports it is, overlaid
 *                              on the simulated one (like an onion-skin in animation).
 */
export function createArm({ ghost = false } = {}) {
  const root = new THREE.Group();

  // Materials: "PBR" like Blender's Principled BSDF — colour + metalness + roughness.
  let body, dark, accent;
  if (ghost) {
    // One flat, unlit, transparent material for everything. depthWrite: false stops the
    // ghost from hiding parts of itself or the real model behind it.
    const g = new THREE.MeshBasicMaterial({ color: 0x38bdf8, transparent: true, opacity: 0.22, depthWrite: false });
    body = dark = g;
    accent = JOINT_COLORS.map(() => g);
  } else {
    body = new THREE.MeshStandardMaterial({ color: 0xe8eaf0, metalness: 0.15, roughness: 0.45 });
    dark = new THREE.MeshStandardMaterial({ color: 0x2a2f3a, metalness: 0.6, roughness: 0.35 });
    accent = JOINT_COLORS.map(
      (c) => new THREE.MeshStandardMaterial({ color: c, metalness: 0.2, roughness: 0.4, emissive: c, emissiveIntensity: 0.15 })
    );
  }

  const mesh = (geo, mat) => {
    const m = new THREE.Mesh(geo, mat);
    m.castShadow = !ghost;
    m.receiveShadow = !ghost;
    return m;
  };

  // ── Static pedestal (does not rotate) ───────────────────────────────────────
  const pedestal = mesh(new THREE.CylinderGeometry(0.75, 0.85, 0.18, 48), dark);
  pedestal.position.y = 0.09;
  root.add(pedestal);

  // ── Joint 0: base yaw ───────────────────────────────────────────────────────
  const baseYaw = new THREE.Group();
  root.add(baseYaw);

  const turntable = mesh(new THREE.CylinderGeometry(0.6, 0.62, 0.12, 48), accent[0]);
  turntable.position.y = 0.24;
  baseYaw.add(turntable);

  const column = mesh(new THREE.CylinderGeometry(0.32, 0.42, ARM.BASE_HEIGHT - 0.3, 32), body);
  column.position.y = 0.3 + (ARM.BASE_HEIGHT - 0.3) / 2;
  baseYaw.add(column);

  // A small notch so you can see which way the base faces.
  const notch = mesh(new THREE.BoxGeometry(0.18, 0.06, 0.12), dark);
  notch.position.set(0.6, 0.27, 0);
  baseYaw.add(notch);

  // ── Joint 1: shoulder ───────────────────────────────────────────────────────
  const shoulder = new THREE.Group();
  shoulder.position.y = ARM.BASE_HEIGHT;
  baseYaw.add(shoulder);

  // Cylinders are built standing up (along Y); rotate by 90° about X so the
  // hinge axis points along Z — the axis the joint actually turns around.
  const hinge = (radius, width, mat) => {
    const h = mesh(new THREE.CylinderGeometry(radius, radius, width, 40), mat);
    h.rotation.x = Math.PI / 2;
    return h;
  };
  shoulder.add(hinge(0.3, 0.62, accent[1]));

  // Upper arm: a box whose LEFT end sits at the pivot. We shift it by half its
  // length so the pivot (the group's origin) is at the end, not the middle —
  // exactly like moving an object's origin in Blender.
  const upper = mesh(new THREE.BoxGeometry(ARM.L1, 0.3, 0.4), body);
  upper.position.x = ARM.L1 / 2;
  shoulder.add(upper);

  // ── Joint 2: elbow ──────────────────────────────────────────────────────────
  const elbow = new THREE.Group();
  elbow.position.x = ARM.L1;
  shoulder.add(elbow);
  elbow.add(hinge(0.24, 0.5, accent[2]));

  const fore = mesh(new THREE.BoxGeometry(ARM.L2, 0.22, 0.3), body);
  fore.position.x = ARM.L2 / 2;
  elbow.add(fore);

  // ── Tip / gripper ───────────────────────────────────────────────────────────
  const tip = new THREE.Group();
  tip.position.x = ARM.L2;
  elbow.add(tip);

  const wrist = hinge(0.15, 0.34, dark);
  tip.add(wrist);
  // Two fingers that slide apart as the gripper (4th servo) opens.
  const fingers = [-1, 1].map((side) => {
    const finger = mesh(new THREE.BoxGeometry(0.3, 0.07, 0.05), accent[3] ?? dark);
    finger.position.set(0.17, 0, side * 0.1);
    finger.userData.side = side;
    tip.add(finger);
    return finger;
  });

  const joints = [baseYaw, shoulder, elbow];

  /** Apply joint angles (radians) to the model: [base, shoulder, elbow, gripper]. */
  function setAngles(q) {
    baseYaw.rotation.y = q[0];
    shoulder.rotation.z = q[1];
    elbow.rotation.z = q[2];
    if (q.length > 3) {
      const open = q[3] / (Math.PI / 2); // 0 = closed … 1 = open (90°)
      for (const f of fingers) f.position.z = f.userData.side * (0.035 + open * 0.13);
    }
  }

  /** Highlight a joint (used when hovering its slider). */
  function highlight(index) {
    if (ghost) return;
    accent.forEach((m, i) => (m.emissiveIntensity = i === index ? 0.7 : 0.15));
  }

  /** World-space position of the gripper tip. */
  const tipWorld = new THREE.Vector3();
  function getTipPosition() {
    return tip.getWorldPosition(tipWorld);
  }

  return { root, joints, setAngles, highlight, getTipPosition };
}
