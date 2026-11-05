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

export const JOINT_COLORS = ['#2dd4bf', '#fb923c', '#a78bfa']; // base, shoulder, elbow

export function createArm() {
  const root = new THREE.Group();

  // Materials: "PBR" like Blender's Principled BSDF — colour + metalness + roughness.
  const body = new THREE.MeshStandardMaterial({ color: 0xe8eaf0, metalness: 0.15, roughness: 0.45 });
  const dark = new THREE.MeshStandardMaterial({ color: 0x2a2f3a, metalness: 0.6, roughness: 0.35 });
  const accent = JOINT_COLORS.map(
    (c) => new THREE.MeshStandardMaterial({ color: c, metalness: 0.2, roughness: 0.4, emissive: c, emissiveIntensity: 0.15 })
  );

  const mesh = (geo, mat) => {
    const m = new THREE.Mesh(geo, mat);
    m.castShadow = true;
    m.receiveShadow = true;
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
  for (const side of [-1, 1]) {
    const finger = mesh(new THREE.BoxGeometry(0.28, 0.06, 0.06), dark);
    finger.position.set(0.16, 0, side * 0.1);
    tip.add(finger);
  }

  const joints = [baseYaw, shoulder, elbow];

  /** Apply joint angles (radians) to the model. */
  function setAngles(q) {
    baseYaw.rotation.y = q[0];
    shoulder.rotation.z = q[1];
    elbow.rotation.z = q[2];
  }

  /** Highlight a joint (used when hovering its slider). */
  function highlight(index) {
    accent.forEach((m, i) => (m.emissiveIntensity = i === index ? 0.7 : 0.15));
  }

  /** World-space position of the gripper tip. */
  const tipWorld = new THREE.Vector3();
  function getTipPosition() {
    return tip.getWorldPosition(tipWorld);
  }

  return { root, joints, setAngles, highlight, getTipPosition };
}
