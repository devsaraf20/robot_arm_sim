/**
 * shaders.js — two small hand-written GPU shaders.
 *
 * A shader is a tiny program that runs on the graphics card, in a language called
 * GLSL. Every material has two:
 *   • VERTEX shader   – runs once per vertex (corner) and decides WHERE it goes on screen.
 *   • FRAGMENT shader – runs once per pixel the shape covers and decides its COLOUR.
 *
 * If you have used Unreal's Material Editor or Blender's Shader Nodes, you have
 * already built shaders — those node graphs are turned into GLSL like this behind
 * the scenes. Here we just write the "nodes" as text.
 *
 * Values passed in from JavaScript are "uniforms" (the same for every pixel —
 * like a Scalar/Vector Parameter in Unreal). Values passed from the vertex shader
 * to the fragment shader are "varyings" (interpolated across the triangle).
 */
import * as THREE from 'three';

/* ─────────────────────────────────────────────────────────────────────────────
 * 1) INFINITE-LOOKING GRID FLOOR
 * A single big flat square. The fragment shader draws grid lines procedurally
 * (no texture image) and fades them out with distance so the edge is invisible.
 * ──────────────────────────────────────────────────────────────────────────── */
export function createGridMaterial({ color = '#94a3b8', accent = '#38bdf8' } = {}) {
  return new THREE.ShaderMaterial({
    transparent: true,
    depthWrite: false,
    uniforms: {
      uColor: { value: new THREE.Color(color) },
      uAccent: { value: new THREE.Color(accent) },
      uFadeDistance: { value: 14.0 },
    },
    vertexShader: /* glsl */ `
      varying vec3 vWorldPos;            // sent to the fragment shader
      void main() {
        // modelMatrix: object → world. We keep the world position for the grid math.
        vec4 world = modelMatrix * vec4(position, 1.0);
        vWorldPos = world.xyz;
        // projectionMatrix * viewMatrix: world → camera → screen (the "camera lens").
        gl_Position = projectionMatrix * viewMatrix * world;
      }
    `,
    fragmentShader: /* glsl */ `
      uniform vec3 uColor;
      uniform vec3 uAccent;
      uniform float uFadeDistance;
      varying vec3 vWorldPos;

      // Returns 1.0 on a grid line and 0.0 between lines, with soft anti-aliased edges.
      float gridLine(vec2 p, float spacing) {
        vec2 coord = p / spacing;
        // fwidth() = "how much does this value change between neighbouring pixels".
        // Dividing by it keeps lines exactly ~1 pixel wide at any distance.
        vec2 g = abs(fract(coord - 0.5) - 0.5) / fwidth(coord);
        return 1.0 - min(min(g.x, g.y), 1.0);
      }

      void main() {
        vec2 p = vWorldPos.xz;
        float minor = gridLine(p, 0.5);
        float major = gridLine(p, 2.5);

        // Highlight the X axis (red-ish) and Z axis (blue-ish) like Blender's viewport.
        float axisX = 1.0 - min(abs(p.y) / fwidth(p.y), 1.0);
        float axisZ = 1.0 - min(abs(p.x) / fwidth(p.x), 1.0);

        vec3 col = uColor;
        float alpha = minor * 0.18 + major * 0.35;
        col = mix(col, vec3(0.95, 0.35, 0.35), axisX);
        col = mix(col, vec3(0.35, 0.55, 0.95), axisZ);
        alpha = max(alpha, max(axisX, axisZ) * 0.7);

        // Soft glow ring showing the arm's maximum horizontal reach.
        float r = length(p);
        float ring = smoothstep(0.06, 0.0, abs(r - 3.0));
        col = mix(col, uAccent, ring);
        alpha = max(alpha, ring * 0.5);

        // Fade out with distance from the centre so the floor has no hard edge.
        alpha *= 1.0 - smoothstep(uFadeDistance * 0.4, uFadeDistance, r);

        gl_FragColor = vec4(col, alpha);
      }
    `,
  });
}

/* ─────────────────────────────────────────────────────────────────────────────
 * 2) GLOWING TARGET (FRESNEL RIM)
 * "Fresnel" = surfaces facing away from you glow more at their edges, like the
 * rim of a soap bubble. Same as the Fresnel node in Unreal/Blender.
 * ──────────────────────────────────────────────────────────────────────────── */
export function createGlowMaterial(color = '#22c55e') {
  return new THREE.ShaderMaterial({
    transparent: true,
    depthWrite: false,
    uniforms: {
      uColor: { value: new THREE.Color(color) },
      uTime: { value: 0 },
    },
    vertexShader: /* glsl */ `
      varying vec3 vNormal;
      varying vec3 vViewDir;
      void main() {
        vec4 viewPos = modelViewMatrix * vec4(position, 1.0);
        vNormal = normalize(normalMatrix * normal); // surface direction, in camera space
        vViewDir = normalize(-viewPos.xyz);         // direction from surface to camera
        gl_Position = projectionMatrix * viewPos;
      }
    `,
    fragmentShader: /* glsl */ `
      uniform vec3 uColor;
      uniform float uTime;
      varying vec3 vNormal;
      varying vec3 vViewDir;
      void main() {
        // dot = 1 when the surface faces the camera, 0 at the silhouette edge.
        float facing = clamp(dot(normalize(vNormal), normalize(vViewDir)), 0.0, 1.0);
        float rim = pow(1.0 - facing, 2.0);
        float pulse = 0.85 + 0.15 * sin(uTime * 3.0);   // gentle breathing animation
        vec3 col = uColor * (0.35 + rim * 1.6) * pulse;
        gl_FragColor = vec4(col, 0.35 + rim * 0.65);
      }
    `,
  });
}
