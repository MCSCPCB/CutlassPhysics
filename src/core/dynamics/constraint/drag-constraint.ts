import type { RigidBody } from "../rigidbody/rigid-body.js";

/** 单刚体抓取约束；目标在一次步进内按子步插值。 */
export class DragConstraint {
  lax = 0; lay = 0; laz = 0;
  x = 0; y = 0; z = 0;
  px = 0; py = 0; pz = 0;
  qx = 0; qy = 0; qz = 0; qw = 1;
  hertz = 4;
  dampingRatio = 1;
  maxAcceleration = 60;
  holdRotation = true;
  angularHertz = 3;
  angularDampingRatio = 1;
  maxAngularAcceleration = 40;
  ix = 0; iy = 0; iz = 0;
  ax = 0; ay = 0; az = 0;

  constructor(readonly body: RigidBody) {}

  finish(): void {
    this.px = this.x; this.py = this.y; this.pz = this.z;
  }

  warmStart(): void {
    const b = this.body, r = b.transform, m = b.invMass;
    if (m === 0) return;
    const x = r.r00 * this.lax + r.r01 * this.lay + r.r02 * this.laz;
    const y = r.r10 * this.lax + r.r11 * this.lay + r.r12 * this.laz;
    const z = r.r20 * this.lax + r.r21 * this.lay + r.r22 * this.laz;
    b.vx += m * this.ix; b.vy += m * this.iy; b.vz += m * this.iz;
    const tx = y * this.iz - z * this.iy + this.ax;
    const ty = z * this.ix - x * this.iz + this.ay;
    const tz = x * this.iy - y * this.ix + this.az;
    b.wx += b.ii00 * tx + b.ii01 * ty + b.ii02 * tz;
    b.wy += b.ii10 * tx + b.ii11 * ty + b.ii12 * tz;
    b.wz += b.ii20 * tx + b.ii21 * ty + b.ii22 * tz;
  }

  solve(h: number, fraction: number): void {
    const b = this.body, r = b.transform, m = b.invMass;
    if (m === 0) return;
    const a = b.ii00, c = b.ii01, d = b.ii02, e = b.ii11, f = b.ii12, g = b.ii22;
    const x = r.r00 * this.lax + r.r01 * this.lay + r.r02 * this.laz;
    const y = r.r10 * this.lax + r.r11 * this.lay + r.r12 * this.laz;
    const z = r.r20 * this.lax + r.r21 * this.lay + r.r22 * this.laz;
    const ex = r.px + x - (this.px + (this.x - this.px) * fraction);
    const ey = r.py + y - (this.py + (this.y - this.py) * fraction);
    const ez = r.pz + z - (this.pz + (this.z - this.pz) * fraction);
    let omega = 2 * Math.PI * this.hertz, damp = 2 * this.dampingRatio + h * omega;
    let scale = h * omega * damp, soft = 1 / (1 + scale), bias = omega / damp;
    scale *= soft;
    const k00 = m + (-(-z * e + y * f) * z + (-z * f + y * g) * y);
    const k01 = (-z * c + y * d) * z - (-z * f + y * g) * x;
    const k02 = -(-z * c + y * d) * y + (-z * e + y * f) * x;
    const k11 = m + ((z * a - x * d) * z - (z * d - x * g) * x);
    const k12 = -(z * a - x * d) * y + (z * c - x * f) * x;
    const k22 = m + (-(-y * a + x * c) * y + (-y * c + x * e) * x);
    const co00 = k11 * k22 - k12 * k12, co01 = k02 * k12 - k01 * k22, co02 = k01 * k12 - k02 * k11;
    let det = k00 * co00 + k01 * co01 + k02 * co02;
    if (Math.abs(det) > 1e-12) {
      det = 1 / det;
      const vx = b.vx + b.wy * z - b.wz * y + bias * ex;
      const vy = b.vy + b.wz * x - b.wx * z + bias * ey;
      const vz = b.vz + b.wx * y - b.wy * x + bias * ez;
      const oldX = this.ix, oldY = this.iy, oldZ = this.iz;
      let jx = oldX - scale * (co00 * vx + co01 * vy + co02 * vz) * det - soft * oldX;
      let jy = oldY - scale * (co01 * vx + (k00 * k22 - k02 * k02) * vy + (k01 * k02 - k00 * k12) * vz) * det - soft * oldY;
      let jz = oldZ - scale * (co02 * vx + (k01 * k02 - k00 * k12) * vy + (k00 * k11 - k01 * k01) * vz) * det - soft * oldZ;
      const max = h * this.maxAcceleration / m, length2 = jx * jx + jy * jy + jz * jz;
      if (length2 > max * max) { const s = max / Math.sqrt(length2); jx *= s; jy *= s; jz *= s; }
      this.ix = jx; this.iy = jy; this.iz = jz;
      jx -= oldX; jy -= oldY; jz -= oldZ;
      b.vx += m * jx; b.vy += m * jy; b.vz += m * jz;
      const tx = y * jz - z * jy, ty = z * jx - x * jz, tz = x * jy - y * jx;
      b.wx += a * tx + c * ty + d * tz;
      b.wy += c * tx + e * ty + f * tz;
      b.wz += d * tx + f * ty + g * tz;
    }
    if (!this.holdRotation) return;
    let qx: number, qy: number, qz: number, qw: number;
    const trace = r.r00 + r.r11 + r.r22;
    if (trace > 0) {
      const s = Math.sqrt(trace + 1), inv = 0.5 / s;
      qw = 0.5 * s; qx = (r.r21 - r.r12) * inv; qy = (r.r02 - r.r20) * inv; qz = (r.r10 - r.r01) * inv;
    } else if (r.r00 > r.r11 && r.r00 > r.r22) {
      const s = Math.sqrt(r.r00 - r.r11 - r.r22 + 1), inv = 0.5 / s;
      qx = 0.5 * s; qy = (r.r01 + r.r10) * inv; qz = (r.r02 + r.r20) * inv; qw = (r.r21 - r.r12) * inv;
    } else if (r.r11 > r.r22) {
      const s = Math.sqrt(r.r11 - r.r22 - r.r00 + 1), inv = 0.5 / s;
      qy = 0.5 * s; qx = (r.r01 + r.r10) * inv; qz = (r.r12 + r.r21) * inv; qw = (r.r02 - r.r20) * inv;
    } else {
      const s = Math.sqrt(r.r22 - r.r00 - r.r11 + 1), inv = 0.5 / s;
      qz = 0.5 * s; qx = (r.r02 + r.r20) * inv; qy = (r.r12 + r.r21) * inv; qw = (r.r10 - r.r01) * inv;
    }
    let rx = -this.qw * qx + this.qx * qw - this.qy * qz + this.qz * qy;
    let ry = -this.qw * qy + this.qx * qz + this.qy * qw - this.qz * qx;
    let rz = -this.qw * qz - this.qx * qy + this.qy * qx + this.qz * qw;
    let rw = this.qw * qw + this.qx * qx + this.qy * qy + this.qz * qz;
    if (rw < 0) { rx = -rx; ry = -ry; rz = -rz; rw = -rw; }
    const length = Math.sqrt(rx * rx + ry * ry + rz * rz);
    const angle = length > 1e-9 ? 2 * Math.atan2(length, rw) / length : 0;
    omega = 2 * Math.PI * this.angularHertz; damp = 2 * this.angularDampingRatio + h * omega;
    scale = h * omega * damp; soft = 1 / (1 + scale); bias = omega / damp; scale *= soft;
    const c00 = e * g - f * f, c01 = d * f - c * g, c02 = c * f - d * e;
    det = a * c00 + c * c01 + d * c02;
    if (Math.abs(det) < 1e-18) return;
    det = 1 / det;
    const i00 = c00 * det, i01 = c01 * det, i02 = c02 * det;
    const i11 = (a * g - d * d) * det, i12 = (c * d - a * f) * det, i22 = (a * e - c * c) * det;
    const wx = b.wx - bias * rx * angle, wy = b.wy - bias * ry * angle, wz = b.wz - bias * rz * angle;
    const oldX = this.ax, oldY = this.ay, oldZ = this.az;
    let jx = oldX - scale * (i00 * wx + i01 * wy + i02 * wz) - soft * oldX;
    let jy = oldY - scale * (i01 * wx + i11 * wy + i12 * wz) - soft * oldY;
    let jz = oldZ - scale * (i02 * wx + i12 * wy + i22 * wz) - soft * oldZ;
    const max = h * this.maxAngularAcceleration * Math.max(i00, i11, i22), length2 = jx * jx + jy * jy + jz * jz;
    if (length2 > max * max) { const s = max / Math.sqrt(length2); jx *= s; jy *= s; jz *= s; }
    this.ax = jx; this.ay = jy; this.az = jz;
    jx -= oldX; jy -= oldY; jz -= oldZ;
    b.wx += a * jx + c * jy + d * jz;
    b.wy += c * jx + e * jy + f * jz;
    b.wz += d * jx + f * jy + g * jz;
  }
}
