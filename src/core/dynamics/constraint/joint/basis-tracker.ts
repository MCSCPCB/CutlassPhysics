// 移植自 OimoPhysics src/oimo/dynamics/constraint/joint/BasisTracker.hx（MIT，见 THIRD-PARTY-NOTICES.txt）
// 锚点同步使用展开的标量运算。
import { compat } from "../../../compat.js";
import type { Joint } from "./joint.js";

/**
 * 关节公共坐标系：X 轴按逆质量插值，Y、Z 轴沿最短弧更新。
 * 两端逆质量均为 0 时，插值比例取 0.5。
 */
export class BasisTracker {
  private readonly joint: Joint;
  xX = 0; xY = 0; xZ = 0;
  yX = 0; yY = 0; yZ = 0;
  zX = 0; zY = 0; zZ = 0;

  constructor(joint: Joint) {
    this.joint = joint;
  }

  /** 以两个刚体的 X 轴为准更新坐标系（原版 trackByX） */
  trackByX(): void {
    const j = this.joint;
    const invM1 = j.b1.invMass, invM2 = j.b2.invMass;
    const x1X = j.bX1X, x1Y = j.bX1Y, x1Z = j.bX1Z;
    const x2X = j.bX2X, x2Y = j.bX2Y, x2Z = j.bX2Z;

    // q = 从 X1 转到 X2 的最短弧
    let qX: number, qY: number, qZ: number, qW: number;
    let d = x1X * x2X + x1Y * x2Y + x1Z * x2Z;
    if (d < -0.999999999) {
      const x2 = x1X * x1X, y2 = x1Y * x1Y, z2 = x1Z * x1Z;
      let e: number;
      if (x2 < y2) {
        if (x2 < z2) {
          e = 1 / Math.sqrt(y2 + z2);
          qX = 0; qY = x1Z * e; qZ = -x1Y * e;
        } else {
          e = 1 / Math.sqrt(x2 + y2);
          qX = x1Y * e; qY = -x1X * e; qZ = 0;
        }
      } else if (y2 < z2) {
        e = 1 / Math.sqrt(z2 + x2);
        qX = -x1Z * e; qY = 0; qZ = x1X * e;
      } else {
        e = 1 / Math.sqrt(x2 + y2);
        qX = x1Y * e; qY = -x1X * e; qZ = 0;
      }
      qW = 0;
    } else {
      let cX = x1Y * x2Z - x1Z * x2Y, cY = x1Z * x2X - x1X * x2Z, cZ = x1X * x2Y - x1Y * x2X;
      const w = Math.sqrt((1 + d) * 0.5);
      d = 0.5 / w;
      cX *= d; cY *= d; cZ *= d;
      qX = cX; qY = cY; qZ = cZ; qW = w;
    }

    // slerp(单位四元数, q, 插值比例)。比例 = invM1 / (invM1 + invM2)，两边都不是动态刚体时取 0.5
    const ratio = invM1 + invM2 === 0 && !compat.keepOimoBugs ? 0.5 : invM1 / (invM1 + invM2);
    let q1X = 0, q1Y = 0, q1Z = 0, q1W = 1;
    let q2X = qX, q2Y = qY, q2Z = qZ, q2W = qW;
    let d1 = q1X * q2X + q1Y * q2Y + q1Z * q2Z + q1W * q2W;
    if (d1 < 0) {
      d1 = -d1;
      q2X = -q2X; q2Y = -q2Y; q2Z = -q2Z; q2W = -q2W;
    }
    let sX: number, sY: number, sZ: number, sW: number;
    if (d1 > 0.999999) {
      // 夹角很小：线性插值再归一化
      const dqX = q2X - q1X, dqY = q2Y - q1Y, dqZ = q2Z - q1Z, dqW = q2W - q1W;
      q2X = q1X + dqX * ratio;
      q2Y = q1Y + dqY * ratio;
      q2Z = q1Z + dqZ * ratio;
      q2W = q1W + dqW * ratio;
      let l = q2X * q2X + q2Y * q2Y + q2Z * q2Z + q2W * q2W;
      if (l > 1e-32) l = 1 / Math.sqrt(l);
      sX = q2X * l; sY = q2Y * l; sZ = q2Z * l; sW = q2W * l;
    } else {
      const theta = ratio * Math.acos(d1);
      q2X += q1X * -d1; q2Y += q1Y * -d1; q2Z += q1Z * -d1; q2W += q1W * -d1;
      let l = q2X * q2X + q2Y * q2Y + q2Z * q2Z + q2W * q2W;
      if (l > 1e-32) l = 1 / Math.sqrt(l);
      q2X *= l; q2Y *= l; q2Z *= l; q2W *= l;
      const sin = Math.sin(theta), cos = Math.cos(theta);
      q1X *= cos; q1Y *= cos; q1Z *= cos; q1W *= cos;
      sX = q1X + q2X * sin; sY = q1Y + q2Y * sin; sZ = q1Z + q2Z * sin; sW = q1W + q2W * sin;
    }

    // 新的 x = slerp 转过的 X1
    let x2 = 2 * sX, y2 = 2 * sY, z2 = 2 * sZ;
    let xx = sX * x2, yy = sY * y2, zz = sZ * z2;
    let xy = sX * y2, yz = sY * z2, xz = sX * z2;
    let wx = sW * x2, wy = sW * y2, wz = sW * z2;
    let m00 = 1 - yy - zz, m01 = xy - wz, m02 = xz + wy;
    let m10 = xy + wz, m11 = 1 - xx - zz, m12 = yz - wx;
    let m20 = xz - wy, m21 = yz + wx, m22 = 1 - xx - yy;
    const newXX = m00 * x1X + m01 * x1Y + m02 * x1Z;
    const newXY = m10 * x1X + m11 * x1Y + m12 * x1Z;
    const newXZ = m20 * x1X + m21 * x1Y + m22 * x1Z;
    const prevXX = this.xX, prevXY = this.xY, prevXZ = this.xZ;
    const prevYX = this.yX, prevYY = this.yY, prevYZ = this.yZ;

    // 上一次的 x 沿最短弧转到新的 x，同样转动上一次的 y
    let d2 = prevXX * newXX + prevXY * newXY + prevXZ * newXZ;
    if (d2 < -0.999999999) {
      const px2 = prevXX * prevXX, py2 = prevXY * prevXY, pz2 = prevXZ * prevXZ;
      let e: number;
      if (px2 < py2) {
        if (px2 < pz2) {
          e = 1 / Math.sqrt(py2 + pz2);
          sX = 0; sY = prevXZ * e; sZ = -prevXY * e;
        } else {
          e = 1 / Math.sqrt(px2 + py2);
          sX = prevXY * e; sY = -prevXX * e; sZ = 0;
        }
      } else if (py2 < pz2) {
        e = 1 / Math.sqrt(pz2 + px2);
        sX = -prevXZ * e; sY = 0; sZ = prevXX * e;
      } else {
        e = 1 / Math.sqrt(px2 + py2);
        sX = prevXY * e; sY = -prevXX * e; sZ = 0;
      }
      sW = 0;
    } else {
      let cX = prevXY * newXZ - prevXZ * newXY, cY = prevXZ * newXX - prevXX * newXZ, cZ = prevXX * newXY - prevXY * newXX;
      const w = Math.sqrt((1 + d2) * 0.5);
      d2 = 0.5 / w;
      cX *= d2; cY *= d2; cZ *= d2;
      sX = cX; sY = cY; sZ = cZ; sW = w;
    }
    x2 = 2 * sX; y2 = 2 * sY; z2 = 2 * sZ;
    xx = sX * x2; yy = sY * y2; zz = sZ * z2;
    xy = sX * y2; yz = sY * z2; xz = sX * z2;
    wx = sW * x2; wy = sW * y2; wz = sW * z2;
    m00 = 1 - yy - zz; m01 = xy - wz; m02 = xz + wy;
    m10 = xy + wz; m11 = 1 - xx - zz; m12 = yz - wx;
    m20 = xz - wy; m21 = yz + wx; m22 = 1 - xx - yy;
    let newYX = m00 * prevYX + m01 * prevYY + m02 * prevYZ;
    let newYY = m10 * prevYX + m11 * prevYY + m12 * prevYZ;
    let newYZ = m20 * prevYX + m21 * prevYY + m22 * prevYZ;

    // z = x × y；太短（第一次、或者转过头）时放弃上一次的坐标系，取任意一个和 x 垂直的方向
    let newZX = newXY * newYZ - newXZ * newYY;
    let newZY = newXZ * newYX - newXX * newYZ;
    let newZZ = newXX * newYY - newXY * newYX;
    if (newZX * newZX + newZY * newZY + newZZ * newZZ > 1e-6) {
      let l = newZX * newZX + newZY * newZY + newZZ * newZZ;
      if (l > 0) l = 1 / Math.sqrt(l);
      newZX *= l; newZY *= l; newZZ *= l;
    } else {
      const nx2 = newXX * newXX, ny2 = newXY * newXY, nz2 = newXZ * newXZ;
      let e: number;
      if (nx2 < ny2) {
        if (nx2 < nz2) {
          e = 1 / Math.sqrt(ny2 + nz2);
          newZX = 0; newZY = newXZ * e; newZZ = -newXY * e;
        } else {
          e = 1 / Math.sqrt(nx2 + ny2);
          newZX = newXY * e; newZY = -newXX * e; newZZ = 0;
        }
      } else if (ny2 < nz2) {
        e = 1 / Math.sqrt(nz2 + nx2);
        newZX = -newXZ * e; newZY = 0; newZZ = newXX * e;
      } else {
        e = 1 / Math.sqrt(nx2 + ny2);
        newZX = newXY * e; newZY = -newXX * e; newZZ = 0;
      }
    }
    // y = z × x
    newYX = newZY * newXZ - newZZ * newXY;
    newYY = newZZ * newXX - newZX * newXZ;
    newYZ = newZX * newXY - newZY * newXX;
    this.xX = newXX; this.xY = newXY; this.xZ = newXZ;
    this.yX = newYX; this.yY = newYY; this.yZ = newYZ;
    this.zX = newZX; this.zY = newZY; this.zZ = newZZ;
  }
}
