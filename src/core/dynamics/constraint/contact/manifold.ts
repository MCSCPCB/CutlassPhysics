// 移植自 OimoPhysics src/oimo/dynamics/constraint/contact/Manifold.hx（MIT，见 Credits.md）
import type { Transform } from "../../../common/transform.js";
import { MAX_MANIFOLD_POINTS } from "../../../settings.js";
import { ManifoldPoint } from "./manifold-point.js";

/** 接触流形：一对形状的接触法线、两根切向，以及最多 4 个接触点 */
export class Manifold {
  /** 法线、切向、副切向，三者两两正交 */
  nX = 0; nY = 0; nZ = 0;
  tX = 0; tY = 0; tZ = 0;
  bX = 0; bY = 0; bZ = 0;
  numPoints = 0;
  readonly points: ManifoldPoint[] = [];

  constructor() {
    for (let i = 0; i < MAX_MANIFOLD_POINTS; i++) this.points.push(new ManifoldPoint());
  }

  clear(): void {
    for (let i = 0; i < this.numPoints; i++) this.points[i].clear();
    this.numPoints = 0;
  }

  /** 由法线构造正交基：用法线分量最小的那根轴构造切向，数值最稳 */
  buildBasis(nx: number, ny: number, nz: number): void {
    this.nX = nx; this.nY = ny; this.nZ = nz;
    const nx2 = nx * nx, ny2 = ny * ny, nz2 = nz * nz;
    let tx: number, ty: number, tz: number, bx: number, by: number, bz: number;
    // 最小分量判断：a < b 时比较 a、c，否则比较 b、c
    const axis = nx2 < ny2 ? (nx2 < nz2 ? 0 : 2) : ny2 < nz2 ? 1 : 2;
    if (axis === 0) {
      const invL = 1 / Math.sqrt(ny2 + nz2);
      tx = 0; ty = -nz * invL; tz = ny * invL;
      bx = ny * tz - nz * ty; by = -nx * tz; bz = nx * ty;
    } else if (axis === 1) {
      const invL = 1 / Math.sqrt(nx2 + nz2);
      tx = nz * invL; ty = 0; tz = -nx * invL;
      bx = ny * tz; by = nz * tx - nx * tz; bz = -ny * tx;
    } else {
      const invL = 1 / Math.sqrt(nx2 + ny2);
      tx = -ny * invL; ty = nx * invL; tz = 0;
      bx = -nz * ty; by = nz * tx; bz = nx * ty - ny * tx;
    }
    this.tX = tx; this.tY = ty; this.tZ = tz;
    this.bX = bx; this.bY = by; this.bZ = bz;
  }

  /** 刚体移动后，按局部坐标重新算接触点的世界位置和深度 */
  updateDepthsAndPositions(tf1: Transform, tf2: Transform): void {
    for (let i = 0; i < this.numPoints; i++) {
      const p = this.points[i];
      p.rp1X = tf1.r00 * p.lp1X + tf1.r01 * p.lp1Y + tf1.r02 * p.lp1Z;
      p.rp1Y = tf1.r10 * p.lp1X + tf1.r11 * p.lp1Y + tf1.r12 * p.lp1Z;
      p.rp1Z = tf1.r20 * p.lp1X + tf1.r21 * p.lp1Y + tf1.r22 * p.lp1Z;
      p.rp2X = tf2.r00 * p.lp2X + tf2.r01 * p.lp2Y + tf2.r02 * p.lp2Z;
      p.rp2Y = tf2.r10 * p.lp2X + tf2.r11 * p.lp2Y + tf2.r12 * p.lp2Z;
      p.rp2Z = tf2.r20 * p.lp2X + tf2.r21 * p.lp2Y + tf2.r22 * p.lp2Z;
      p.p1X = p.rp1X + tf1.px; p.p1Y = p.rp1Y + tf1.py; p.p1Z = p.rp1Z + tf1.pz;
      p.p2X = p.rp2X + tf2.px; p.p2Y = p.rp2Y + tf2.py; p.p2Z = p.rp2Z + tf2.pz;
      const dX = p.p1X - p.p2X, dY = p.p1Y - p.p2Y, dZ = p.p1Z - p.p2Z;
      const dotN = dX * this.nX + dY * this.nY + dZ * this.nZ;
      p.depth = -dotN;
    }
  }
}
