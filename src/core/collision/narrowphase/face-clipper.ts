// 移植自 OimoPhysics src/oimo/collision/narrowphase/detector/BoxBoxDetector.hx 中的
// IncidentVertex 和 FaceClipper（MIT，见 Credits.md）
// 把入射面（四边形）裁剪进参考面矩形 [-w, w] × [-h, h]，最多得到 8 个点，再精简到 4 个。
// 四边裁剪直接访问顶点字段，减少函数调用开销。

/** 入射面顶点：x、y 是参考面坐标系里的坐标，wx、wy、wz 是世界坐标（相对参考盒中心） */
export class IncidentVertex {
  x = 0;
  y = 0;
  wx = 0;
  wy = 0;
  wz = 0;
}

function copyVertex(o: IncidentVertex, v: IncidentVertex): void {
  o.x = v.x;
  o.y = v.y;
  o.wx = v.wx;
  o.wy = v.wy;
  o.wz = v.wz;
}

export class FaceClipper {
  w = 0;
  h = 0;
  numVertices = 0;
  vertices: IncidentVertex[] = [];
  private tmpVertices: IncidentVertex[] = [];

  constructor() {
    for (let i = 0; i < 8; i++) {
      this.vertices.push(new IncidentVertex());
      this.tmpVertices.push(new IncidentVertex());
    }
  }

  init(w: number, h: number): void {
    this.w = w;
    this.h = h;
    this.numVertices = 0;
  }

  addIncidentVertex(x: number, y: number, wx: number, wy: number, wz: number): void {
    const v = this.vertices[this.numVertices++];
    v.x = x;
    v.y = y;
    v.wx = wx;
    v.wy = wy;
    v.wz = wz;
  }

  /** 依次用参考面的左、右、上、下四条边裁剪，最多得到 8 个点 */
  clip(): void {
    // 入射面严格位于参考面内时直接保留缓冲区；边界点仍执行裁剪。
    let inside = true;
    for (let i = 0; i < this.numVertices; i++) {
      const v = this.vertices[i];
      if (!(this.w + v.x > 0 && this.w - v.x > 0 && this.h + v.y > 0 && this.h - v.y > 0)) {
        inside = false;
        break;
      }
    }
    if (inside) return;
    // 到四条边的有向距离分别是 w + x、w - x、h + y、h - y（正数表示在内侧）
    this.clipPass(this.w, false, false);
    this.clipPass(this.w, false, true);
    this.clipPass(this.h, true, false);
    this.clipPass(this.h, true, true);
  }

  /** 超过 4 个点时保留两条对角线方向的极值点。 */
  reduce(): void {
    if (this.numVertices < 4) return;
    // 对角线投影分别为 x + y 和 -x + y。
    const src = this.vertices;
    const first = src[0];
    let max1 = first.x + first.y, min1 = max1;
    let max2 = first.y - first.x, min2 = max2;
    let max1V = first, min1V = first, max2V = first, min2V = first;
    for (let i = 1; i < this.numVertices; i++) {
      const v = src[i];
      const dot1 = v.x + v.y;
      const dot2 = v.y - v.x;
      if (dot1 > max1) { max1 = dot1; max1V = v; }
      if (dot1 < min1) { min1 = dot1; min1V = v; }
      if (dot2 > max2) { max2 = dot2; max2V = v; }
      if (dot2 < min2) { min2 = dot2; min2V = v; }
    }
    // 按顺序放进另一个缓冲区，再交换
    const dst = this.tmpVertices;
    copyVertex(dst[0], max1V);
    copyVertex(dst[1], max2V);
    copyVertex(dst[2], min1V);
    copyVertex(dst[3], min2V);
    this.vertices = dst;
    this.tmpVertices = src;
    this.numVertices = 4;
  }

  /**
   * 用一条边裁剪一遍，结果写进另一个缓冲区再交换。
   * 顶点到边的有向距离是 negate ? bound - c : bound + c（c 是顶点的 x 或 y），正数表示在内侧。
   */
  private clipPass(bound: number, useY: boolean, negate: boolean): void {
    const src = this.vertices, dst = this.tmpVertices, n = this.numVertices;
    let m = 0;
    let o: IncidentVertex, t: number;
    for (let i = 0; i < n; i++) {
      const v1 = src[i], v2 = src[(i + 1) % n];
      const c1 = useY ? v1.y : v1.x, c2 = useY ? v2.y : v2.x;
      const s1 = negate ? bound - c1 : bound + c1;
      const s2 = negate ? bound - c2 : bound + c2;
      if (s1 > 0 && s2 > 0) {
        o = dst[m++];
        o.x = v1.x; o.y = v1.y; o.wx = v1.wx; o.wy = v1.wy; o.wz = v1.wz;
      } else if (s1 > 0 && s2 <= 0) {
        // v2 被裁掉：保留 v1，再加上 v1→v2 与边的交点
        o = dst[m++];
        o.x = v1.x; o.y = v1.y; o.wx = v1.wx; o.wy = v1.wy; o.wz = v1.wz;
        t = s1 / (s1 - s2);
        o = dst[m++];
        o.x = v1.x + (v2.x - v1.x) * t; o.y = v1.y + (v2.y - v1.y) * t;
        o.wx = v1.wx + (v2.wx - v1.wx) * t; o.wy = v1.wy + (v2.wy - v1.wy) * t; o.wz = v1.wz + (v2.wz - v1.wz) * t;
      } else if (s1 <= 0 && s2 > 0) {
        // v1 被裁掉：只加交点
        t = s1 / (s1 - s2);
        o = dst[m++];
        o.x = v1.x + (v2.x - v1.x) * t; o.y = v1.y + (v2.y - v1.y) * t;
        o.wx = v1.wx + (v2.wx - v1.wx) * t; o.wy = v1.wy + (v2.wy - v1.wy) * t; o.wz = v1.wz + (v2.wz - v1.wz) * t;
      }
    }
    this.vertices = dst;
    this.tmpVertices = src;
    this.numVertices = m;
  }
}
