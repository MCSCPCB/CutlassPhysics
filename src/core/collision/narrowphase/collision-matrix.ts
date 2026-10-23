// 移植自 OimoPhysics src/oimo/collision/narrowphase/CollisionMatrix.hx（MIT，见 THIRD-PARTY-NOTICES.txt）
import { GeometryType } from "../geometry/geometry.js";
import { BoxBoxDetector } from "./box-box-detector.js";
import { CapsuleCapsuleDetector } from "./capsule-capsule-detector.js";
import type { Detector } from "./detector.js";
import { GjkEpaDetector } from "./gjk-epa-detector.js";
import { SphereBoxDetector } from "./sphere-box-detector.js";
import { SphereCapsuleDetector } from "./sphere-capsule-detector.js";
import { SphereSphereDetector } from "./sphere-sphere-detector.js";

/** 按两个几何体的类型选择窄相检测器。不对称：getDetector(a, b) 和 getDetector(b, a) 可能不同 */
export class CollisionMatrix {
  private readonly detectors: Detector[][] = [];

  constructor() {
    // 没有专门检测器的组合（圆柱、圆锥、凸包，以及盒子对胶囊体）都用 GJK/EPA，共用一个实例
    const gjkEpa = new GjkEpaDetector();
    for (let i = 0; i < 6; i++) this.detectors.push([gjkEpa, gjkEpa, gjkEpa, gjkEpa, gjkEpa, gjkEpa]);
    const d = this.detectors;
    d[GeometryType.SPHERE][GeometryType.SPHERE] = new SphereSphereDetector();
    d[GeometryType.SPHERE][GeometryType.BOX] = new SphereBoxDetector(false);
    d[GeometryType.SPHERE][GeometryType.CAPSULE] = new SphereCapsuleDetector(false);
    d[GeometryType.BOX][GeometryType.SPHERE] = new SphereBoxDetector(true);
    d[GeometryType.BOX][GeometryType.BOX] = new BoxBoxDetector();
    d[GeometryType.CAPSULE][GeometryType.SPHERE] = new SphereCapsuleDetector(true);
    d[GeometryType.CAPSULE][GeometryType.CAPSULE] = new CapsuleCapsuleDetector();
  }

  getDetector(type1: number, type2: number): Detector {
    return this.detectors[type1][type2];
  }
}
