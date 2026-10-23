// 移植自 OimoPhysics src/oimo/dynamics/{Contact,ContactLink}.hx（MIT，见 Credits.md）
import { CachedDetectorData } from "../collision/narrowphase/cached-detector-data.js";
import { GeometryType } from "../collision/geometry/geometry.js";
import type { Detector } from "../collision/narrowphase/detector.js";
import { DetectorResult } from "../collision/narrowphase/detector-result.js";
import { SoftBoxBoxDetector } from "../collision/narrowphase/soft-box-box-detector.js";
import {
  ALTERNATIVE_CONTACT_POSITION_CORRECTION_ALGORITHM,
  ALTERNATIVE_POSITION_CORRECTION_DEPTH_THRESHOLD,
  DEFAULT_CONTACT_POSITION_CORRECTION_ALGORITHM,
} from "../settings.js";
import { ContactConstraint } from "./constraint/contact/contact-constraint.js";
import { Manifold } from "./constraint/contact/manifold.js";
import { ManifoldUpdater } from "./constraint/contact/manifold-updater.js";
import type { RigidBody } from "./rigidbody/rigid-body.js";
import { RigidBodyType } from "./rigidbody/rigid-body-config.js";
import type { Shape } from "./rigidbody/shape.js";

const softBoxDetector = new SoftBoxBoxDetector();

/** 刚体上的接触链表节点：通过它找到接触和另一个刚体 */
export class ContactLink {
  prev: ContactLink | null = null;
  next: ContactLink | null = null;
  contact: Contact | null = null;
  other: RigidBody | null = null;
}

function pushLink(b: RigidBody, l: ContactLink): void {
  if (b.contactLinkList === null) {
    b.contactLinkList = l;
    b.contactLinkListLast = l;
  } else {
    b.contactLinkListLast!.next = l;
    l.prev = b.contactLinkListLast;
    b.contactLinkListLast = l;
  }
}

function removeLink(b: RigidBody, l: ContactLink): void {
  const prev = l.prev, next = l.next;
  if (prev !== null) prev.next = next;
  if (next !== null) next.prev = prev;
  if (l === b.contactLinkList) b.contactLinkList = next;
  if (l === b.contactLinkListLast) b.contactLinkListLast = prev;
  l.next = null;
  l.prev = null;
}

/** 由包围盒重叠建立的形状对，重叠结束时销毁。touching 表示形状实际接触状态。 */
export class Contact {
  prev: Contact | null = null;
  next: Contact | null = null;
  readonly link1 = new ContactLink();
  readonly link2 = new ContactLink();
  s1: Shape | null = null;
  s2: Shape | null = null;
  b1: RigidBody | null = null;
  b2: RigidBody | null = null;
  detector: Detector | null = null;
  readonly detectorResult = new DetectorResult();
  /** 这对形状跨步保存的窄相数据（GJK 缓存）。Contact 对象复用时在 detach 里清空 */
  readonly cachedDetectorData = new CachedDetectorData();
  /** 这一步宽相又收集到了这一对 */
  latest = false;
  /** 这一步跳过窄相检测 */
  shouldBeSkipped = false;
  readonly manifold = new Manifold();
  readonly updater = new ManifoldUpdater(this.manifold);
  readonly contactConstraint = new ContactConstraint(this.manifold);
  /** 两个形状是否真的接触 */
  touching = false;

  attach(s1: Shape, s2: Shape, detector: Detector | null): void {
    this.s1 = s1;
    this.s2 = s2;
    const b1 = (this.b1 = s1.rigidBody!), b2 = (this.b2 = s2.rigidBody!);
    this.touching = false;
    pushLink(b1, this.link1);
    pushLink(b2, this.link2);
    b1.numContactLinks++;
    b2.numContactLinks++;
    this.link1.other = b2;
    this.link2.other = b1;
    this.link1.contact = this;
    this.link2.contact = this;
    this.detector = detector;
    this.contactConstraint.attach(s1, s2);
  }

  detach(): void {
    const b1 = this.b1!, b2 = this.b2!;
    removeLink(b1, this.link1);
    removeLink(b2, this.link2);
    b1.numContactLinks--;
    b2.numContactLinks--;
    this.link1.other = null;
    this.link2.other = null;
    this.link1.contact = null;
    this.link2.contact = null;
    this.s1 = null;
    this.s2 = null;
    this.b1 = null;
    this.b2 = null;
    this.touching = false;
    this.cachedDetectorData.clear();
    this.manifold.clear();
    this.detector = null;
    this.contactConstraint.detach();
  }

  /**
   * 执行窄相检测并更新流形。
   * 预测接触模式使用形状的 speculativeMargin，touching 延迟至求解结束后更新。
   */
  updateManifold(speculative = false): void {
    if (this.detector === null) return;
    const s1 = this.s1!, s2 = this.s2!, result = this.detectorResult;
    const b1 = this.b1!, b2 = this.b2!;
    const boxPair = s1.geom.type === GeometryType.BOX && s2.geom.type === GeometryType.BOX;
    const detector = speculative && boxPair ? softBoxDetector : this.detector;
    let margin = s1.speculativeMargin + s2.speculativeMargin;
    // 动态接触按相对速度计算预测距离；静态接触使用运动方形状的预测距离。
    if (margin > 0 && b1.type !== RigidBodyType.STATIC && b2.type !== RigidBodyType.STATIC) {
      const w = b1.world!;
      const dvX = b2.vx - b1.vx, dvY = b2.vy - b1.vy, dvZ = b2.vz - b1.vz;
      let speed = Math.sqrt(dvX * dvX + dvY * dvY + dvZ * dvZ) +
        Math.sqrt(b1.wx * b1.wx + b1.wy * b1.wy + b1.wz * b1.wz) + Math.sqrt(b2.wx * b2.wx + b2.wy * b2.wy + b2.wz * b2.wz);
      if (speed > w.pairSpeculativeSpeed) speed = w.pairSpeculativeSpeed;
      margin = w.speculativeDistance + w.speculativeTick() * speed;
      if (margin > w.maxSpeculativeDistance) margin = w.maxSpeculativeDistance;
    }
    // 盒子地形固定为检测器的第一侧，再还原接触双方的顺序。
    if (speculative && b1.type !== RigidBodyType.STATIC && b2.type === RigidBodyType.STATIC && boxPair) {
      detector.detect(result, s2.geom, s1.geom, s2.transform, s1.transform, this.cachedDetectorData, margin);
      result.nx = -result.nx; result.ny = -result.ny; result.nz = -result.nz;
      for (let i = 0; i < result.numPoints; i++) {
        const p = result.points[i], x = p.p1x, y = p.p1y, z = p.p1z;
        p.p1x = p.p2x; p.p1y = p.p2y; p.p1z = p.p2z;
        p.p2x = x; p.p2y = y; p.p2z = z;
      }
    } else {
      detector.detect(result, s1.geom, s2.geom, s1.transform, s2.transform, this.cachedDetectorData, margin);
    }
    // 通过形状过滤器剔除接触点，默认不过滤。
    if (s1.contactFilter !== null) s1.contactFilter(s1, result, true, s2);
    if (s2.contactFilter !== null) s2.contactFilter(s2, result, false, s1);
    const found = result.numPoints > 0;
    if (!speculative) this.touching = found;
    if (found) {
      // 接触法线改变后，旧编号不再代表同一承重面。
      if (speculative && this.manifold.nX * result.nx + this.manifold.nY * result.ny + this.manifold.nZ * result.nz <= 0.99) {
        this.manifold.clear();
      }
      this.manifold.buildBasis(result.nx, result.ny, result.nz);
      // 穿透较深时改用另一种位置修正算法（默认是 split impulse）
      this.contactConstraint.positionCorrectionAlgorithm =
        result.getMaxDepth() > ALTERNATIVE_POSITION_CORRECTION_DEPTH_THRESHOLD
          ? ALTERNATIVE_CONTACT_POSITION_CORRECTION_ALGORITHM
          : DEFAULT_CONTACT_POSITION_CORRECTION_ALGORITHM;
      // GJK/EPA 每次只给 1 个点，和流形里已有的点合并；其余检测器一次给出整个流形
      if (result.incremental) {
        this.updater.incrementalUpdate(result, this.b1!.transform, this.b2!.transform, margin);
      } else {
        this.updater.totalUpdate(result, this.b1!.transform, this.b2!.transform, speculative);
      }
    } else {
      this.manifold.clear();
    }
  }
}
