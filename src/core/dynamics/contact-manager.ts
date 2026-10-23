// 移植自 OimoPhysics src/oimo/dynamics/ContactManager.hx（MIT，见 THIRD-PARTY-NOTICES.txt）
import type { BroadPhase } from "../collision/broadphase/broad-phase.js";
import { CollisionMatrix } from "../collision/narrowphase/collision-matrix.js";
import { Contact } from "./contact.js";
import { RigidBodyType } from "./rigidbody/rigid-body-config.js";
import type { Shape } from "./rigidbody/shape.js";

/** 接触开始时调用 beginContact，分离或接触销毁时调用 endContact。 */
export interface ContactListener {
  /** 在窄相更新流形之后调用，此时流形里是这一步的接触点，速度还是求解之前的 */
  beginContact(c: Contact): void;
  /** 接触分开，或正在接触时被销毁（例如形状被移除）。销毁时 c 还连着两个形状 */
  endContact(c: Contact): void;
}

/** 管理世界里的接触：形状的包围盒开始重叠时创建，不再重叠时销毁 */
export class ContactManager {
  numContacts = 0;
  contactList: Contact | null = null;
  contactListLast: Contact | null = null;
  private contactPool: Contact | null = null;
  readonly broadPhase: BroadPhase;
  readonly collisionMatrix = new CollisionMatrix();
  private maxRestitutionFlag = false;
  /** 接触事件监听者，null 表示不发送通知。 */
  listener: ContactListener | null = null;

  /** 是否使用两侧恢复系数的较大值，默认使用几何平均。修改时同步到已有接触。 */
  get maxRestitution(): boolean {
    return this.maxRestitutionFlag;
  }

  set maxRestitution(value: boolean) {
    this.maxRestitutionFlag = value;
    for (let c = this.contactList; c !== null; c = c.next) c.contactConstraint.maxRestitution = value;
  }

  constructor(broadPhase: BroadPhase) {
    this.broadPhase = broadPhase;
  }

  /** 宽相：收集重叠的代理对，创建新接触，销毁过期的接触（原版 _updateContacts） */
  updateContacts(): void {
    this.broadPhase.collectPairs();
    this.createContacts();
    this.destroyOutdatedContacts();
  }

  /** 预测接触模式。窄相仅更新流形，touching 和事件由求解后的 updateTouching 更新。 */
  speculative = false;

  /** 窄相：更新不需要跳过的接触的流形（原版 _updateManifolds） */
  updateManifolds(): void {
    const listener = this.listener;
    if (this.speculative) {
      for (let c = this.contactList; c !== null; c = c.next) if (!c.shouldBeSkipped) c.updateManifold(true);
      return;
    }
    for (let c = this.contactList; c !== null; c = c.next) {
      if (c.shouldBeSkipped) continue;
      if (listener === null) {
        c.updateManifold();
      } else {
        const was = c.touching;
        c.updateManifold();
        if (c.touching !== was) {
          if (c.touching) listener.beginContact(c);
          else listener.endContact(c);
        }
      }
    }
  }

  /**
   * 求解后更新接触状态并通知。stamp 为本次步进编号。
   * 法向冲量大于 0 的点视为接触；空流形视为分离，未参与求解的接触保留状态。
   */
  updateTouching(stamp: number): void {
    const listener = this.listener;
    for (let c = this.contactList; c !== null; c = c.next) {
      const cc = c.contactConstraint;
      let touching: boolean;
      if (c.manifold.numPoints === 0) touching = false;
      else if (cc.solvedStamp === stamp) touching = cc.pushed;
      else continue;
      if (touching === c.touching) continue;
      c.touching = touching;
      if (listener !== null) {
        if (touching) listener.beginContact(c);
        else listener.endContact(c);
      }
    }
  }

  destroyContact(c: Contact): void {
    const prev = c.prev, next = c.next;
    if (prev !== null) prev.next = next;
    if (next !== null) next.prev = prev;
    if (c === this.contactList) this.contactList = next;
    if (c === this.contactListLast) this.contactListLast = prev;
    c.next = null;
    c.prev = null;
    if (this.listener !== null && c.touching) this.listener.endContact(c);
    c.detach();
    // 放回对象池
    c.next = this.contactPool;
    this.contactPool = c;
    this.numContacts--;
  }

  private createContacts(): void {
    for (let pp = this.broadPhase.proxyPairList; pp !== null; pp = pp.next) {
      // 按代理编号排序，同一对形状的先后顺序固定
      const p1 = pp.p1!, p2 = pp.p2!;
      const s1 = (p1.id < p2.id ? p1.userData : p2.userData) as Shape;
      const s2 = (p1.id < p2.id ? p2.userData : p1.userData) as Shape;
      if (!this.shouldCollide(s1, s2)) continue;
      // 在接触较少的刚体上找这一对是否已经有接触
      const b1 = s1.rigidBody!, b2 = s2.rigidBody!;
      const id1 = s1.id, id2 = s2.id;
      let found = false;
      for (let l = b1.numContactLinks < b2.numContactLinks ? b1.contactLinkList : b2.contactLinkList; l !== null; l = l.next) {
        const c = l.contact!;
        if (c.s1!.id === id1 && c.s2!.id === id2) {
          c.latest = true;
          found = true;
          break;
        }
      }
      if (found) continue;
      // 新建接触：优先从对象池里取
      let c = this.contactPool;
      if (c !== null) {
        this.contactPool = c.next;
        c.next = null;
      } else {
        c = new Contact();
      }
      if (this.contactList === null) {
        this.contactList = c;
        this.contactListLast = c;
      } else {
        this.contactListLast!.next = c;
        c.prev = this.contactListLast;
        this.contactListLast = c;
      }
      c.latest = true;
      c.attach(s1, s2, this.collisionMatrix.getDetector(s1.geom.type, s2.geom.type));
      c.contactConstraint.maxRestitution = this.maxRestitutionFlag;
      this.numContacts++;
    }
  }

  private destroyOutdatedContacts(): void {
    // 宽相是否只收集新出现的重叠对（BVH 是，暴力法不是）
    const incremental = this.broadPhase.incremental;
    let c = this.contactList;
    while (c !== null) {
      const next = c.next;
      if (c.latest) {
        // 这一步宽相又收集到了：标记成旧的，留到下一步
        c.latest = false;
        c.shouldBeSkipped = false;
      } else if (!incremental) {
        // 暴力法会收集所有重叠的对，没被收集到说明已经分开
        this.destroyContact(c);
      } else {
        const s1 = c.s1!, s2 = c.s2!, r1 = s1.rigidBody!, r2 = s2.rigidBody!;
        const active1 = !r1.sleeping && r1.type !== RigidBodyType.STATIC;
        const active2 = !r2.sleeping && r2.type !== RigidBodyType.STATIC;
        if (!active1 && !active2) {
          // 两个刚体都不活动：跳过
          c.shouldBeSkipped = true;
        } else if (!this.broadPhase.isOverlapping(s1.proxy!, s2.proxy!) || !this.shouldCollide(s1, s2)) {
          // 代理已经分开，或者不该碰撞
          this.destroyContact(c);
        } else {
          // 代理还重叠，但形状的包围盒可能已经分开，分开时跳过窄相
          const a1 = s1.aabb, a2 = s2.aabb;
          c.shouldBeSkipped = !(
            a1.minX < a2.maxX && a1.maxX > a2.minX &&
            a1.minY < a2.maxY && a1.maxY > a2.minY &&
            a1.minZ < a2.maxZ && a1.maxZ > a2.minZ
          );
        }
      }
      c = next;
    }
  }

  private shouldCollide(s1: Shape, s2: Shape): boolean {
    const r1 = s1.rigidBody!, r2 = s2.rigidBody!;
    // 同一个刚体上的形状
    if (r1 === r2) return false;
    // 两个都不是动态刚体
    if (r1.type !== RigidBodyType.DYNAMIC && r2.type !== RigidBodyType.DYNAMIC) return false;
    // 碰撞分组过滤
    if ((s1.collisionGroup & s2.collisionMask) === 0 || (s2.collisionGroup & s1.collisionMask) === 0) return false;
    // 用关节连着、并且关节不允许碰撞：在关节较少的刚体上找
    let jl, other;
    if (r1.numJointLinks < r2.numJointLinks) {
      jl = r1.jointLinkList;
      other = r2;
    } else {
      jl = r2.jointLinkList;
      other = r1;
    }
    for (; jl !== null; jl = jl.next) {
      if (jl.other === other && !jl.joint.allowCollision) return false;
    }
    return true;
  }
}
