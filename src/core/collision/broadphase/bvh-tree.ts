// 移植自 OimoPhysics src/oimo/collision/broadphase/bvh/{BvhNode,BvhProxy,BvhTree,BvhStrategy}.hx（MIT，见 Credits.md）
// 使用 SIMPLE 插入策略，不执行平衡旋转或整树重建。
import { Proxy } from "./proxy.js";

export class BvhNode {
  /** 对象池链表 */
  next: BvhNode | null = null;
  child0: BvhNode | null = null;
  child1: BvhNode | null = null;
  /** 自己是父节点的第几个子节点（0 或 1） */
  childIndex = 0;
  parent: BvhNode | null = null;
  /** 叶子为 0 */
  height = 0;
  /** 只有叶子才有 */
  proxy: BvhProxy | null = null;
  minX = 0;
  minY = 0;
  minZ = 0;
  maxX = 0;
  maxY = 0;
  maxZ = 0;

  setChild(index: number, child: BvhNode): void {
    if (index === 0) this.child0 = child;
    else this.child1 = child;
    child.parent = this;
    child.childIndex = index;
  }
}

export class BvhProxy extends Proxy {
  leaf: BvhNode | null = null;
  /** 这一步移出了加厚包围盒，collectPairs() 时要在树里重新插入 */
  moved = false;
}

export class BvhTree {
  root: BvhNode | null = null;
  private nodePool: BvhNode | null = null;

  /** 为代理创建叶子并插入树 */
  insertProxy(proxy: BvhProxy): void {
    const leaf = this.pick();
    leaf.proxy = proxy;
    proxy.leaf = leaf;
    leaf.minX = proxy.minX;
    leaf.minY = proxy.minY;
    leaf.minZ = proxy.minZ;
    leaf.maxX = proxy.maxX;
    leaf.maxY = proxy.maxY;
    leaf.maxZ = proxy.maxZ;
    this.insertLeaf(leaf);
  }

  /** 从树里删掉代理的叶子 */
  deleteProxy(proxy: BvhProxy): void {
    const leaf = proxy.leaf!;
    this.deleteLeaf(leaf);
    proxy.leaf = null;
    this.pool(leaf);
  }

  private insertLeaf(leaf: BvhNode): void {
    if (this.root === null) {
      this.root = leaf;
      return;
    }
    // 从根往下找插入位置：每次走向中心离叶子更近的子节点
    const cX = leaf.minX + leaf.maxX, cY = leaf.minY + leaf.maxY, cZ = leaf.minZ + leaf.maxZ;
    let sibling = this.root;
    while (sibling.height > 0) {
      const c1 = sibling.child0!, c2 = sibling.child1!;
      const d1X = c1.minX + c1.maxX - cX, d1Y = c1.minY + c1.maxY - cY, d1Z = c1.minZ + c1.maxZ - cZ;
      const d2X = c2.minX + c2.maxX - cX, d2Y = c2.minY + c2.maxY - cY, d2Z = c2.minZ + c2.maxZ - cZ;
      const dist1 = d1X * d1X + d1Y * d1Y + d1Z * d1Z;
      const dist2 = d2X * d2X + d2Y * d2Y + d2Z * d2Z;
      sibling = dist1 < dist2 ? c1 : c2;
    }
    // 新建一个节点，作为叶子和 sibling 的共同父节点
    const parent = sibling.parent;
    const node = this.pick();
    if (parent === null) {
      this.root = node;
    } else {
      parent.setChild(sibling.childIndex, node);
    }
    node.setChild(sibling.childIndex, sibling);
    node.setChild(sibling.childIndex ^ 1, leaf);
    this.fixUpward(node);
  }

  private deleteLeaf(leaf: BvhNode): void {
    if (this.root === leaf) {
      this.root = null;
      return;
    }
    const parent = leaf.parent!;
    const sibling = leaf.childIndex === 0 ? parent.child1! : parent.child0!;
    const grandParent = parent.parent;
    if (grandParent === null) {
      sibling.parent = null;
      sibling.childIndex = 0;
      this.root = sibling;
      this.pool(parent);
      return;
    }
    sibling.parent = grandParent;
    grandParent.setChild(parent.childIndex, sibling);
    this.pool(parent);
    this.fixUpward(grandParent);
  }

  /** 从 node 往上逐层更新高度和包围盒 */
  private fixUpward(start: BvhNode): void {
    for (let node: BvhNode | null = start; node !== null; node = node.parent) {
      const c1 = node.child0!, c2 = node.child1!;
      const h1 = c1.height, h2 = c2.height;
      node.height = (h1 > h2 ? h1 : h2) + 1;
      node.minX = c1.minX < c2.minX ? c1.minX : c2.minX;
      node.minY = c1.minY < c2.minY ? c1.minY : c2.minY;
      node.minZ = c1.minZ < c2.minZ ? c1.minZ : c2.minZ;
      node.maxX = c1.maxX > c2.maxX ? c1.maxX : c2.maxX;
      node.maxY = c1.maxY > c2.maxY ? c1.maxY : c2.maxY;
      node.maxZ = c1.maxZ > c2.maxZ ? c1.maxZ : c2.maxZ;
    }
  }

  private pool(node: BvhNode): void {
    node.childIndex = 0;
    node.child0 = null;
    node.child1 = null;
    node.parent = null;
    node.height = 0;
    node.proxy = null;
    node.next = this.nodePool;
    this.nodePool = node;
  }

  private pick(): BvhNode {
    const node = this.nodePool;
    if (node === null) return new BvhNode();
    this.nodePool = node.next;
    node.next = null;
    return node;
  }
}
