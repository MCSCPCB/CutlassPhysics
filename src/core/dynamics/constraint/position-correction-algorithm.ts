// 移植自 OimoPhysics src/oimo/dynamics/constraint/PositionCorrectionAlgorithm.hx（MIT，见 THIRD-PARTY-NOTICES.txt）

/** 约束的位置修正算法，编号和 Oimo 一致 */
export const PositionCorrectionAlgorithm = {
  /** 将穿透转换为目标分离速度，可能引入额外动能。 */
  BAUMGARTE: 0,
  /** 速度求解之后单独解一遍“伪速度”把物体推开，不影响真实速度 */
  SPLIT_IMPULSE: 1,
  /** 非线性高斯-赛德尔，直接修正位置和朝向。 */
  NGS: 2,
} as const;
