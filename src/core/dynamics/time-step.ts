// 移植自 OimoPhysics src/oimo/dynamics/TimeStep.hx（MIT，见 THIRD-PARTY-NOTICES.txt）

/** 一步的时间信息 */
export class TimeStep {
  dt = 0;
  invDt = 0;
  /** 这一步和上一步 dt 的比值，热启动时按它缩放上一步的冲量 */
  dtRatio = 1;
}
