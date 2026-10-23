// 移植自 OimoPhysics src/oimo/common/Setting.hx（MIT，见 THIRD-PARTY-NOTICES.txt）
// 模拟参数，默认值与 OimoPhysics 一致。
import { PositionCorrectionAlgorithm } from "./dynamics/constraint/position-correction-algorithm.js";

/** 每个接触流形最多保留的接触点数 */
export const MAX_MANIFOLD_POINTS = 4;

/** 允许的穿透量（米）。BoxBox 检测用它给面轴加偏置，减少接触法线来回跳 */
export const LINEAR_SLOP = 0.005;

/** 分离距离在这个范围内（米）的接触点仍然保留 */
export const CONTACT_PERSISTENCE_THRESHOLD = 0.05;

/** GJK 默认外边距（米） */
export const DEFAULT_GJK_MARGIN = 0.05;
/** GJK 是否从上一步的结果开始迭代（原版 enableGJKCaching） */
export const ENABLE_GJK_CACHING = true;

export const DEFAULT_FRICTION = 0.2;
export const DEFAULT_RESTITUTION = 0.2;
/** 默认密度（kg/m³） */
export const DEFAULT_DENSITY = 1;
export const DEFAULT_COLLISION_GROUP = 1;
export const DEFAULT_COLLISION_MASK = 1;

/** 每步最大位移（米），超过时按比例压低速度 */
export const MAX_TRANSLATION_PER_STEP = 20;
/** 每步最大转角，单位为弧度，使用上游的截断 π 常量。 */
export const MAX_ROTATION_PER_STEP = 3.14159265358979;

/** 速度低于这些阈值并持续 SLEEPING_TIME_THRESHOLD 秒后刚体进入休眠 */
export const SLEEPING_VELOCITY_THRESHOLD = 0.2;
export const SLEEPING_ANGULAR_VELOCITY_THRESHOLD = 0.5;
export const SLEEPING_TIME_THRESHOLD = 1.0;

/** BVH 代理包围盒的加厚余量，单位为米。形状超出此范围时更新树。 */
export const BVH_PROXY_PADDING = 0.1;

/** 这一步移动过的代理占比低于它时，BVH 只为移动过的代理找新的重叠对，否则整棵树重新碰撞 */
export const BVH_INCREMENTAL_COLLISION_THRESHOLD = 0.45;

/** 撞击的法向相对速度超过它（m/s）才会反弹 */
export const CONTACT_ENABLE_BOUNCE_THRESHOLD = 0.5;
/** Baumgarte 位置修正：每步把超出 LINEAR_SLOP 的穿透推出去的比例 */
export const VELOCITY_BAUMGARTE = 0.2;
/** split impulse 位置修正：每次迭代推出去的比例 */
export const POSITION_SPLIT_IMPULSE_BAUMGARTE = 0.4;
/** 穿透超过它（米）时改用 ALTERNATIVE 位置修正算法 */
export const ALTERNATIVE_POSITION_CORRECTION_DEPTH_THRESHOLD = 0.05;
/** 穿透较浅时的位置修正算法 */
export const DEFAULT_CONTACT_POSITION_CORRECTION_ALGORITHM: number = PositionCorrectionAlgorithm.BAUMGARTE;
/** 穿透较深时的位置修正算法 */
export const ALTERNATIVE_CONTACT_POSITION_CORRECTION_ALGORITHM: number = PositionCorrectionAlgorithm.SPLIT_IMPULSE;
/** 默认重力加速度（m/s²），只有 y 分量 */
export const DEFAULT_GRAVITY_Y = -9.80665;
/** NGS 位置修正：每次迭代推出去的比例 */
export const POSITION_NGS_BAUMGARTE = 1.0;

// 关节
/** 关节默认的位置修正算法 */
export const DEFAULT_JOINT_POSITION_CORRECTION_ALGORITHM: number = PositionCorrectionAlgorithm.BAUMGARTE;
/** 关节热启动的冲量保留比例，分别用于 Baumgarte 和其他算法。 */
export const JOINT_WARM_STARTING_FACTOR_FOR_BAUMGARTE = 0.8;
export const JOINT_WARM_STARTING_FACTOR = 0.95;
/** 弹簧阻尼的最小阻尼比 */
export const MIN_SPRING_DAMPER_DAMPING_RATIO = 1e-6;
/** 一个关节最多几行约束（3 个平移 + 3 个旋转） */
export const MAX_JACOBIAN_ROWS = 6;
/** 关节角度限制的容差，单位为弧度，对应上游截断 π 计算的 1°。 */
export const ANGULAR_SLOP = 0.017453292519943278;
/** 布娃娃关节摆动锥的最小半角（弧度），设得更小时按这个算 */
export const MIN_RAGDOLL_MAX_SWING_ANGLE = 1e-6;
/** 直接求解器判断冲量是否超出上下限、是否满足互补条件时的容差 */
export const DIRECT_MLCP_SOLVER_EPS = 1e-9;
