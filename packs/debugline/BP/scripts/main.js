import { CommandPermissionLevel, CustomCommandParamType, CustomCommandStatus, system, world } from "@minecraft/server";
import { PhysicsWorldStore } from "./physics.js";
import { SCENES, SCENE_NAMES } from "./scenes.js";
import { Renderer } from "./renderer.js";

const sessions = new Map();

function storeFor(dimensionId) {
  return new PhysicsWorldStore(`debugline:${dimensionId}`);
}

function sessionFor(dimension) {
  const existing = sessions.get(dimension.id);
  if (existing) return existing;

  const persistence = storeFor(dimension.id).open(dimension, {
    terrain: {}, stepMode: "manual",
    encodeUserData: body => typeof body.userData === "string" ? body.userData : body.type,
    onError: error => console.warn(`[cutlass] ${String(error)}`),
  });
  const physics = persistence.world;
  const renderer = new Renderer(dimension);
  try {
    // 线框由已恢复刚体的形状和颜色重建。
    const savedBodies = physics.createSnapshot().bodies;
    physics.getBodies().forEach((body, i) => {
      renderer.add(body, savedBodies[i].options.shapes, body.userData ?? body.type);
    });
    renderer.update();
    const session = { physics, persistence, renderer };
    sessions.set(dimension.id, session);
    return session;
  } catch (error) {
    persistence.close(false);
    renderer.clear();
    throw error;
  }
}

function closeSession(dimensionId) {
  const session = sessions.get(dimensionId);
  if (!session) return;
  sessions.delete(dimensionId);
  session.persistence.close(false);
  session.renderer.clear();
}

function loadScene(player, name) {
  if (!player.isValid) return;
  const dimension = player.dimension;
  const added = [];
  let session;
  try {
    if (name === "clear") {
      closeSession(dimension.id);
      storeFor(dimension.id).delete();
      return;
    }
    session = sessionFor(dimension);
    const { physics, renderer } = session;
    const direction = player.getViewDirection();
    const length = Math.hypot(direction.x, direction.z);
    const forward = length > 1e-6
      ? { x: direction.x / length, y: 0, z: direction.z / length }
      : { x: 0, y: 0, z: 1 };
    SCENES[name]({
      world: physics,
      origin: player.location,
      forward,
      add(shapes, { color, ...options }) {
        const label = color ?? options.type ?? "dynamic";
        const body = physics.createBody({ shapes, ...options, userData: label });
        added.push(body);
        renderer.add(body, shapes, label);
        return body;
      },
    });
    renderer.update();
  } catch (error) {
    for (const body of added) if (body.isValid) body.remove();
    session?.renderer.update();
    console.warn(`[cutlass] ${String(error)}`);
    if (player.isValid) player.sendMessage(name === "clear" ? "场景清除失败。" : "场景加载失败。");
    return;
  }
  try {
    session.persistence.save();
  } catch (error) {
    console.warn(`[cutlass] ${String(error)}`);
    if (player.isValid) player.sendMessage("场景已创建，保存失败。");
  }
}

system.beforeEvents.startup.subscribe(({ customCommandRegistry }) => {
  customCommandRegistry.registerEnum("cutlass:scene", [...SCENE_NAMES, "clear"]);
  customCommandRegistry.registerCommand({
    name: "cutlass:cutlass",
    description: "显示或清除物理场景",
    permissionLevel: CommandPermissionLevel.GameDirectors,
    cheatsRequired: true,
    optionalParameters: [{ name: "cutlass:scene", type: CustomCommandParamType.Enum }],
  }, (origin, name = "drop") => {
    const player = origin.sourceEntity?.typeId === "minecraft:player" ? origin.sourceEntity : origin.initiator;
    if (player?.typeId !== "minecraft:player" || !player.isValid) {
      return { status: CustomCommandStatus.Failure, message: "此命令需要玩家执行。" };
    }
    if (name !== "clear" && !Object.prototype.hasOwnProperty.call(SCENES, name)) {
      return { status: CustomCommandStatus.Failure, message: "未知场景。" };
    }
    // 命令回调处于受限执行环境，场景修改延后执行。
    system.run(() => loadScene(player, name));
    return { status: CustomCommandStatus.Success };
  });
});

system.run(() => {
  for (const id of ["overworld", "nether", "the_end"]) {
    try {
      const dimension = world.getDimension(id);
      if (storeFor(dimension.id).read() !== undefined) sessionFor(dimension);
    } catch (error) {
      console.warn(`[cutlass] ${String(error)}`);
    }
  }
});

// 每个维度共用一个物理世界。
system.runInterval(() => {
  for (const [dimensionId, session] of sessions) {
    try {
      session.physics.step();
      session.renderer.update();
    } catch (error) {
      closeSession(dimensionId);
      console.warn(`[cutlass] ${String(error)}`);
    }
  }
}, 1);
