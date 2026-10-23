import { system, world } from "@minecraft/server";
import { PhysicsWorldStore } from "./physics.js";
import { Renderer, ENTITY_TYPE } from "./renderer.js";
import { Structures } from "./structures.js";
import { DragController, heldItem } from "./drag.js";

const sessions = new Map();
const selections = new Map();
const lastUse = new Map();
const dragging = new DragController();
const storeFor = id => new PhysicsWorldStore(`physics_block:${id}`);

function report(error, player) {
  console.warn(`[cutlass] ${String(error)}`);
  if (player?.isValid) player.sendMessage(String(error.message ?? error));
}

function sessionFor(dimension) {
  let session = sessions.get(dimension.id);
  if (session) return session;
  const persistence = storeFor(dimension.id).open(dimension, {
    terrain: {}, stepMode: "manual",
    encodeUserData: body => body.userData,
    onError: error => report(error),
  });
  try {
    const physics = persistence.world;
    const renderer = new Renderer(dimension);
    const structures = new Structures(dimension, physics, renderer);
    structures.restore();
    session = { physics, persistence, renderer, structures };
    sessions.set(dimension.id, session);
    return session;
  } catch (error) { persistence.close(false); throw error; }
}

function save(session) {
  try { session.persistence.save(); }
  catch (error) { report(error); }
}

function use(player, typeId, location) {
  if (!player.isValid || heldItem(player) !== typeId) return;
  if (typeId === "minecraft:slime_ball") {
    const session = sessions.get(player.dimension.id);
    if (session) dragging.toggle(player, session);
    return;
  }
  location ??= player.getBlockFromViewDirection({ maxDistance: 6, includeLiquidBlocks: false })?.block.location;
  if (!location) return;
  const first = selections.get(player.id);
  if (!first || first.dimensionId !== player.dimension.id) {
    selections.set(player.id, { dimensionId: player.dimension.id, location: { ...location } });
    player.onScreenDisplay.setActionBar(`起点：${location.x}, ${location.y}, ${location.z}`);
    return;
  }
  const session = sessionFor(player.dimension);
  const count = session.structures.assemble(first.location, location);
  selections.delete(player.id);
  save(session);
  player.onScreenDisplay.setActionBar(`已物理化 ${count} 个方块`);
}

function queueUse(player, typeId, location) {
  if (system.currentTick - (lastUse.get(player.id) ?? -100) < 7) return;
  lastUse.set(player.id, system.currentTick);
  const dimensionId = player.dimension.id;
  system.run(() => {
    if (!player.isValid || player.dimension.id !== dimensionId) return;
    try { use(player, typeId, location); }
    catch (error) { report(error, player); }
  });
}

const isTool = typeId => typeId === "minecraft:stick" || typeId === "minecraft:slime_ball";

world.beforeEvents.playerInteractWithBlock.subscribe(event => {
  if (!isTool(event.itemStack?.typeId)) return;
  event.cancel = true;
  if (event.isFirstEvent === false) return;
  queueUse(event.player, event.itemStack.typeId, { ...event.block.location });
});

world.beforeEvents.itemUse.subscribe(event => {
  if (!isTool(event.itemStack.typeId)) return;
  event.cancel = true;
  queueUse(event.source, event.itemStack.typeId);
});

world.beforeEvents.playerInteractWithEntity.subscribe(event => {
  if (event.target.typeId !== ENTITY_TYPE) return;
  event.cancel = true;
  if (event.itemStack?.typeId === "minecraft:slime_ball") queueUse(event.player, event.itemStack.typeId);
});

world.beforeEvents.playerBreakBlock.subscribe(event => {
  if (!isTool(event.itemStack?.typeId)) return;
  event.cancel = true;
  queueUse(event.player, event.itemStack.typeId, { ...event.block.location });
});

function breakEntity(entity) {
  for (const session of sessions.values()) {
    const entry = session.renderer.byEntity.get(entity.id);
    if (entry) session.structures.breakCell(entry.cell.id);
  }
}

world.afterEvents.entityHitEntity.subscribe(({ damagingEntity, hitEntity }) => {
  if (hitEntity.typeId !== ENTITY_TYPE) return;
  if (damagingEntity.typeId === "minecraft:player" && isTool(heldItem(damagingEntity))) {
    if (heldItem(damagingEntity) === "minecraft:slime_ball") queueUse(damagingEntity, "minecraft:slime_ball");
    return;
  }
  breakEntity(hitEntity);
});

world.afterEvents.entityDie.subscribe(({ deadEntity }) => {
  dragging.release(deadEntity.id);
  selections.delete(deadEntity.id);
  breakEntity(deadEntity);
});

world.afterEvents.playerLeave.subscribe(({ playerId }) => {
  dragging.release(playerId);
  selections.delete(playerId);
  lastUse.delete(playerId);
});

world.afterEvents.playerDimensionChange.subscribe(({ player }) => {
  dragging.release(player.id);
  selections.delete(player.id);
  lastUse.delete(player.id);
});

world.afterEvents.entityLoad.subscribe(({ entity }) => {
  if (entity.typeId !== ENTITY_TYPE) return;
  system.run(() => {
    if (!entity.isValid) return;
    try { sessionFor(entity.dimension).renderer.attach(entity); }
    catch (error) { report(error); }
  });
});

system.run(() => {
  for (const name of ["overworld", "nether", "the_end"]) {
    try {
      const dimension = world.getDimension(name);
      if (storeFor(dimension.id).read() !== undefined) sessionFor(dimension);
    } catch (error) { report(error); }
  }
});

system.runInterval(() => {
  try { dragging.update(); }
  catch (error) { report(error); }
  for (const session of sessions.values()) {
    try {
      if (session.structures.flushBreaks()) save(session);
      session.physics.step();
      for (const record of session.structures.records.values()) session.renderer.updateRecord(record);
    } catch (error) { report(error); }
  }
}, 1);
