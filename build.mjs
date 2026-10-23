// 生成单文件 ESM；类型声明由 tsconfig.build.json 生成。
import { existsSync, mkdirSync, readFileSync, rmSync, writeFileSync } from "node:fs";
import { fileURLToPath, URL } from "node:url";
import { build } from "esbuild";

const root = fileURLToPath(new URL(".", import.meta.url));
const args = process.argv.slice(2);
if (args.length > 1 || args.some(arg => !["--sync-pack", "--check-pack"].includes(arg))) {
  throw new Error("用法：node build.mjs [--sync-pack | --check-pack]");
}

rmSync(new URL("./dist", import.meta.url), { recursive: true, force: true });

await build({
  absWorkingDir: root,
  entryPoints: ["src/index.ts"],
  outfile: "dist/index.js",
  bundle: true,
  format: "esm",
  platform: "neutral",
  mainFields: ["module", "main"],
  target: "es2020",
  // Minecraft 模块由游戏运行时提供。
  external: ["@minecraft/*"],
  sourcemap: true,
  logLevel: "info",
  banner: {
    js: "/*! cutlass-physics | Portions derived from OimoPhysics (c) 2017 saharan | MIT License | See Credits.md */",
  },
});

if (args.length) {
  const physics = readFileSync(new URL("./dist/index.js", import.meta.url), "utf8")
    .replace(/\n\/\/# sourceMappingURL=.*\n?$/, "\n");
  const files = new Map([
    ["scripts/physics.js", physics],
    ["LICENSE", readFileSync(new URL("./LICENSE", import.meta.url), "utf8")],
    ["Credits.md", readFileSync(new URL("./Credits.md", import.meta.url), "utf8")],
  ]);
  for (const name of ["debugline", "physics_block"]) {
    const pack = new URL(`./packs/${name}/BP/`, import.meta.url);
    for (const [file, content] of files) {
      const target = new URL(file, pack);
      const text = content.replaceAll("\r\n", "\n");
      if (args[0] === "--sync-pack") {
        // 仅同步库文件及许可文件，示例脚本独立维护。
        mkdirSync(new URL(".", target), { recursive: true });
        writeFileSync(target, text);
      } else if (!existsSync(target) || readFileSync(target, "utf8").replaceAll("\r\n", "\n") !== text) {
        throw new Error(`行为包文件未同步：${name}/${file}；运行 node build.mjs --sync-pack 后重试。`);
      }
    }
  }
}
