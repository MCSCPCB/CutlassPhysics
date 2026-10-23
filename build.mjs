// 生成单文件 ESM；类型声明由 tsconfig.build.json 生成。
import { rmSync } from "node:fs";
import { fileURLToPath, URL } from "node:url";
import { build } from "esbuild";

const root = fileURLToPath(new URL(".", import.meta.url));

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
    js: "/*! cutlass-physics | Portions derived from OimoPhysics (c) 2017 saharan | MIT License | See THIRD-PARTY-NOTICES.txt */",
  },
});
