// Minecraft 脚本环境提供的 console 接口。
export {};

declare global {
  const console: {
    log(...data: unknown[]): void;
    info(...data: unknown[]): void;
    warn(...data: unknown[]): void;
    error(...data: unknown[]): void;
  };
}
