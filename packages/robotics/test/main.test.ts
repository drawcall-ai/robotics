import { execFileSync } from "node:child_process";
import { fileURLToPath } from "node:url";
import { expect, it } from "vitest";

// The bin as published, so this needs a build first, as `pnpm check` does.
const bin = fileURLToPath(new URL("../dist/cli/main.js", import.meta.url));

function usage(invocation: string | undefined): string | undefined {
  const env = { ...process.env, DRAWCALL_INVOCATION: invocation };
  const help = execFileSync(process.execPath, [bin, "--help"], {
    env,
    encoding: "utf8",
  });
  return help.split("\n")[0];
}

it("names itself as drawcall forwards to it, and as npx runs it otherwise", () => {
  expect(usage("drawcall robotics")).toBe(
    "Usage: drawcall robotics [options] [command]",
  );
  expect(usage(undefined)).toBe(
    "Usage: npx @drawcall/robotics [options] [command]",
  );
  expect(usage("")).toBe("Usage: npx @drawcall/robotics [options] [command]");
});
