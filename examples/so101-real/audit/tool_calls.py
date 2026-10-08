"""List every tool call of the solver transcript: line number, tool, compact input."""
import json, sys
path = sys.argv[1]
for n, line in enumerate(open(path), 1):
    d = json.loads(line)
    if d.get("type") == "user" and n == 1:
        c = d["message"]["content"]
        print("PROMPT:", c if isinstance(c, str) else json.dumps(c)[:6000]); continue
    if d.get("type") != "assistant": continue
    for c in d["message"].get("content", []):
        if c.get("type") == "tool_use":
            inp = c["input"]
            s = inp.get("command") or inp.get("file_path") or inp.get("pattern") or json.dumps(inp)
            if c["name"] in ("Write", "Edit"): s = inp.get("file_path")
            print(f"L{n} {c['name']}: {s}")
