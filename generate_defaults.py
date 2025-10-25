#!/usr/bin/env python3
import re
from datetime import datetime

INPUT_FILE = "GSB_Settings.txt"
OUTPUT_FILE = "GSB_Defaults.h"

def parse_settings(text):
    """Parse $<id>=<value> lines and optional trailing comments."""
    settings = []
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue

        # Split off inline comments: support ';' or '//' as comment marker
        comment = ""
        if ";" in line:
            line, comment = line.split(";", 1)
        elif "//" in line:
            line, comment = line.split("//", 1)

        match = re.match(r"\$(\d+)\s*=\s*([\-0-9.]+)", line.strip())
        if match:
            sid, val = match.groups()
            settings.append({
                "id": int(sid),
                "val": val.strip(),
                "comment": comment.strip()
            })
    return sorted(settings, key=lambda s: s["id"])

def generate_header(settings):
    lines = []
    lines.append("// ================================================================")
    lines.append("// GSB_Defaults.h")
    lines.append("// Auto-generated from GSB_Settings.txt")
    lines.append(f"// Generated on {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append("// ================================================================")
    lines.append("#pragma once\n")

    for s in settings:
        sid = s["id"]
        val = s["val"]
        comment = s["comment"]

        # Add 'f' suffix for floats (only if not integer-like)
        if "." in val and not val.lower().endswith("f"):
            val = f"{val}f"

        comment_str = f" // {comment}" if comment else ""
        lines.append(f"#define GRBL_DEFAULT_{sid:<4} {val:<10}{comment_str}")

    lines.append("\n// End of auto-generated defaults")
    lines.append("// ================================================================")
    return "\n".join(lines) + "\n"

def main():
    with open(INPUT_FILE, "r", encoding="utf-8") as f:
        text = f.read()

    settings = parse_settings(text)
    header = generate_header(settings)

    with open(OUTPUT_FILE, "w", encoding="utf-8") as f:
        f.write(header)

    print(f"✅ Generated {OUTPUT_FILE} with {len(settings)} settings.")

if __name__ == "__main__":
    main()
