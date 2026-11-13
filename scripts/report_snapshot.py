#!/usr/bin/env python3
"""scripts/report_snapshot.py : render tool-report excerpts as a terminal-style PNG for the README.
usage: report_snapshot.py out.png "title" file1[:start-end[,start-end...]] [file2[:...]] ...
Line ranges are 1-based and inclusive; a file without a range is taken whole. Text is copied verbatim from the
reports (no editing), each file under a header line with its path. Needs Pillow (/apps/miniconda/bin/python3)."""
import os, sys
from PIL import Image, ImageDraw, ImageFont

FONT_PATHS = ["/usr/share/fonts/dejavu-sans-mono-fonts/DejaVuSansMono.ttf",
              "/usr/share/fonts/dejavu/DejaVuSansMono.ttf",
              "/usr/share/fonts/liberation-mono/LiberationMono-Regular.ttf"]
def font(size):
    for p in FONT_PATHS:
        if os.path.exists(p): return ImageFont.truetype(p, size)
    return ImageFont.load_default()

out, title, specs = sys.argv[1], sys.argv[2], sys.argv[3:]
BG, FG, DIM, HDR, ACC = (24, 26, 33), (220, 223, 228), (130, 136, 150), (110, 180, 255), (255, 196, 90)
f, fb = font(15), font(17)
lines = [("title", title)]
for sp in specs:
    path, _, rng = sp.partition(":")
    txt = open(path, errors="replace").read().expandtabs(8).split("\n")
    sel = []
    if rng:
        for r in rng.split(","):
            a, b = (int(x) for x in r.split("-"))
            if sel: sel.append(("dim", "   ..."))
            sel += [("text", t) for t in txt[a - 1:b]]
    else:
        sel = [("text", t) for t in txt]
    lines.append(("hdr", "$ cat " + os.path.relpath(path, os.path.join(os.path.dirname(__file__), ".."))))
    lines += sel
    lines.append(("text", ""))
cw = f.getbbox("M")[2]; lh = 20
W = max(len(t) for _, t in lines) * cw + 48
H = len(lines) * lh + 60
img = Image.new("RGB", (W, H), BG); d = ImageDraw.Draw(img)
d.rectangle([0, 0, W, 30], fill=(38, 41, 52))
for i, c in enumerate([(255, 95, 86), (255, 189, 46), (39, 201, 63)]): d.ellipse([12 + 20 * i, 9, 24 + 20 * i, 21], fill=c)
y = 40
for kind, t in lines:
    if kind == "title": d.text((84, 5), t, font=fb, fill=FG); continue
    col = {"hdr": HDR, "dim": DIM}.get(kind, FG)
    if kind == "text" and any(k in t for k in ("Critical Path Slack", "Total Negative Slack", "No. of Violating Paths", "slack (MET)",
                                                "slack (VIOLATED)", "Total cell area", "Total Power", "Total Dynamic Power", "Cell Leakage")):
        col = ACC
    d.text((24, y), t, font=f, fill=col); y += lh
img.save(out)
print("wrote", out, img.size)
