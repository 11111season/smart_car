"""Quick analysis of why car marker flips to beacon."""
import sys

fname = sys.argv[1] if len(sys.argv) > 1 else "d:/UsersASUSDesktop/Vision and Drones/serial_log_20260715_000057.csv"

tracks = {}
with open(fname) as fh:
    for line in fh:
        line = line.strip()
        if not line or line.startswith("time,") or line.startswith("BEACON:") or line.startswith("["):
            continue
        parts = line.split(",")
        if len(parts) < 3:
            continue
        try:
            frame_id = int(parts[1])
            blob_num = int(parts[2])
        except:
            continue
        idx = 3
        for _ in range(min(blob_num, 5)):
            if idx + 9 > len(parts):
                break
            try:
                tid = int(parts[idx])
                area = int(parts[idx + 1])
                cr = int(parts[idx + 2])
                asp = int(parts[idx + 3])
                raw_b = int(parts[idx + 4])
                raw_m = int(parts[idx + 5])
                filt_b = int(parts[idx + 6])
                filt_m = int(parts[idx + 7])
                typ = int(parts[idx + 8])
            except:
                pass
            idx += 9
            if tid not in tracks:
                tracks[tid] = []
            tracks[tid].append((frame_id, area, cr, asp, raw_b, raw_m, filt_b, filt_m, typ))

for tid in [133, 138, 137, 142]:
    if tid not in tracks:
        continue
    data = tracks[tid]
    print(f"\n=== Track {tid} ({len(data)} frames) ===")
    # Show first 15
    for f, a, cr, asp, rb, rm, fb, fm, t in data[:15]:
        tag = ["UNK", "BEA", "CAR", "???"][t if t < 4 else 3]
        print(f"  f={f:>5d} area={a:>3d} cr={cr:>3d} asp={asp:>3d} rb={rb:>3d} rm={rm:>3d} fb={fb:>3d} fm={fm:>3d} {tag}")
    # Show switching region
    switches = [(f, a, cr, asp, rb, rm, fb, fm, t) for f, a, cr, asp, rb, rm, fb, fm, t in data if t != 2 and t != 1]
    if switches:
        print(f"  Non-standard type frames ({len(switches)}):")
        for f, a, cr, asp, rb, rm, fb, fm, t in switches[:10]:
            print(f"    f={f:>5d} area={a:>3d} cr={cr:>3d} asp={asp:>3d} rb={rb:>3d} rm={rm:>3d} fb={fb:>3d} fm={fm:>3d} type={t}")
