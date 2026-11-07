# python3 -I final_cond_table.py <bigm_cond file>...: per K and cfg, verdicts, passed/dropped cuts and nodes vs the plain covers.
import re, sys, collections
for path in sys.argv[1:]:
    base = {}; plain = {}; rows = collections.defaultdict(list)
    for line in open(path, errors='replace'):
        m = re.match(r'^CONDBASE \S+ (\S+): .* nodes=(\S+) ', line)
        if m: base[m.group(1)] = m.group(2); continue
        m = re.match(r'^CONDPLAIN \S+ (\S+): vs_oracle=(\S+) .* nodes=(\S+) .*prep\(offered=(\d+) passed=(\d+) dropped=(\d+)', line)
        if m: plain[m.group(1)] = (m.group(2), m.group(3), m.group(5)); continue
        m = re.match(r'^COND \S+ (\S+) K=(\S+) cfg=(\S+): vs_oracle=(\S+) \| status=(\S+) .* nodes=(\S+) t=(\S+)s calls=\S+ prep\(offered=(\d+) passed=(\d+) dropped=(\d+)', line)
        if m:
            inst, K, cfg, vo, st, nodes, t, off, pas, drop = m.groups()
            rows[(K, cfg)].append((inst, vo, nodes, pas, drop))
    print('#####', path)
    print('  base nodes', base, '| plain covers (verdict, nodes, passed)', plain)
    for (K, cfg), v in sorted(rows.items(), key=lambda kv: (float(kv[0][0]), kv[0][1])):
        verd = collections.Counter(x[1] for x in v)
        print(f"  K={K:6s} {cfg:8s} {dict(verd)} nodes={[x[2] for x in v]} passed={[x[3] for x in v]} dropped={[x[4] for x in v]}")
