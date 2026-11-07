# python3 -I final_harm_table.py <harm file>...: nodes / time / dropped per instance x (sep, cfg).
import re, sys, collections, math
for path in sys.argv[1:]:
    rows = collections.OrderedDict(); base = {}
    for line in open(path, errors='replace'):
        m = re.match(r'^HARMBASE \S+ (\S+): .*vs_oracle=(\S+) \| status=(\S+) value=(\S+) dual=\S+ nodes=(\S+) t=(\S+)s', line)
        if m: base[m.group(1)] = (m.group(5), m.group(6), m.group(2)); continue
        m = re.match(r'^HARM \S+ (\S+) sep=(\S+) cfg=(\S+): vs_oracle=(\S+) \| status=(\S+) value=(\S+) dual=\S+ nodes=(\S+) t=(\S+)s calls=(\S+) prep\(offered=(\d+) passed=(\d+) dropped=(\d+) rescaled=(\d+)', line)
        if m:
            inst, sep, cfg, vo, st, val, nodes, t, calls, off, pas, drop, resc = m.groups()
            rows.setdefault((inst, sep), {})[cfg] = (vo, st.split('{')[0], nodes, t, drop, pas, resc)
    print('#####', path)
    bad = [ (k, c, v) for k, d in rows.items() for c, v in d.items() if v[0] != 'OK']
    print('non-OK:', bad if bad else 'none')
    for (inst, sep), d in rows.items():
        b = base.get(inst, ('?', '?', '?'))
        cells = ' '.join(f"{c}:{v[2]}n/{v[3]}s/d{v[4]}/p{v[5]}/r{v[6]}" + ('' if v[0]=='OK' else '!'+v[0]+':'+v[1]) for c, v in d.items())
        print(f"{inst:11s} {sep:7s} base={b[0]}n/{b[1]}s  {cells}")
    # geometric means of node ratios
    for sep in ('plain', 'eager', 'nearsat'):
        for a, bcfg in (('PIPE', 'NOSCALE'), ('PIPE', 'DROP9'), ('PIPE', 'OLD')):
            r = []; rt = []
            for (inst, s), d in rows.items():
                if s != sep or a not in d or bcfg not in d: continue
                na, nb = float(d[a][2]) + 1, float(d[bcfg][2]) + 1
                ta, tb = float(d[a][3]) + 0.01, float(d[bcfg][3]) + 0.01
                r.append(na / nb); rt.append(ta / tb)
            if r:
                g = math.exp(sum(map(math.log, r)) / len(r)); gt = math.exp(sum(map(math.log, rt)) / len(rt))
                print(f"  gm {sep} {a}/{bcfg}: nodes {g:.3f} time {gt:.3f} over {len(r)} (min {min(r):.2f} max {max(r):.2f})")
