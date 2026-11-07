# python3 -I final_summarize.py <file>...: compact tables of the final_* experiment lines.
import re, sys, collections
def kv(s):
    return dict(re.findall(r'(\w+)=([^\s|()]+)', s))
for path in sys.argv[1:]:
    print('#####', path)
    sweep = collections.defaultdict(list)
    for line in open(path, errors='replace'):
        line = line.rstrip('\n')
        m = re.match(r'^(SWEEP|BIGM|TINY|HARM)(BASE)? (\S+) (.*)$', line)
        if not m: continue
        kind, base, be, rest = m.groups()
        d = kv(rest)
        prep = re.search(r'prep\(([^)]*)\)', rest)
        p = kv(prep.group(1)) if prep else {}
        if kind == 'SWEEP' and not base:
            fam = rest.split()[0]
            sweep[(re.sub(r's\d+$', '', fam), d['s'], d['cfg'].rstrip(':'))].append((d.get('vs_oracle'), d.get('nodes'), p.get('passed'), p.get('dropped'), d.get('calls')))
        elif kind in ('BIGM', 'TINY', 'HARM'):
            head = rest.split(':')[0]
            print(f"{kind}{base or ''} {head:60s} {d.get('vs_oracle','')+'/'+d.get('vs_base','') if not base else d.get('vs_oracle',''):12s} st={d.get('status','')[:22]:22s} v={d.get('value','')[:14]:14s} nodes={d.get('nodes',''):>7s} t={d.get('t',''):>7s} calls={d.get('calls',''):>6s} off={p.get('offered',''):>6s} pass={p.get('passed',''):>6s} drop={p.get('dropped',''):>6s} resc={p.get('rescaled','')}")
    for k, v in sorted(sweep.items()):
        verdicts = collections.Counter(x[0] for x in v)
        print(f"SWEEP {k[0]:8s} s={k[1]:6s} {k[2]:8s} {dict(verdicts)} nodes={[x[1] for x in v]} passed={[x[2] for x in v]} dropped={[x[3] for x in v]} calls={[x[4] for x in v]}")
