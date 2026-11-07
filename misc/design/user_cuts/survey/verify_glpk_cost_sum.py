import sys,collections
rows=collections.OrderedDict()
for line in open(sys.argv[1]):
    if line.startswith('X|'):
        p=line.strip().split('|'); rows.setdefault(p[1],{})[p[2]]=dict(status=p[3]); continue
    if not line.startswith('C|'): continue
    p=line.strip().split('|')
    d=dict(name=p[1],cfg=p[2],m=int(p[3]),n=int(p[4]),ni=int(p[5]),tm=p[6],tn=p[7],ti=p[8],secs=float(p[9]),status=p[10],value=p[11],bound=p[12],closed=int(p[13]),it=int(p[14]),fired=int(p[15]),cuts=int(p[16]),skip=int(p[17]))
    rows.setdefault(d['name'],{})[d['cfg']]=d
def short(s): return s.split('{')[0][:9]
print(f"{'instance':16s} {'m x n':>11s} {'pre m x n':>11s} | on: stat nodes it secs val | off: ... | offcuts: ... (cuts) | npp ... | nppcuts (cuts skip)")
for k,r in rows.items():
    o=r.get('on')
    if not o or 'closed' not in o: print(k,r); continue
    def f(c):
        d=r.get(c)
        if not d: return '-'
        if 'closed' not in d: return d['status']
        v=d['value']
        try: v=f"{float(v):.6g}"
        except: pass
        return f"{short(d['status'])} {d['closed']} {d['it']} {d['secs']:.2f} {v}"+(f" c{d['cuts']}" if d['cuts'] else '')+(f" skip{d['skip']}" if d['skip'] else '')
    print(f"{k:18s} {o['m']}x{o['n']}->{o['tm']}x{o['tn']} | {f('on')} | {f('off')} | {f('offcuts')} | {f('npp')} | {f('nppcuts')}")
