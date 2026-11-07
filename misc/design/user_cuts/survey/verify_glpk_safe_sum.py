import re,collections,sys
agg=collections.OrderedDict()
for line in open(sys.argv[1]):
    if line.startswith('FINISHED'): continue
    m=re.match(r'\[(\S+) p=(\S+) nz=(\S+) tol=(\S+) s=(\d+)\] (.*)',line)
    if not m: continue
    key=m.group(1)+' p='+m.group(2)+(' nz' if m.group(3)=='1' else '')+('' if m.group(4)=='1e-9' else ' tol='+m.group(4))
    a=agg.setdefault(key,dict(n=0,kept=0,lost=0,wrong=0,abort=0,fired=0,passed=0,maxrows=0,nodes=0,msgs=[],t=0.0))
    a['n']+=1
    body=m.group(6)
    if '=> KEPT' in body: a['kept']+=1
    elif '=> LOST' in body: a['lost']+=1
    elif '=> WRONG' in body: a['wrong']+=1
    else: a['abort']+=1
    r=re.search(r'fired=(\d+) offered=(\d+) passed=(\d+) maxrows=(\d+) nodes=(\d+) it=(\d+) ([\d.]+)s',body)
    if r:
        a['fired']+=int(r.group(1));a['passed']+=int(r.group(3));a['maxrows']=max(a['maxrows'],int(r.group(4)));a['nodes']+=int(r.group(5));a['t']+=float(r.group(7))
    if 'KEPT' not in body: a['msgs'].append(body[:190])
show=int(sys.argv[2]) if len(sys.argv)>2 else 1
for k,a in agg.items():
    print(f"{k:26s} n={a['n']:3d} kept={a['kept']:3d} lost={a['lost']:2d} wrong={a['wrong']:2d} abort={a['abort']:2d} fired={a['fired']:5d} passed={a['passed']:5d} maxrows={a['maxrows']:4d} nodes={a['nodes']:5d} t={a['t']:.1f}")
    for msg in a['msgs'][:show]: print('    ',msg)
