import sys,math,collections
rows=collections.OrderedDict()
for line in open(sys.argv[1]):
    if line.startswith('X|'): print('ABNORMAL',line.strip()); continue
    if not line.startswith('C|'): continue
    p=line.strip().split('|')
    rows.setdefault(p[1],{})[p[2]]=dict(secs=float(p[9]),status=p[10],value=float(p[11]) if p[11] not in('nan',) else float('nan'),closed=int(p[13]),it=int(p[14]),cuts=int(p[16]),skip=int(p[17]),tm=p[6],tn=p[7],m=int(p[3]),n=int(p[4]))
def fam(k):
    return k.split(':')[0] if ':' in k else 'miplib'
opt=lambda d:d['status'].startswith('optimal')
# value agreement
for k,r in rows.items():
    vals={c:d['value'] for c,d in r.items() if opt(d) and not math.isnan(d['value'])}
    if len(vals)>1:
        lo,hi=min(vals.values()),max(vals.values())
        if hi-lo>1.5e-4*(1+abs(hi)): print('VALUE DISAGREES',k,vals)
def gm(xs):
    return math.exp(sum(math.log(x) for x in xs)/len(xs)) if xs else float('nan')
for group in ['atsp','mkp','gap','ufl','lot','fcnd','miplib','ALL']:
    ks=[k for k in rows if group=='ALL' or fam(k)==group]
    for c,label in (('off','off / on'),('offcuts','off+cuts / on'),('npp','npp / on'),('off2','npp / off')):
        pairs=[]; closed_lost=[]; closed_won=[]
        base='on' if c!='off2' else 'off'
        cfg=c if c!='off2' else 'npp'
        for k in ks:
            r=rows[k]
            if base not in r or cfg not in r: continue
            a,b=r[base],r[cfg]
            if opt(a) and opt(b): pairs.append((max(b['closed'],1)/max(a['closed'],1),max(b['it'],1)/max(a['it'],1),max(b['secs'],.01)/max(a['secs'],.01),k))
            elif opt(a) and not opt(b): closed_lost.append(k)
            elif opt(b) and not opt(a): closed_won.append(k)
        worst=sorted(pairs,key=lambda t:-t[0])[:3]
        best=sorted(pairs,key=lambda t:t[0])[:2]
        print(f"{group:7s} {label:14s} both closed {len(pairs):2d}: nodes gm {gm([t[0] for t in pairs]):7.2f} it gm {gm([t[1] for t in pairs]):6.2f} secs gm {gm([t[2] for t in pairs]):6.2f} | worse-than-closed {closed_lost} | gained {closed_won} | worst {[(t[3],round(t[0],1)) for t in worst]} best {[(t[3],round(t[0],2)) for t in best]}")
print('instances',len(rows),'real',sum(1 for k in rows if fam(k)=='miplib'))
# what presolve removes on the real set
for k,r in rows.items():
    o=r['on']
    if fam(k)=='miplib' and (o['tm'],o['tn'])!=(str(o['m']),str(o['n'])): pass
tot=[k for k in rows if fam(k)=='miplib']
print('real with fewer columns after presolve', sum(1 for k in tot if int(rows[k]['on']['tn'])<rows[k]['on']['n']), 'of', len(tot))
for k in rows:
    d=rows[k].get('nppcuts'); e=rows[k].get('offcuts')
    if d and d['skip']: print('npp route lost cuts:',k,'landed',d['cuts'],'unusable',d['skip'],'| offcuts landed',e['cuts'])
