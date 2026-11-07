import re,sys,collections
rows=[]
for l in open(sys.argv[1]):
    m=re.match(r'MEASURE v\S+ (\S+) shift=(\d) (.{11}): (\S+) value_same=(\S+) nodes=(\d+) lpiter=(\d+) t=([\d.]+) runs=(\d+) rows=(\d+) applied=(\d+) deep_rows=(\d+) conss_end=(\d+) pool_end=(\d+)',l)
    if m: rows.append(m.groups())
agg=collections.OrderedDict()
for r in rows:
    k=(r[0],r[2].strip())
    a=agg.setdefault(k,[0,0,0,0,0,0,True,0,True,0])
    a[0]+=int(r[5]);a[1]+=int(r[6]);a[2]+=float(r[7]);a[3]=max(a[3],int(r[8]));a[4]+=int(r[9]);a[5]+=int(r[10]);a[6]=a[6] and r[3]=='optimal';a[8]=a[8] and r[4]=='true';a[9]+=1
for k,a in agg.items():
    print(f"{k[0]:20} {k[1]:11} n={a[9]} nodes={a[0]:7} lpit={a[1]:8} t={a[2]:7.2f} maxruns={a[3]} rows={a[4]:6} applied={a[5]:6} allopt={a[6]} allsame={a[8]}")
