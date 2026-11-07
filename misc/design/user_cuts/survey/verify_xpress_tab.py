import re,sys
# case table: one row per conformance case with its hook counters
for fn in sys.argv[1:]:
    print('##',fn)
    lines=open(fn).read().split('\n')
    i=0
    rows=[]
    for k,l in enumerate(lines):
        m=re.match(r'(PASS|FAIL) ([^:]+): (.*)',l)
        if not m: continue
        res,name,rest=m.groups()
        cc=re.search(r'calls=(\d+) cuts=(\d+) add_errors=(\d+)',rest)
        st=re.search(r'\| status=(\S+) solve=(\S+) value=(\S+) dual=(\S+) nodes=(\d+)',rest)
        bs=re.search(r'baseline value (\S+) nodes (\S+)',rest)
        x=''
        if k+1<len(lines) and lines[k+1].startswith('   xp'):
            q=lines[k+1]
            mm=re.search(r'nodes=(\d+) max_depth=(\d+) max_rounds=(\d+) added_native=(\d+) dropped_by_sanitize=(\d+).*COLS=\[(\d+),(\d+)\] input=(\d+) restarts=(\d+) presolve_empty=(\d+) .*?status\{(.*?)\} errors\{(.*?)\}',q)
            if mm: x='hnodes=%s depth=%s rounds=%s native=%s drop=%s cols=[%s,%s]/%s rst=%s pe=%s pst{%s} err{%s}'%mm.groups()
        print(res,name,'| base',bs.group(1)[:8] if bs else '?',bs.group(2) if bs else '?','|',(st.group(1)[:14],st.group(2),st.group(3)[:8],st.group(5)) if st else rest[:80],'| calls',cc.group(1) if cc else '','cuts',cc.group(2) if cc else '','err',cc.group(3) if cc else '','|',x)
