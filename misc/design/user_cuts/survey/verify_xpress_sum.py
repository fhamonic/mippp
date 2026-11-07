import re,sys
# one line per case of the safe2 sweeps
for fn in sys.argv[1:]:
    print('##',fn)
    for l in open(fn):
        if l.startswith(('CRASH','CHILD')):
            print(l.strip()[:170]); continue
        if not re.match(r'(OK|BAD)',l): continue
        m=re.match(r'(\S+)\s+(.*?) \[(.*?)\]: base (\S+) \| status=(\S+) solve=(\S+) value=(\S+) dual=\S+ nodes=(\d+) \| calls=(\d+) native=(\d+) dropped=(\d+) rows (\S+) CUTS_max=(\d+) nodes_seen=(\d+) depth=(\d+) presolve\{(.*?)\} empty=(\d+) errors\{(.*?)\}',l)
        if not m: print('??',l[:150]); continue
        g=m.groups()
        print(g[0].ljust(3),g[2].ljust(17),g[1][:52].ljust(52),g[4][:10].ljust(10),g[5].rjust(6),'v',g[6][:9].ljust(9),'base',g[3][:7].ljust(7),'n',g[7].rjust(5),'c',g[8].rjust(7),'nat',g[9].rjust(7),'dr',g[10].rjust(5),'rows',g[11],'pst{'+g[15]+'}','err{'+g[17]+'}')
