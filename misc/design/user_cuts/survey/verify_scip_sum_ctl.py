import re,sys,glob,os
base=sys.argv[1:] or sorted(glob.glob('vout_scip_ctl_*.txt'))
for f in base:
    t=open(f,errors='replace').read().splitlines()
    pas=[l for l in t if l.startswith('PASS')]; fail=[l for l in t if l.startswith('FAIL')]
    real=[l for l in fail if 'NO BASELINE' not in l]
    calls=rows=applied=errs=nodes=0
    for l in t:
        m=re.search(r'\[V\] runs=(\d+) conss_end=\d+ nodes=(\d+) lpiter=\d+ entries=(\d+) not_optimal=(\d+) deep_calls=\d+ rows=(\d+) deep_rows=\d+ one_term=\d+ pool_adds=\d+ sanitized=\d+ infeasible_flags=\d+ add_errors=(\d+) exceptions=\d+ sepa\(calls=\S+ found=\S+ applied=(-?\d+)',l)
        if m and int(m.group(3))>0:
            calls+=int(m.group(3)); rows+=int(m.group(5)); errs+=int(m.group(6)); applied+=int(m.group(7)); nodes+=int(m.group(2))
    err=[l for l in t if 'ERROR' in l]
    nocore='dumped core' in ' '.join(t[-3:])
    print(f"{os.path.basename(f):38} pass={len(pas)} fail={len(fail)} real_fail={len(real)} hooked: calls={calls} rows={rows} applied={applied} add_errors={errs} nodes={nodes} ERRORlines={len(err)} {'CRASH' if nocore else ''} last={t[-1][:30] if t else ''}")
    for l in real[:4]: print("     ",l[:230])
