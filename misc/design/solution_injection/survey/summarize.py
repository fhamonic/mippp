import sys, re, collections
rows = []
for f in sys.argv[1:]:
    for line in open(f):
        if ' mode=' not in line: continue
        d = dict(kv.split('=', 1) for kv in line.split()[1:] if '=' in kv)
        d['backend'] = line.split()[0]
        rows.append(d)
cols = ['seed','mode','status','obj','time','rej','acc','lost_improving','refound','refound_next','inj','marked_calls','marked_rej','lies','unchecked_inc','ti_1pct','ti_opt','bound']
by = collections.defaultdict(list)
for r in rows: by[r['backend']].append(r)
for b, rs in by.items():
    print('==', b)
    print(' '.join(f'{c:>13}' for c in cols))
    for r in rs:
        print(' '.join(f"{r.get(c,''):>13}" for c in cols))
