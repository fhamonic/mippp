#!/usr/bin/env python3
# Applies one code change to a copy of include/, rebuilds a backend's tests, and reports which fail.
import os, shutil, subprocess, sys
SP='/tmp/claude-1000/-home-plaiseek-Projects-mippp/b99a8387-c5e9-4bc6-9141-d4502a992e5d/scratchpad/testbuild'
R='/home/plaiseek/Projects/mippp'
G=open(SP+'/gtest_path.txt').read().strip(); M=open(SP+'/melon_path.txt').read().strip()
def run(name, backend, edits, flt='*Progress*:*CandidateSolution*', envs=[{}]):
    inc=SP+'/mut_inc'
    shutil.rmtree(inc, ignore_errors=True); shutil.copytree(R+'/include', inc)
    for path,a,b in edits:
        p=inc+'/mippp/solvers/'+path
        s=open(p).read()
        if a not in s:
            print(f'== {name}: TARGET MISSING: {a[:60]}'); return
        open(p,'w').write(s.replace(a,b,1))
    exe=f'{SP}/mut_{backend}'
    c=subprocess.run(['/opt/gcc-15/bin/g++','-std=c++26','-O0','-I',inc,'-I',R+'/test','-I',G+'/include','-I',M,f'solvers/{backend}.cpp','main.cpp','-o',exe,G+'/lib/libgtest.a','-ldl','-lpthread'],cwd=R+'/test',capture_output=True,text=True)
    if c.returncode:
        print(f'== {name}: DOES NOT COMPILE'); print('\n'.join(l for l in c.stderr.splitlines() if ' error ' in l)[:600]); return
    for env in envs:
        e=dict(os.environ); e['MIPPP_NO_VERSION_WARNING']='1'; e.update(env)
        try:
            r=subprocess.run(['timeout','-k','5','150',exe,f'--gtest_filter={flt}'],capture_output=True,text=True,env=e,cwd=SP)
        except Exception as x:
            print(name,x); continue
        out=r.stdout+r.stderr
        failed=sorted({l.split('/0.')[-1].split(',')[0].split(' ')[0] for l in out.splitlines() if l.startswith('[  FAILED  ]') and '/0.' in l})
        passed=[l for l in out.splitlines() if l.startswith('[  PASSED  ]')]
        tag=' '.join(f'{k.split("_")[1]}={os.path.basename(os.path.dirname(os.path.dirname(v)))[:24]}' for k,v in env.items())
        print(f'== {name} {tag} exit={r.returncode}: caught by [{", ".join(failed)}] {passed[0] if passed else "no PASSED line"}')
    os.remove(exe)
ALL=sys.argv[1:]
rel=lambda f: open(SP+'/../progress/'+f).read().strip()
muts={
'M2':lambda: run('M2 HiGHS stop never reset','highs',[('highs/impl/v1/highs_milp.hpp','data_in->user_interrupt = handle.interrupt_requested ? 1 : 0;','if(handle.interrupt_requested) data_in->user_interrupt = 1;')],'*Progress*',[{'MIPPP_HIGHS_LIBRARY':rel('lib_highs_1.7.2.txt')},{'MIPPP_HIGHS_LIBRARY':rel('lib_highs_1.14.0.txt')},{}]),
'M3':lambda: run('M3 Gurobi registered by the setters','gurobi',[('gurobi/impl/v1/gurobi_milp.hpp','const bool attach = candidate_solution_callback || progress_callback;','const bool attach = false;'),('gurobi/impl/v1/gurobi_milp.hpp','candidate_solution_callback = std::forward<F>(f);\n','candidate_solution_callback = std::forward<F>(f); check(GRB->setcallbackfunc(model, main_callback, this));\n'),('gurobi/impl/v1/gurobi_milp.hpp','        progress_callback = std::forward<F>(f);\n','        progress_callback = std::forward<F>(f); check(GRB->setcallbackfunc(model, main_callback, this));\n')]),
'M4':lambda: run('M4 Xpress offset dropped','xpress',[('xpress/impl/v1/xpress_milp.hpp','return objective_offset + detail::as_infinity(value, XPRS_PLUSINFINITY);','return detail::as_infinity(value, XPRS_PLUSINFINITY);')]),
'M5':lambda: run('M5 GLPK offset dropped','glpk',[('glpk/impl/v1/glpk_milp.hpp','return objective_offset + glp->mip_obj_val(presolved);','return glp->mip_obj_val(presolved);')]),
'M6':lambda: run('M6 MOSEK validity flags ignored','mosek',[('mosek/impl/v1/mosek_milp.hpp','if(intinf[codes.num_incumbents] == 0)\n                return _infinity_if(codes.minimizing);\n',''),('mosek/impl/v1/mosek_milp.hpp','if(intinf[codes.bound_defined] == 0)\n                return _infinity_if(!codes.minimizing);\n','')]),
'M7':lambda: run('M7 CPLEX attached on every path','cplex',[('cplex/impl/v1/cplex_milp.hpp','int probtype = CPX->getprobtype(env, lp);\n        switch(probtype) {','int probtype = CPX->getprobtype(env, lp);\n        const auto detach0 = _attached_callbacks();\n        switch(probtype) {'),('cplex/impl/v1/cplex_milp.hpp','const auto detach = _attached_callbacks();\n                check(CPX->mipopt','check(CPX->mipopt')]),
'M7b':lambda: run('M7b CPLEX reads a thread\'s view','cplex',[('cplex/impl/v1/cplex_milp.hpp','(progress_callback ? CPX_CALLBACKCONTEXT_GLOBAL_PROGRESS : 0)','(progress_callback ? CPX_CALLBACKCONTEXT_LOCAL_PROGRESS : 0)')]),
'M8':lambda: run('M8 COPT progress on MIPSOL','copt',[('copt/impl/v1/copt_milp.hpp','? COPT_CBCONTEXT_MIPNODE | COPT_CBCONTEXT_INCUMBENT','? COPT_CBCONTEXT_MIPSOL | COPT_CBCONTEXT_INCUMBENT')]),
'M9':lambda: run('M9 SCIP subscription survives a free','scip',[('scip/impl/v1/scip_milp.hpp','if(_progress_subscribed) check(_drop_progress_callback());\n','')]),
'M10':lambda: run('M10 Xpress registered once more per setter','xpress',[('xpress/impl/v1/xpress_milp.hpp','        candidate_solution_callback = std::forward<F>(f);\n','        candidate_solution_callback = std::forward<F>(f); check(XPRS->addcbpreintsol(prob, candidate_solution_callback_fun, this, 1));\n')]),
'M11':lambda: run('M11 SCIP handler left out of a resumed solve','scip',[('scip/impl/v1/scip_milp.hpp','if(!_progress_plugin || _progress_subscribed) return;','if(!_progress_plugin || _progress_subscribed || _results_valid_at_entry) return;')]),
'M12':lambda: run('M12 GLPK cb_info set once','glpk',[('glpk/impl/v1/glpk_milp.hpp','model_params.cb_info = this;','static void * first = this; model_params.cb_info = first;')]),
}
for k in ALL: muts[k]()
