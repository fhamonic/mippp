#include "verify_scip_native.hpp"
using namespace vs;
int main() {
  for(const char* sel : {"dfs","restartdfs"}) for(bool local:{true,false}) {
    G()=recipe{}; V m; auto & n=m.N(); void* s=m.sp();
    n.setPresolving(s,3,1); n.setHeuristics(s,3,1); n.setSeparating(s,3,1); m.seti("presolving/maxrestarts",0); m.seti("misc/usesymmetry",0); n.setBoolParam(s,"conflict/enable",0);
    m.seti(std::format("nodeselection/{}/stdpriority",sel).c_str(),1000000);
    auto g=cp::build_gadgets(m,10,false);
    long st[3]={0,0,0}; long after=0; bool added=false; int addd=-1; double xval=-1; long long addnode=-1; std::vector<std::string> log;
    raw_cut: ;
    const cp::raw_cut cut=g.invalid_global_cut();
    m.attach_custom([&](plugin& p,const std::vector<double>& x,int d){
        st[g.state(x)+1]++;
        if(!added){ if(g.state(x)==g.worse_state() && d>=1 && cut.violation(x)>1e-4){ recipe rr; rr.local=local; rr.add_pool=false; p.add(cut,x,rr); added=true; addd=d; xval=x[static_cast<std::size_t>(g.y[0])]; addnode=n.nodeGetNumber(n.getCurrentNode(s)); } return; }
        if(++after<=8) log.push_back(std::format("n{}d{}y0={:.3g}viol={:.3g}",n.nodeGetNumber(n.getCurrentNode(s)),d,x[static_cast<std::size_t>(g.y[0])],cut.violation(x)));
    });
    m.set_time_limit(std::chrono::duration<double>(30)); m.solve();
    std::string dd; for(auto [k,c]: m.P->calls_by_depth) dd+=std::format(" {}:{}",k,c);
    std::string ll; for(auto&e:log) ll+=" "+e;
    std::println(stderr,"{} local={} value={} nodes={} entries={} add_depth={} add_node={} y0_at_add={} after={} y0 frac:{} 0:{} 1:{} depth{} |{}",sel,local,cf::value_of(m),n.getNTotalNodes(s),m.P->entries,addd,addnode,xval,after,st[0],st[1],st[2],dd,ll);
  }
}
