#include "../research/forward_dispatch.hpp"
#include <array>
#include <iomanip>
#include <iostream>

using namespace egcs2017;
using namespace egcs2017::research;

int main() {
  try {
    long states=0,locked=0,removed=0,changed=0;
    std::array<long,4> different{},better{},worse{},tied{};
    std::array<double,4> sum_difference{},max_abs_difference{};
    long supported_removal_changes=0,other_removal_changes=0;
    long complete_states=0;std::array<double,4> complete_sum_difference{};
    double sum_full=0,sum_forward=0,partition_error=0;
    int F,C,n;
    const auto opt=locked_ls_options();
    while(std::cin>>F>>C>>n) {
      Building b;b.floors=F;
      std::vector<CarView> cars(C);
      for(auto& c:cars) {
        int count;
        std::cin>>c.id>>c.x>>c.v>>c.dir>>c.moving>>c.arriving>>c.committed
          >>c.release>>c.doors_open>>c.load>>count;
        for(int j=0;j<count;++j) {int f,q;std::cin>>f>>q;c.car_calls[f]=q;}
      }
      std::vector<CallView> calls(n);
      for(auto& k:calls)std::cin>>k.key.floor>>k.key.dir;
      for(int c=0;c<C;++c) {
        int count;std::cin>>count;
        for(int j=0;j<count;++j) {
          CallView k;std::cin>>k.key.floor>>k.key.dir;
          k.assigned=cars[c].id;k.close_to_service=true;k.registered=-1;calls.push_back(k);
        }
      }

      double unused;
      for(int i=0;i<2*n*C+n*n*C;++i)std::cin>>unused;
      HallKey nc;std::cin>>nc.floor>>nc.dir;
      for(int i=0;i<n;++i)std::cin>>calls[i].registered>>calls[i].assigned>>calls[i].waiting;
      if(!std::cin)throw std::runtime_error("truncated input");
      const auto M=build_matrices(b,cars,calls,nc,opt);
      if(int(M.N.size())!=n)throw std::runtime_error("call reconstruction mismatch");
      bool has_locked=false;
      for(auto& row:M.locked)has_locked|=!row.empty();
      locked+=has_locked;
      auto forward=M.u_lk;
      for(auto& row:forward)std::fill(row.begin(),row.end(),0.0);
      for(int c=0;c<C;++c) {
        auto up=forward_features(b,cars[c],UP),down=forward_features(b,cars[c],DOWN);
        for(int i=0;i<n;++i)for(auto l:M.locked[c]) {
          auto& f=M.N[i].key.dir==UP?up:down;
          if(f && f->contains(M.N[i].key) && f->contains(l)) {

            CostOptions raw;
            forward[i][c]+=std::max(0.0,pairwise(b,cars[c],M.N[i].key,l,
                                   M.u_raw[i][c],unary(b,cars[c],l,raw)));
          }
        }
      }
      std::array<std::vector<std::vector<double>>,4> u{M.u,M.u,M.u,M.u};
      for(int i=0;i<n;++i)for(int c=0;c<C;++c) {
        const double r=M.u_lk[i][c],f=forward[i][c];
        if(f>r+1e-9)throw std::runtime_error("forward part exceeds total correction");
        const double other=r-f;
        u[0][i][c]-=r;
        u[1][i][c]-=other;
        u[2][i][c]-=f;

        partition_error=std::max(partition_error,std::abs((f+other)-r));
        sum_full+=r;sum_forward+=f;
      }
      std::array<std::vector<int>,4> x;
      for(int k=0;k<4;++k) {
        x[k]=greedy_public(u[k],M.w,C,opt);
        local_search_public(x[k],u[k],M.w,C,opt);
      }
      const double full=g_of_public(x[3],M.u,M.w,C,opt);
      bool has_removal=false;
      for(int i=0;i<n;++i) {
        const int c=x[3][i];
        if(c>=0 && cars[c].load>=std::max(1,b.capacity-opt.capacity_margin))has_removal=true;
      }
      removed+=has_removal;complete_states+=!has_removal;
      if(x[0]!=x[3]) {
        ++changed;
        supported_removal_changes+=x[2]!=x[3];
        other_removal_changes+=x[1]!=x[3];
      }
      for(int k=0;k<4;++k) {
        const double diff=g_of_public(x[k],M.u,M.w,C,opt)-full;
        different[k]+=x[k]!=x[3];sum_difference[k]+=diff;
        max_abs_difference[k]=std::max(max_abs_difference[k],std::abs(diff));

        better[k]+=diff < -1e-9;worse[k]+=diff > 1e-9;tied[k]+=std::abs(diff)<=1e-9;
        if(!has_removal)complete_sum_difference[k]+=diff;
      }
      ++states;
    }
    std::cout<<std::setprecision(17)<<"{\"states\":"<<states<<",\"locked_states\":"<<locked
      <<",\"states_with_full_postprocessing_removal\":"<<removed
      <<",\"complete_full_action_states\":"<<complete_states
      <<",\"full_vs_no_correction_changes\":"<<changed
      <<",\"among_changed_forward_removal_changes\":"<<supported_removal_changes
      <<",\"among_changed_other_removal_changes\":"<<other_removal_changes
      <<",\"sum_all_locked_coefficients\":"<<sum_full
      <<",\"sum_forward_locked_coefficients\":"<<sum_forward
      <<",\"maximum_partition_error\":"<<partition_error<<",\"variants\":[";
    const char* names[]={"none","forward_only","other_only","full"};
    for(int k=0;k<4;++k) {
      if(k)std::cout<<',';
      std::cout<<"{\"name\":\""<<names[k]<<"\",\"labels_differ\":"<<different[k]
        <<",\"full_cost_better\":"<<better[k]<<",\"full_cost_worse\":"<<worse[k]
        <<",\"full_cost_tied\":"<<tied[k]<<",\"full_cost_difference_sum\":"<<sum_difference[k]
        <<",\"maximum_absolute_cost_difference\":"<<max_abs_difference[k]
        <<",\"complete_full_action_difference_sum\":"<<complete_sum_difference[k]<<'}';
    }
    std::cout<<"]}\n";
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}
}
