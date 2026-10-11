#include "dispatch_audit_io.hpp"
#include <chrono>
#include <fstream>
#include <iomanip>
using namespace egcs2017;
using namespace egcs2017::research;
int main(int argc,char** argv){try{
 if(argc!=2)throw std::runtime_error("usage: scale_check CSV");std::ofstream out(argv[1]);out<<std::setprecision(17);
 out<<"floors,cars,free,locked,speed,direction,supported_free_free,total_free_free,supported_free_fixed,total_free_fixed,matrix_error,greedy_changed,final_changed,return_changed,original_cost_difference,cached_us,combined_us\n";
 for(int F:{24,40})for(double speed:{1.6,5.0})for(int direction:{0,UP,DOWN}){
  Building b;b.floors=F;b.kin.v_max=speed;std::vector<CarView> cars(12);std::vector<CallView> calls;std::vector<HallKey> keys;
  for(int f=0;f<F;++f)for(int d:{DOWN,UP})if((f>0||d==UP)&&(f<F-1||d==DOWN))keys.push_back({f,d});
  const int n=F==24?12:24;std::vector<HallKey> free;std::vector<std::vector<HallKey>> locked(12);
  for(int c=0;c<12;++c){auto& car=cars[c];car.id=c;car.committed=1+(c*3)%(F-2);car.x=b.pos(car.committed);car.dir=direction;car.load=2;car.car_calls[std::max(0,car.committed-1)]=1;car.car_calls[std::min(F-1,car.committed+2)]=1;
   HallKey key=keys.at(c);calls.push_back(CallView{key,-100.0-c,c,true,1});locked[c].push_back(key);car.assigned.push_back(key);
   if(direction==0){car.car_calls.clear();car.load=0;}
  }
  for(int i=0;i<n;++i){HallKey key=keys.at(i+12);calls.push_back(CallView{key,double(i),-1,false,1});free.push_back(key);}
  long supported_ff=0,total_ff=0,supported_fl=0,total_fl=0;
  for(int c=0;c<12;++c){auto up=forward_features(b,cars[c],UP),down=forward_features(b,cars[c],DOWN);auto cross=cross_features(b,cars[c]);
   auto supported=[&](HallKey a,HallKey z){return (up&&up->contains(a)&&up->contains(z))||(down&&down->contains(a)&&down->contains(z))||(cross&&cross->contains(a,z));};
   for(int i=0;i<n;++i){for(int j=i+1;j<n;++j){++total_ff;supported_ff+=supported(free[i],free[j]);}for(auto l:locked[c]){++total_fl;supported_fl+=supported(free[i],l);}}
  }
  auto cached=structured_decide(b,cars,calls,no_new_call(),PairKernel::Cached),combined=structured_decide(b,cars,calls,no_new_call(),PairKernel::Combined);
  double error=audit::matrix_error(cached.matrices,combined.matrices);auto opt=locked_ls_options();double difference=g_of_public(combined.final,cached.matrices.u,cached.matrices.w,12,opt)-cached.final_cost;
  if(error>1e-7||std::abs(difference)>1e-7)throw std::runtime_error("large-setting coefficient/cost disagreement");
  double us[2]={0,0};for(int round=0;round<6;++round)for(int order=0;order<2;++order){int k=(round+order)%2;auto start=std::chrono::steady_clock::now();auto d=structured_decide(b,cars,calls,no_new_call(),k?PairKernel::Combined:PairKernel::Cached);
   us[k]+=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count();if(d.after.size()!=calls.size())throw std::runtime_error("lost assignment");}
  out<<F<<",12,"<<n<<",12,"<<speed<<','<<direction<<','<<supported_ff<<','<<total_ff<<','<<supported_fl<<','<<total_fl<<','<<error<<','<<(cached.greedy!=combined.greedy)<<','<<(cached.final!=combined.final)<<','<<(cached.after!=combined.after)<<','<<difference<<','<<us[0]/6<<','<<us[1]/6<<'\n';out.flush();
 }return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}}
