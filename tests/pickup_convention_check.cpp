#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <map>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr long kap=200, gam=175, op=160, cl=240, tau=96, d0=op+cl, h0=kap+d0, Q=h0+tau;
struct Fraction {
 long n=0,d=1;
 Fraction(long a=0,long b=1):n(a),d(b){if(d<=0)throw std::runtime_error("bad denominator");long g=std::gcd(n,d);n/=g;d/=g;}
 friend Fraction operator+(Fraction a,Fraction b){return {a.n*b.d+b.n*a.d,a.d*b.d};}
 friend Fraction operator-(Fraction a,Fraction b){return {a.n*b.d-b.n*a.d,a.d*b.d};}
 friend bool operator==(Fraction a,Fraction b){return a.n==b.n&&a.d==b.d;}
 double seconds()const{return double(n)/d/80;}
};
struct Call{int f,dir,id,dest;};
struct State{int F,s;std::map<int,int> k;bool open=false;long release=0;};
struct Pickup {std::array<long,2> a{-1,-1},b{-1,-1};};

Pickup itinerary(const State& initial,std::vector<Call> pending){
 auto k=initial.k;int pos=initial.s,dir=1;long clock=initial.release;Pickup result;
 auto board=[&](int direction,long a,long b){
  int count=0;
  for(size_t q=0;q<pending.size();){const auto c=pending[q];
   if(c.f==pos&&c.dir==direction){result.a[c.id]=a;result.b[c.id]=b;if(c.dest>=0)++k[c.dest];pending.erase(pending.begin()+q);++count;}
   else ++q;
  }return count;
 };
 if(initial.open){int boarded=board(dir,0,0);clock+=tau*boarded;}
 int guard=100;
 while(!pending.empty()){
  if(--guard==0)throw std::runtime_error("itinerary failed to terminate");
  int target=-1,best=100000;bool tail=false;
  for(const auto& [f,n]:k){(void)n;int dist=(f-pos)*dir;if(dist>=0&&dist<best){target=f;best=dist;}}
  for(const auto& c:pending){int dist=(c.f-pos)*dir;if(c.dir==dir&&dist>=0&&dist<best){target=c.f;best=dist;}}
  if(target<0){int far=-1;
   for(const auto& c:pending){int dist=(c.f-pos)*dir;if(c.dir==-dir&&dist>=0&&dist>far){target=c.f;far=dist;}}
   if(target<0){dir=-dir;continue;}tail=true;
  }
  if(target!=pos)clock+=kap+gam*std::abs(target-pos);
  pos=target;if(tail)dir=-dir;
  int alight=0;if(k.count(pos)){alight=k.at(pos);k.erase(pos);}
  const long a=clock,b=clock+op+tau*alight;int boarded=board(dir,a,b);
  bool ahead=false;for(const auto& [f,n]:k){(void)n;ahead|=(f-pos)*dir>0;}
  for(const auto& c:pending)ahead|=(c.f-pos)*dir>0;
  if(!ahead){bool opposite=false;for(const auto& c:pending)opposite|=c.f==pos&&c.dir==-dir;
   if(opposite){dir=-dir;boarded+=board(dir,a,b);}
  }
  clock+=d0+tau*(alight+boarded);
 }return result;
}
int count_at(const State& s,int f){auto p=s.k.find(f);return p==s.k.end()?0:p->second;}
long beta(const State& s,int i){if((i==s.s&&s.open)||count_at(s,i))return tau;return i==s.s?d0+tau:Q;}
void check(Fraction got,Fraction expected,const State& s,int i,int j,const std::string& label){
 if(got==expected)return;
 std::cerr<<label<<": F="<<s.F<<" s="<<s.s<<" i="<<i<<" j="<<j<<" open="<<s.open<<" release="<<s.release<<" K=";
 for(auto [f,n]:s.k)std::cerr<<f<<':'<<n<<',';
 std::cerr<<" got="<<got.seconds()<<" expected="<<expected.seconds()<<'\n';throw std::runtime_error("exact equality failed");
}
struct Pair {Fraction a,b;};
Pair pair_cost(const State& s,int i,int j,int second_direction){
 const auto ui=itinerary(s,{{i,1,0,-1}}),uj=itinerary(s,{{j,second_direction,1,-1}});
 long sum_a=0,sum_b=0;const int D=s.F-1-i;
 for(int dest=i+1;dest<s.F;++dest){const auto joint=itinerary(s,{{i,1,0,dest},{j,second_direction,1,-1}});
  check(joint.a[0],ui.a[0],s,i,j,"first A invariant");check(joint.b[0],ui.b[0],s,i,j,"first B invariant");
  sum_a+=joint.a[0]+joint.a[1];sum_b+=joint.b[0]+joint.b[1];
 }
 return {Fraction(sum_a,D)-ui.a[0]-uj.a[1],Fraction(sum_b,D)-ui.b[0]-uj.b[1]};
}
void same(const State& s,int i,int j){const auto w=pair_cost(s,i,j,1);long count=0;for(auto [f,n]:s.k){(void)n;count+=i<f&&f<j;}
 Fraction A=Fraction(beta(s,i))+Fraction(Q*(j-i-1)-h0*count,s.F-1-i);
 check(w.a,A,s,i,j,"same A generator");check(w.b,A+Fraction(tau,s.F-1-i),s,i,j,"same B generator");
}
void mixed(const State& s,int i,int j){const auto w=pair_cost(s,i,j,-1);const int D=s.F-1-i;
 int z=s.s,c=0,n=0;for(auto [f,q]:s.k){if(f>=s.s)z=std::max(z,f);if(f>i){++c;n+=q;}}
 const int zi=std::max(i,z);long lambda=0;for(auto [f,q]:s.k)if(j<f&&f<s.s)lambda+=h0+tau*q;
 const auto ui=itinerary(s,{{i,1,0,-1}}),uj=itinerary(s,{{j,-1,1,-1}});
 long b=ui.a[0]+d0+tau*(1+count_at(s,i));
 if(s.open&&i==s.s)b=s.release+tau+(count_at(s,i)?d0+tau*count_at(s,i):0);
 long sum_x=0,prob_m=0,prob_dm=0;
 for(int dest=i+1;dest<s.F;++dest){int M=std::max(zi,dest);const auto joint=itinerary(s,{{i,1,0,dest},{j,-1,1,-1}});
  long x=b+gam*(j-i)+h0*(c+1-(count_at(s,dest)>0))+tau*(n+1)+kap+lambda
         +2*gam*std::max(M-j,0)-(h0+tau*count_at(s,j))*(M==j)-tau*(dest==j&&M==j);
  check(joint.a[1],x,s,i,j,"mixed X A");
  long xb=x+op+tau*count_at(s,j)*((j<s.s)+(M==j))+tau*(dest==j&&M==j);
  check(joint.b[1],xb,s,i,j,"mixed X B");sum_x+=x;prob_m+=M==j;prob_dm+=dest==j&&M==j;
 }
 check(w.a,Fraction(sum_x,D)-uj.a[1],s,i,j,"mixed expectation");
 const Fraction delay=Fraction(tau*(count_at(s,j)*(prob_m-D*(j==z))+prob_dm),D);
 check(w.b,w.a+delay,s,i,j,"mixed B1");
 const int a=std::max(z,j);
 const long V0=gam*(s.F-1-a)*(s.F-a)-Q*(j>z)+((h0+tau*count_at(s,z))*(s.F-1-z)-tau)*(j==z);
 const long V1=gam*(s.F-1-j)*(s.F-j)-Q;
 const long V0b=gam*(s.F-1-a)*(s.F-a)-h0*(j>z)+h0*(s.F-1-z)*(j==z);
 const long V1b=gam*(s.F-1-j)*(s.F-j)-h0;
 const long E=b+Q+kap,Cj=gam*(s.F-j)+lambda-uj.a[1];
 const Fraction A=i<z?Fraction(beta(s,i)+Q)+Fraction(-h0*c+V0,D):j>i?Fraction(beta(s,i)+Q)+Fraction(V1,D):Fraction(E+Cj);
 const Fraction B=i<z?Fraction(beta(s,i)+Q)+Fraction(-h0*c+V0b,D):j>i?Fraction(beta(s,i)+Q)+Fraction(V1b,D):Fraction(E+Cj-tau*count_at(s,j)*(j==z));
 check(w.a,A,s,i,j,"mixed A generator");check(w.b,B,s,i,j,"mixed B generator");
}
}
int main(){try{
 long states=0,same_pairs=0,mixed_pairs=0;
 for(int F=3;F<=6;++F){int combinations=1;for(int f=0;f<F;++f)combinations*=3;
  for(int mask=0;mask<combinations;++mask){std::map<int,int> k;int m=mask;for(int f=0;f<F;++f){int digit=m%3;m/=3;if(digit)k[f]=digit==1?1:3;}
   for(int s=0;s<F-1;++s)for(bool opened:{false,true})for(long release:{0L,77L}){
    State state{F,s,k,opened,release};++states;
    for(int i=s;i<F-1;++i){for(int j=i+1;j<F-1;++j){same(state,i,j);++same_pairs;}
     for(int j=1;j<F;++j){mixed(state,i,j);++mixed_pairs;}
    }
   }
  }
 }
 std::cout<<"{\"exact_tick_denominator\":80,\"floors\":[3,4,5,6],\"states\":"<<states<<",\"same_pairs\":"<<same_pairs<<",\"mixed_pairs\":"<<mixed_pairs
 <<",\"known_count_alphabet\":[0,1,3],\"initial_direction\":\"up\",\"initial_motion\":\"stationary\",\"door_states\":[\"closed\",\"open\"],\"release_ticks\":[0,77],\"failures\":0,\"examples\":[";
 for(int q:{1,3}){State s{8,0,{{4,q}},false,0};auto w=pair_cost(s,2,4,-1);mixed(s,2,4);
  if(q==3)std::cout<<',';
  std::cout<<"{\"k4\":"<<q<<",\"A_seconds\":"<<w.a.seconds()<<",\"B_seconds\":"<<w.b.seconds()<<'}';}
 std::cout<<"]}\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
