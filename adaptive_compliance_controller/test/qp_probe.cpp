// Development-only stdin/stdout harness for comparison to an independent solver.
#include <adaptive_compliance_controller/adaptive_math.hpp>
#include <iostream>
#include <iomanip>
int main(){using namespace adaptive_compliance_controller::math;
  V3 h,g,l,u,x;std::array<V3,2>a;Eigen::Vector2d b;
  std::cout<<std::setprecision(17);
  while(std::cin>>h[0]){
    std::cin>>h[1]>>h[2];for(auto *v:{&g,&l,&u,&a[0],&a[1]})for(int i=0;i<3;++i)std::cin>>(*v)[i];
    std::cin>>b[0]>>b[1];
    const bool ok=solveQP(h,g,l,u,a,b,x);
    std::cout<<(ok?1:0)<<' '<<(ok?x[0]:0.)<<' '<<(ok?x[1]:0.)<<' '<<(ok?x[2]:0.)<<'\n';
  }
}
