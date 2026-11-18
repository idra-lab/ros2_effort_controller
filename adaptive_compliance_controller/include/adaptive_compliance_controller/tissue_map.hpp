#pragma once
#include <adaptive_compliance_controller/adaptive_math.hpp>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace adaptive_compliance_controller {
struct Tissue {double surface=0.,kappa=10000.,viscosity=100.;};
class TissueMap {
 public:
  void flat(const Tissue &value) {validate(value);flat_=value;x_.clear();y_.clear();}
  void load(const std::string &directory) {
    x_=read(directory+"/x.txt");y_=read(directory+"/y.txt");
    validateAxis(x_);validateAxis(y_);
    const size_t count=x_.size()*y_.size();
    z_=read(directory+"/z.txt");k_=read(directory+"/stiffness.txt");d_=read(directory+"/damping.txt");
    if(z_.size()!=count || k_.size()!=count || d_.size()!=count)
      throw std::runtime_error("Map grids must have exactly len(x)*len(y) entries, x-major/y-minor");
    for(size_t i=0;i<count;++i)validate({z_[i],k_[i],d_[i]});
  }
  bool sample(double x,double y,Tissue &out) const {
    if(!std::isfinite(x)||!std::isfinite(y))return false;
    if(x_.empty()) {out=flat_;return true;}
    if(x<x_.front()||x>x_.back()||y<y_.front()||y>y_.back())return false;
    const size_t i=cell(x_,x),j=cell(y_,y);
    const double u=(x-x_[i])/(x_[i+1]-x_[i]),v=(y-y_[j])/(y_[j+1]-y_[j]);
    out={interpolate(z_,i,j,u,v),interpolate(k_,i,j,u,v),interpolate(d_,i,j,u,v)};
    return true;
  }
 private:
  Tissue flat_;
  std::vector<double>x_,y_,z_,k_,d_;
  static void validate(const Tissue &t) {
    if(!std::isfinite(t.surface)||!std::isfinite(t.kappa)||t.kappa<=0.||
       !std::isfinite(t.viscosity)||t.viscosity<0.)throw std::runtime_error("Invalid tissue surface/HC parameters");
  }
  static std::vector<double> read(const std::string &path) {
    std::ifstream file(path);
    if(!file)throw std::runtime_error("Cannot read tissue map: "+path);
    std::vector<double> values;double value;
    while(file>>value) {if(!std::isfinite(value))throw std::runtime_error("Nonfinite map entry");values.push_back(value);}
    if(!file.eof()||values.empty())throw std::runtime_error("Empty or malformed tissue map: "+path);
    return values;
  }
  static void validateAxis(const std::vector<double> &axis) {
    if(axis.size()<2)throw std::runtime_error("Tissue map axes need at least two coordinates");
    for(size_t i=1;i<axis.size();++i)if(axis[i]<=axis[i-1])throw std::runtime_error("Map axes must strictly increase");
  }
  static size_t cell(const std::vector<double> &a,double value) {
    return std::min(a.size()-2,static_cast<size_t>(std::upper_bound(a.begin(),a.end(),value)-a.begin()-1));
  }
  double interpolate(const std::vector<double> &grid,size_t i,size_t j,double u,double v) const {
    const size_t n=y_.size();
    return (1-u)*((1-v)*grid[i*n+j]+v*grid[i*n+j+1])+
                u*((1-v)*grid[(i+1)*n+j]+v*grid[(i+1)*n+j+1]);
  }
};
}  // namespace adaptive_compliance_controller
