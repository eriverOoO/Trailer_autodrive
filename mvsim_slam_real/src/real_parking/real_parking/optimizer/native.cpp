// Independent implementation of the original local Python equations. No external code copied.
#include <cmath>
#include <algorithm>
#include <vector>
#include <limits>
static double wrap(double x){double y=std::fmod(x+M_PI,2*M_PI);if(y<0)y+=2*M_PI;return y-M_PI;}
static void derivative(const double*s,double v,double d,const double*g,double*k){double w=v*std::tan(d)/g[0],b=s[2]-s[3];k[0]=v*std::cos(s[2]);k[1]=v*std::sin(s[2]);k[2]=w;k[3]=(v*std::sin(b)-g[1]*w*std::cos(b))/g[2];}
static void step(double*s,double v,double d,double dt,const double*g){double a[4],b[4],c[4],e[4],t[4];derivative(s,v,d,g,a);for(int i=0;i<4;i++)t[i]=s[i]+dt*a[i]/2;derivative(t,v,d,g,b);for(int i=0;i<4;i++)t[i]=s[i]+dt*b[i]/2;derivative(t,v,d,g,c);for(int i=0;i<4;i++)t[i]=s[i]+dt*c[i];derivative(t,v,d,g,e);for(int i=0;i<4;i++)s[i]+=dt*(a[i]+2*b[i]+2*c[i]+e[i])/6;s[2]=wrap(s[2]);s[3]=wrap(s[3]);}
extern "C" void propagate_native(const double*start,double distance,double delta,const double*g,int n,double*out){double s[4];std::copy(start,start+4,s);for(int i=0;i<n;i++){step(s,std::copysign(1.,distance),delta,std::abs(distance)/n,g);std::copy(s,s+4,out+4*i);}}
extern "C" void clearance_native(const double*states,int ns,const double*points,int np,int split,double hitch,const double*edt,int width,int height,const double*origin,double res,int smooth,double*out){
 double oc=std::cos(-origin[2]),os=std::sin(-origin[2]);
 for(int i=0;i<ns;i++){const double*s=states+4*i;double c0=std::cos(s[2]),s0=std::sin(s[2]),c1=std::cos(s[3]),s1=std::sin(s[3]);double minimum=std::numeric_limits<double>::infinity();bool ambiguous=false;
  for(int j=0;j<np;j++){bool trailer=j>=split;double c=trailer?c1:c0,ss=trailer?s1:s0;double x=c*points[2*j]-ss*points[2*j+1]+s[0]-(trailer?hitch*c0:0);double y=ss*points[2*j]+c*points[2*j+1]+s[1]-(trailer?hitch*s0:0);double dx=x-origin[0],dy=y-origin[1];double gx=(oc*dx-os*dy)/res,gy=(os*dx+oc*dy)/res,v=0;
   if(smooth){gx-=.5;gy-=.5;if(gx>=0&&gy>=0&&gx<=width-1&&gy<=height-1){int ix=(int)std::floor(gx),iy=(int)std::floor(gy);int nx=std::min(ix+1,width-1),ny=std::min(iy+1,height-1);double tx=gx-ix,ty=gy-iy;v=edt[iy*width+ix]*(1-tx)*(1-ty)+edt[iy*width+nx]*tx*(1-ty)+edt[ny*width+ix]*(1-tx)*ty+edt[ny*width+nx]*tx*ty;}}
   else{if(std::abs(gx-std::round(gx))<1e-9||std::abs(gy-std::round(gy))<1e-9)ambiguous=true;int ix=(int)std::floor(gx),iy=(int)std::floor(gy);if(ix>=0&&iy>=0&&ix<width&&iy<height)v=edt[iy*width+ix];}
   minimum=std::min(minimum,v);
  }out[i]=minimum;out[i+ns]=ambiguous?1.:0.;
 }
}
