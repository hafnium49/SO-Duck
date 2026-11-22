#include <vector>
#include <array>
#include <cmath>
#include <cstring>
#include <algorithm>
using V=std::array<double,3>;
using Tri=std::array<V,3>;
class vtkTriangle {public: static int TrianglesIntersect(const double*,const double*,const double*,const double*,const double*,const double*);};
struct Node { V mid,half;int a,b,l=-1,r=-1; };
struct Mesh {std::vector<Tri> t;std::vector<Node> n;
int build(int a,int b){V lo={1e30,1e30,1e30},hi={-1e30,-1e30,-1e30};for(int i=a;i<b;i++)for(auto &p:t[i]) for(int j=0;j<3;j++){lo[j]=std::min(lo[j],p[j]);hi[j]=std::max(hi[j],p[j]);}
 Node r;r.a=a;r.b=b; int k=0;for(int j=0;j<3;j++){r.mid[j]=(hi[j]+lo[j])*.5;r.half[j]=(hi[j]-lo[j])*.5;if(r.half[j]>r.half[k])k=j;}int id=n.size();n.push_back(r);if(b-a<=6)return id;
 int s=(a+b)/2;std::nth_element(t.begin()+a,t.begin()+s,t.begin()+b,[k](const Tri &u,const Tri &v){return u[0][k]+u[1][k]+u[2][k]<v[0][k]+v[1][k]+v[2][k];});int l=build(a,s),h=build(s,b);n[id].l=l;n[id].r=h;return id;}};
struct Query{Mesh *A,*B;const double* m;std::vector<V> mid,half,tp;std::vector<char> done;V transform(const V&p){V r;for(int i=0;i<3;i++)r[i]=m[4*i]*p[0]+m[4*i+1]*p[1]+m[4*i+2]*p[2]+m[4*i+3];return r;}
 bool test(int ia,int ib){auto &a=A->n[ia];auto &b=B->n[ib];if(!done[ia]){done[ia]=1;mid[ia]=transform(a.mid);for(int i=0;i<3;i++)half[ia][i]=std::abs(m[i*4])*a.half[0]+std::abs(m[i*4+1])*a.half[1]+std::abs(m[i*4+2])*a.half[2];}for(int i=0;i<3;i++)if(std::abs(mid[ia][i]-b.mid[i])>half[ia][i]+b.half[i]+1e-8)return false;
 if(a.l==-1&&b.l==-1){for(int i=a.a;i<a.b;i++){auto x=transform(A->t[i][0]),y=transform(A->t[i][1]),z=transform(A->t[i][2]);for(int j=b.a;j<b.b;j++){auto &p=B->t[j];if(vtkTriangle::TrianglesIntersect(x.data(),y.data(),z.data(),p[0].data(),p[1].data(),p[2].data()))return true;}}return false;}
 if(a.l!=-1&&(b.l==-1||a.b-a.a>b.b-b.a))return test(a.l,ib)||test(a.r,ib);return test(ia,b.l)||test(ia,b.r);
 }};
extern "C" Mesh* load(const double *data,int count){auto *m=new Mesh;m->t.resize(count);memcpy(m->t.data(),data,count*sizeof(Tri));m->build(0,count);return m;}
extern "C" int collisions(Mesh*A,Mesh*B,const double*ms,unsigned char*out,int count){Query q{A,B};q.mid.resize(A->n.size());q.half.resize(A->n.size());q.done.resize(A->n.size());for(int i=0;i<count;i++){q.m=ms+i*16;memset(q.done.data(),0,q.done.size());out[i]=q.test(0,0);}return 0;}
