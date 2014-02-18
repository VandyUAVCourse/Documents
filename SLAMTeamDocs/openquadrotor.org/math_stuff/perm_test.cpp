#include "permutationsampler.hh"
#include <cstring>
#include <iostream>
#include <cstdlib>

using namespace std;



int main(){
  std::vector<double> v(1000);
  for (int i=0; i<(int)v.size(); i++)
    v[i]=i+1;
  
  PermutationSampler ps(v);
  
  sleep(1);
  for (int i=0; i<(int)v.size(); i++){
    double s=ps.getSum();
    double d=drand48();
    int i=ps.sampleWithRemoval(d*s);
    v[i]=0;
  }

  bool allEmpty=true;
  for (int i=0; i<(int)v.size(); i++){
    if (v[i]!=0.)
      allEmpty=false;
  }

  cerr << "Perm_test = " << ((allEmpty) ? "OK" : "KO") << endl;

}
