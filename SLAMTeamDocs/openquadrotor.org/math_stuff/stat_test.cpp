#include "stat.hh"
#include <cstring>
#include <iostream>

using namespace std;

int acc[100];

int main(){
  memset(acc, 0, sizeof(int)*100);
  
  for (int i=0; i<1000000; i++){
    double u=triangularSample(0.5, 0.5);
    int iu=(int)(u*100);
    acc[iu]++;
  }
  for (int i=0; i<100; i++){
    cout << i << " " << (double)acc[i]/10000 << endl;
  }
}
