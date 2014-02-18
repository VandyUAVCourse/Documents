#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <list>
#include <vector>
#include <cstdlib>
#include <algorithm>

#define LINESIZE 64000
using namespace std;

const char *message[]={
  "clfmerge: sorts and merges a set of logfiles based on the timestamps, and writes the output on standard output",
  "usage clfsort <file 1> <file 2> ... <file N>",
  0
};

struct LogLine{
  string line;
  std::vector<string> tokens;
  double ts, lts;
  string hostname;
};

istream& operator >> (istream& is, LogLine& logline){
  logline.tokens.clear();
  logline.ts=0.;
  char buf[LINESIZE];
  is.getline(buf, LINESIZE);
  logline.line=buf;
  istringstream ls(buf);
  while (ls){
    string token;
    ls >> token;
    logline.tokens.push_back(token);
  }
  if (!logline.tokens.size())
    return is;
  if (logline.tokens.at(0)[0]=='#')
    return is;
  if (logline.tokens.size()<4)
    return is;
  logline.ts=atof(logline.tokens.at(logline.tokens.size()-4).c_str());
  logline.hostname=logline.tokens.at(logline.tokens.size()-3);
  logline.lts=atof(logline.tokens.at(logline.tokens.size()-2).c_str());
  return is;
}

typedef std::vector<LogLine*> LogLinePVector;

bool readLog(istream& is, LogLinePVector& v){
  while (is){
    LogLine *l=new LogLine;
    is >> *l;
    v.push_back(l);
  }
  return true;
} 

bool writeLog(ostream& os, LogLinePVector& v){
  for (LogLinePVector::const_iterator it=v.begin(); it!=v.end(); it++){
    os << (*it)->line << endl;
  }
  return true;
} 

struct LogLineTimeComparator{
  bool operator()(const LogLine* a, const LogLine* b){
    return a->ts<b->ts;
  }
};


int main (int argc, char** argv){
  LogLinePVector v;
  if (argc<1){
    cerr << message << endl;
    return 0;
  }
  int c=1;
  while(c<argc){
    ifstream is(argv[c]);
    if (is){
      cerr << "reading \"" << argv[c] << "\"... "; 
      readLog(is,v);
      cerr << "Done. " << v.size() << " lines read" << endl;
    } else {
      cerr << "file \"" << argv[c] << "\" not foune, skipping." << endl; 
    }
    c++;
  }
  LogLineTimeComparator tc;
  cerr << "sorting" << endl;
  sort(v.begin(), v.end(), tc); 
  cerr << "writing" << endl;
  writeLog(cout,v);
  return 0;
}
