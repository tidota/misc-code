#include <algorithm>
#include <iostream>
#include <vector>

using namespace std;

class Indx
{
private:
  int count;
public:
  Indx()
  {
    count = 0;
  }
  int operator()(){ return count++; };
};

#define prtVec(name, v) \
  cout << name << ": ";\
  for_each(v.begin(), v.end(), [](int n){ cout << n << ", "; });\
  cout << endl;\

int main()
{
  vector<int> vec1(10);

  Indx indx;
  generate(vec1.begin(), vec1.end(), indx);
  prtVec("vec1", vec1);

  auto isMulti3 = [](int n){ return n % 3 == 0; };

  cout << "is Partitioned?: ";
  cout << is_partitioned(vec1.begin(), vec1.end(), isMulti3) << endl;

  cout << "partitioning based on whether it is a multiple of 3" << endl;
  partition(vec1.begin(), vec1.end(), isMulti3);
  prtVec("vec1", vec1);

  cout << "is Partitioned?: ";
  cout << is_partitioned(vec1.begin(), vec1.end(), isMulti3) << endl;

  cout << "restoring" << endl;
  generate(vec1.begin(), vec1.end(), indx);

  cout << "stable partitioning" << endl;
  stable_partition(vec1.begin(), vec1.end(), isMulti3);
  prtVec("vec1", vec1);

  cout << "shuffling" << endl;
  random_shuffle(vec1.begin(), vec1.end());
  prtVec("vec1", vec1);
  vector<int> multi3;
  vector<int> nonMulti3;
  partition(vec1.begin(), vec1.end(), isMulti3);
  int n = count_if(vec1.begin(), vec1.end(), isMulti3);
  multi3.resize(n);
  nonMulti3.resize(vec1.size() - n);
  partition_copy(
    vec1.begin(), vec1.end(), multi3.begin(), nonMulti3.begin(), isMulti3);
  prtVec("vec1", vec1);
  prtVec("multi3", multi3);
  prtVec("nonMulti3", nonMulti3);

  auto it = partition_point(vec1.begin(), vec1.end(), isMulti3);
  multi3.clear();
  prtVec("multi3", multi3);
  multi3.assign(vec1.begin(), it);
  prtVec("multi3", multi3);

}
