#include <algorithm>
#include <iostream>
#include <vector>

using namespace std;

class Indx
{
  int count;
public:
  Indx(){ count = 0; }
  int operator()(){ return count++; }
};

int main()
{
  vector<int> vec1(10);

  cout << "filling vec1 with a sequence of 3" << endl;
  fill(vec1.begin(),vec1.end(),3);

  cout << "vec1: ";
  for_each(vec1.begin(), vec1.end(), [](int n){ cout << n << ", "; });
  cout << endl;

  cout << "filling vec1 at third - fifth slots with 5" << endl;
  fill_n(vec1.begin() + 2, 3, 5);

  cout << "vec1: ";
  for_each(vec1.begin(), vec1.end(), [](int n){ cout << n << ", "; });
  cout << endl;

  Indx indx;
  cout << "generating increasing numbers in vec1" << endl;
  generate(vec1.begin(), vec1.end(), indx);
  cout << "vec1: ";
  for_each(vec1.begin(), vec1.end(), [](int n){ cout << n << ", "; });
  cout << endl;
  cout << "generating increasing numbers in vec1 from the 3rd to 5th" << endl;
  generate_n(vec1.begin() + 2, 5, indx);
  cout << "vec1: ";
  for_each(vec1.begin(), vec1.end(), [](int n){ cout << n << ", "; });
  cout << endl;

  cout << "removing 3 from vec1" << endl;
  {
    auto pend = remove(vec1.begin(), vec1.end(), 3);
    vec1.resize(distance(vec1.begin(), pend));
  }
  cout << "vec1: ";
  for_each(vec1.begin(), vec1.end(), [](int n){ cout << n << ", "; });
  cout << endl;

  cout << "removing even numbers from vec1" << endl;
  {
    auto pend
      = remove_if(vec1.begin(), vec1.end(), [](auto n){ return n % 2 == 0; });
    vec1.resize(distance(vec1.begin(), pend));
  }
  cout << "vec1: ";
  for_each(vec1.begin(), vec1.end(), [](int n){ cout << n << ", "; });
  cout << endl;

  cout << "re-generate increasing numbers to vec1" << endl;
  vec1.resize(10);
  generate(vec1.begin(), vec1.end(), indx);
  cout << "define vec2" << endl;
  vector<int> vec2(20);

  cout << "removing 3 in vec1 and copy them to vec2" << endl;
  {
    auto pend = remove_copy(vec1.begin(), vec1.end(), vec2.begin(), 3);
    vec2.resize(distance(vec2.begin(), pend));
  }
  cout << "vec2: ";
  for_each(vec2.begin(), vec2.end(), [](int n){ cout << n << ", "; });
  cout << endl;
  cout << "removing odd numbers in vec1 and copy them to vec2" << endl;
  {
    auto pend
      = remove_copy_if(
        vec1.begin(), vec1.end(),
        vec2.begin(), [](int n){ return n % 2 == 1; });
    vec2.resize(distance(vec2.begin(), pend));
  }
  cout << "vec2: ";
  for_each(vec2.begin(), vec2.end(), [](int n){ cout << n << ", "; });
  cout << endl;

  cout << "vec3 (3, 3, 3, 4, 4, 7, 1, 1, 3, 4)" << endl;
  vector<int> vec3 = {3, 3, 3, 4, 4, 7, 1, 1, 3, 4};
  cout << "applying unique" << endl;
  {
    auto pend = unique(vec3.begin(), vec3.end());
    vec3.resize(distance(vec3.begin(), pend));
  }
  cout << "vec3: ";
  for_each(vec3.begin(), vec3.end(), [](int n){ cout << n << ", "; });
  cout << endl;

  cout << "applying unique with predicate" << endl;
  {
    auto pend
      = unique(vec3.begin(), vec3.end(), [](int a, int b){ return a+1 == b; });
    vec3.resize(distance(vec3.begin(), pend));
  }
  cout << "vec3: ";
  for_each(vec3.begin(), vec3.end(), [](int n){ cout << n << ", "; });
  cout << endl;

  cout << "generate => vec3" << endl;
  vec3.resize(10);
  generate(vec3.begin(), vec3.end(), indx);
  cout << "unique vec3 => vec2" << endl;
  vec2.resize(10);
  {
    auto pend
      = unique_copy(
        vec3.begin(), vec3.end(), vec2.begin(),
        [](int a, int b){ return a+1 == b; });
    vec2.resize(distance(vec2.begin(), pend));
  }
  cout << "vec2: ";
  for_each(vec2.begin(), vec2.end(), [](int n){ cout << n << ", "; });
  cout << endl;

  cout << "vectors so far" << endl;
  cout << "vec1: ";
  for_each(vec1.begin(), vec1.end(), [](int n){ cout << n << ", "; });
  cout << endl;
  cout << "vec2: ";
  for_each(vec2.begin(), vec2.end(), [](int n){ cout << n << ", "; });
  cout << endl;
  cout << "vec3: ";
  for_each(vec3.begin(), vec3.end(), [](int n){ cout << n << ", "; });
  cout << endl;

  cout << "reversing vec1" << endl;
  reverse(vec1.begin(), vec1.end());
  cout << "vec1: ";
  for_each(vec1.begin(), vec1.end(), [](int n){ cout << n << ", "; });
  cout << endl;
  cout << "reversing vec1 and copy to vec2" << endl;
  vec2.resize(vec1.size());
  reverse_copy(vec1.begin(), vec1.end(), vec2.begin());
  cout << "vec2: ";
  for_each(vec2.begin(), vec2.end(), [](int n){ cout << n << ", "; });
  cout << endl;

  cout << "rotating vec1 by 4" << endl;
  rotate(vec1.begin(), vec1.begin() + 4, vec1.end());
  cout << "vec1: ";
  for_each(vec1.begin(), vec1.end(), [](int n){ cout << n << ", "; });
  cout << endl;

  cout << "roating vec1 by -4 and copy to vec2" << endl;
  rotate_copy(vec1.begin(), vec1.begin() + (10 - 4), vec1.end(), vec2.begin());
  cout << "vec2: ";
  for_each(vec2.begin(), vec2.end(), [](int n){ cout << n << ", "; });
  cout << endl;

  cout << "shuffling vec1" << endl;
  random_shuffle(vec1.begin(), vec1.end());
  cout << "vec1: ";
  for_each(vec1.begin(), vec1.end(), [](int n){ cout << n << ", "; });
  cout << endl;


}
