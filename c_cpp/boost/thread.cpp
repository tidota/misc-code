#include <boost/thread.hpp>
#include <iostream>
#include <string>
#include <vector>

boost::mutex mtx;

void func(std::string str)
{
  boost::unique_lock<boost::mutex> lk(mtx);
  std::cout << "message: " << str << std::endl;
}

int main()
{
  std::vector<std::string> lst = {"hello", "world", "foo", "bar"};
  boost::thread_group thd;

  for (int i = 0; i < lst.size(); ++i)
  {
    thd.create_thread(boost::bind(func, lst[i]));
  }
  thd.join_all();

  return 0;
}
