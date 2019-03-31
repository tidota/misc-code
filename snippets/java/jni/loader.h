#ifndef _LOADER_H
#define _LOADER_H

#include <memory>
#include <string>

namespace database
{

class Params
{
public:
  std::string name;
  Params(const std::string& _name): name(_name){}
};

template<class T>
std::shared_ptr<T> load(
  std::shared_ptr<T> (*)(std::shared_ptr<T>, std::shared_ptr<Params>));

}
#endif // _LOADER_H
