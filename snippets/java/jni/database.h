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

class NodeImpl; // forward declaration

std::shared_ptr<NodeImpl> load(std::shared_ptr<NodeImpl> (*createNode)(
  std::shared_ptr<NodeImpl> parent, std::shared_ptr<Params> data));

}
#endif // _LOADER_H
