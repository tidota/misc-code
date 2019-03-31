#include "loader.h"

using namespace database;

template<class T>
std::shared_ptr<T> load(std::shared_ptr<T> (*createNode)(
  std::shared_ptr<T> parent, std::shared_ptr<Params> data))
{
  auto root = createNode(nullptr, std::make_shared<Params>("root"));
  auto child1 = createNode(root, std::make_shared<Params>("child1"));
  auto child2 = createNode(root, std::make_shared<Params>("child2"));
  auto child3 = createNode(root, std::make_shared<Params>("child3"));

  auto grandKid1 = createNode(child2, std::make_shared<Params>("grandKid1"));
  auto grandKid2 = createNode(child2, std::make_shared<Params>("grandKid2"));

  return root;
}