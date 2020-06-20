#include <iostream>
#include "proto/intlist.pb.h"
#include "proto/str.pb.h"

int main()
{
  customMess::Intlist integer;
  customMess::Str string;

  std::cout << "Hello World Protobuf" << std::endl;

  return 0;
}