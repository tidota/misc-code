#include <iostream>
#include "proto/int.pb.h"
#include "proto/str.pb.h"

int main()
{
  message::Int integer;
  message::Str string;

  std::cout << "Hello World Protobuf" << std::endl;

  return 0;
}