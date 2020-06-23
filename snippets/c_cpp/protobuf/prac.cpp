#include <iostream>
#include <proto/intlist.pb.h>
#include <proto/str.pb.h>

int main()
{
  customMess::Intlist integer;
  customMess::Str string;

  std::cout << "Hello World Protobuf" << std::endl;

  for (int i = 0; i < 5; ++i)
    integer.add_val(i);
  for (int i = 0; i < integer.val_size(); ++i)
    std::cout << "integer[" << i << "]: " << integer.val(i) << std::endl;

  string.set_str("hello?");
  std::cout << "str: " << string.str() << std::endl << std::endl;

  {
    std::string buff;
    integer.SerializeToString(&buff);
    customMess::Intlist anotherInteger;
    anotherInteger.ParseFromString(buff);
    std::cout << "serialized: " << buff << std::endl;
    for (int i = 0; i < anotherInteger.val_size(); ++i)
      std::cout << "anotherInteger[" << i << "]: "
        << anotherInteger.val(i) << std::endl;
  }

  {
    std::string buff;
    string.SerializeToString(&buff);
    customMess::Str anotherString;
    anotherString.ParseFromString(buff);
    std::cout << "serialized: " << buff << std::endl;
    std::cout << "anotherString: " << anotherString.str() << std::endl;
  }

  // bool SerializeToString(string* output) const;: serializes the message and stores the bytes in the given string. Note that the bytes are binary, not text; we only use the string class as a convenient container.
  // bool ParseFromString(const string& data);: parses a message from the given string.
  // bool SerializeToOstream(ostream* output) const;: writes the message to the given C++ ostream.
  // bool ParseFromIstream(istream* input);: parses a message from the given C++ istream.

  google::protobuf::ShutdownProtobufLibrary();

  return 0;
}

