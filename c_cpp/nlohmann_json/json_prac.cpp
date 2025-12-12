#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>

using json = nlohmann::json;

int main()
{
  std::cout << "Creating json data" << std::endl;
  json j;
  j.push_back(json::parse("{\"value1\": \"string\"}"));
  j.push_back(json::parse("{\"value2\": \"string\"}"));
  
  std::cout << "Saving the data into data.json" << std::endl;
  std::ofstream ofile("data.json");
  ofile << j;
  ofile.close();

  std::cout << "  ----------------------" << std::endl;

  std::cout << "Reading data from data.json" << std::endl;
  std::ifstream ifile("data.json");
  // This line is equivalent to: json j_file; ifile >> j_file;
  json j_file = json::parse(ifile);
  ifile.close();
  std::cout << "data:" << std::endl;
  std::cout << j_file << std::endl;

  std::cout << "  ----------------------" << std::endl;

  std::cout << "Saving an empty file" << std::endl;
  std::ofstream ofile_emp("empty.json");
  ofile_emp.close();

  std::cout << "  ----------------------" << std::endl;

  std::cout << "Attempting to open the empty file and handling the exception" << std::endl;
  std::ifstream ifile_emp("empty.json");
  try {
    json j_file = json::parse(ifile_emp);
  } catch (const std::exception & e) {
    std::cout << "Handling the exception. e.what() = " << e.what() << std::endl;
  }
  ifile_emp.close();

  std::cout << "  ----------------------" << std::endl;

  std::cout << "Attempting to access a non-existing item" << std::endl;
  try {
    json empty_obj = json::object();
    auto val = empty_obj["foo"].get<std::string>();
    std::cout << "val = " << val << std::endl;
  } catch (const std::exception & e) {
    std::cout << "std::exception, e.what() = " << e.what() << std::endl;
  }

  std::cout << "  ----------------------" << std::endl;
  return 0;
}

