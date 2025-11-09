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

  std::cout << "Reading data from data.json" << std::endl;
  std::ifstream ifile("data.json");
  json j_file = json::parse(ifile);
  ifile.close();
  std::cout << "data:" << std::endl;
  std::cout << j_file << std::endl;

  return 0;
}

