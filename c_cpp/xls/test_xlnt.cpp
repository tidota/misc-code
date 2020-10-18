#include <iostream>

#include <xlnt/xlnt.hpp>

int main()
{
  std::string filename = "./example.xlsx";
  try
  {
    std::cout << "Opening: " << filename << std::endl;
    xlnt::workbook wb;
    wb.load(filename);
    xlnt::worksheet ws = wb.active_sheet();
    std::cout << "A1: " << ws.cell(1,1).value<int>() << std::endl;
  }
  catch(...)
  {
    std::cout << "File not found: " << filename << std::endl;
  }
  std::cout << "Writing: " << filename << std::endl;
  xlnt::workbook wb;
  xlnt::worksheet ws = wb.active_sheet();
  ws.cell("A1").value(5);
  ws.cell("B2").value("string data");
  ws.cell("C3").formula("=RAND()");
  ws.cell(10,10).value("insersion by indexes?");
  ws.merge_cells("C3:C4");
  ws.freeze_panes("B2");
  wb.save(filename);
  return 0;
}
// compile with -std=c++14 -Ixlnt/include -lxlnt
