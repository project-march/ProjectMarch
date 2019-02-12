//
// Class source for general-purpose GES
// Derived from Slave class
//

#include "GES.h"

// Constructor
GES::GES(std::string name, uint16 number) : Slave(name, number)
{
  type = "GES";
}
