#pragma once

// Some handy macros
// Author: Shayan Hoshyari

// Use this to get rid of unused variable warnings.
#define MINIMESH_UNUSED(x) (void)(x)

// You can use this macro as a shorthand for std::stringstream
// example: std::string name = MINIMESH_STR("run_number_" << i << ".vtk");
#define MINIMESH_STR(X) static_cast<std::ostringstream&>(std::ostringstream().flush() << X).str()
