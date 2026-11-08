// reads lines "op a b" and prints the result as repr, for comparison with Python
#include "../src/py.h"
#include <iostream>
#include <sstream>
int main() {
    std::string line;
    while (std::getline(std::cin, line)) {
        std::istringstream in(line);
        std::string op; std::string as, bs; in >> op >> as >> bs;
        double a = std::strtod(as.c_str(), nullptr), b = bs.empty() ? 0 : std::strtod(bs.c_str(), nullptr);
        std::string r;
        if (op == "hypot") r = py::repr(py::hypot(a, b));
        else if (op == "floordiv") r = py::repr(py::floordiv(a, b));
        else if (op == "mod") r = py::repr(py::mod(a, b));
        else if (op == "round") r = std::to_string(py::round_int(a));
        else if (op == "round1") r = py::repr(py::round_nd(a, (int)b));
        else if (op == "repr") r = py::repr(a);
        else if (op == "pow") r = py::repr(py::pow(a, b));
        else if (op == "f6") r = py::fmt_f(a, 6);
        std::cout << r << "\n";
    }
}
