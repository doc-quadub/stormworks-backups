#include "../src/value.h"
#include <iostream>
int main(int argc, char** argv) {
    Value v = json::load(argv[1]);
    json::dump(v, argv[2]);
}
