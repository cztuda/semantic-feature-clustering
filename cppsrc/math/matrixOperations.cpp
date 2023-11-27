
#include "matrixOperations.h"
#include <vector>
#include <utility>
#define HASH_SIZE 128

namespace ofc::math {


void transposeMatrix(double* array, const size_t len, const size_t nrows){
    const int mn1 = len - 1;
    const int ncols   = len / nrows;
    std::vector<bool> visited(len);
    int cycle = 0;
    while (++cycle != len) {
        if (visited[cycle])
            continue;
        int a = cycle;
        do  {
            a = a == mn1 ? mn1 : (ncols * a) % mn1;
            std::swap(array[a], array[cycle]);
            visited[a] = true;
        } while ((a) != cycle);
    }
}


}
