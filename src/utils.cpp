#include <math.h>
// #include <cstring>
#include <iomanip>
#include <sstream>
#include <fstream>


double angular_distance(double a[2], double b[2]) {
    // approximate with Euclidean distance (correct for small angles)
    return std::sqrt((a[0]-b[0])*(a[0]-b[0])+(a[1]-b[1])*(a[1]-b[1]));
}

std::string ZeroPadNumber2Str(int num, int width)
{
    std::ostringstream ss;
    ss << std::setw( width ) << std::setfill( '0' ) << num;
    return ss.str();
}

double exp_smoothing (double inp, double prev, double alpha = 0.05) {
    return alpha * inp + (1 - alpha) * prev;
}

template <typename T>
int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

void lerp_arr(double* a, double* b, double t, double* output, uint len) {
    for (int i = 0; i < len; i++){
        output[i] = (b[i] - a[i]) * t;
    }
}