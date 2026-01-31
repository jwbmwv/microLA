#define MICROLA_LINEAR_HEADER_ONLY
#include "../include/microla/microla.hpp"
int main()
{
    microla::Vec3f v{1.0f, 2.0f, 3.0f};
    auto len = v.length();
    microla::SquareMat<float, 3> m = microla::SquareMat<float, 3>::identity();
    return 0;
}
