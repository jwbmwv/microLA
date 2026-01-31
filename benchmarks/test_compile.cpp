#define MICROLA_LINEAR_HEADER_ONLY
#include "../include/microla/microla.hpp"
int main()
{
    using microla::Vec;
    using Vec3f = Vec<float, 3>;

    Vec3f v{1.0f, 2.0f, 3.0f};
    auto len = v.length();
    microla::SquareMat<float, 3> m = microla::SquareMat<float, 3>::identity();
    (void)len;
    (void)m;
    return 0;
}
