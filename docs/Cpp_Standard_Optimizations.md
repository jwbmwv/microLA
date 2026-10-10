# C++ Standard Features

MicroLA's maintained baseline is C++20. A newer language mode does not by itself
guarantee faster execution, smaller binaries, or newer standard-library features.
See the [release contract](RELEASE_CONTRACT.md) for toolchain qualification limits.

## C++20 Baseline

The headers use constexpr constructors/factories, `if constexpr`, inline variable
templates, `std::bit_cast`, constraints, and standard branch attributes. Arithmetic,
floating-point, and integral concepts are defined in the umbrella header; their
presence does not constrain every public numerical method.

```cmake
target_compile_features(my_app PRIVATE cxx_std_20)
```

```cpp
#include <microla/microla.hpp>

constexpr microla::Vec<float, 3> axis(1.0F, 0.0F, 0.0F);
constexpr auto identity = microla::Mat<float, 3, 3>::identity();
constexpr auto zero = microla::Mat<float, 3, 3>::zero();
constinit const microla::Vec<float, 3> constant_axis(1.0F, 0.0F, 0.0F);
```

`constinit` is a language keyword. There is no `MICROLA_CONSTINIT` macro.
Constants are callable proxies: both `constants::pi<float>` and
`constants::pi<float>()` work. `auto` deduces the proxy rather than a float.

## Compile-Time Rotations

Under C++20/C++23, use explicit coefficients for a fixed transform or compute
trigonometric transforms at runtime. The shipped API does not provide
`rotation_x_deg<90>()`, `rotation_z_deg<90>()`, or `rotation_deg<90>()` helpers.

```cpp
#include <microla/matrix.hpp>

constexpr microla::Mat<float, 3, 3> quarter_turn_x{
    1, 0, 0,
    0, 0, -1,
    0, 1, 0};
const auto arbitrary_turn = microla::Mat<float, 3, 3>::rotation_z(1.234F);
```

`MICROLA_CONSTEXPR_TRIG` currently follows the C++26 language-version gate in
`compiler_features.hpp`. Actual constant evaluation of `std::sin`, `std::cos`,
`std::hypot`, and other math functions also requires the selected standard library
to implement them as constexpr. Do not infer that support from a compiler version
or preview-language switch alone; compile the exact expression with the toolchain.

The C++26 form, on a supporting compiler and library, is:

```cpp
constexpr auto arbitrary_turn = microla::Mat<float, 3, 3>::rotation_z(1.234F);
```

## Feature Macros

The current header defines `MICROLA_CONSTEXPR20`, `MICROLA_CONSTEXPR23`,
`MICROLA_CONSTEXPR26`, `MICROLA_CONSTEXPR_TRIG`, `MICROLA_CONSTEVAL`,
`MICROLA_IF_CONSTEVAL`, `MICROLA_UNREACHABLE`, `MICROLA_ASSUME`,
`MICROLA_FORCEINLINE`, and `MICROLA_BIT_CAST`.

- `MICROLA_CONSTEVAL` selects consteval where supported and constexpr otherwise.
- `MICROLA_IF_CONSTEVAL` selects `if consteval` when its feature-test macro is
  available; its fallback is `if (false)`, not a general compile-time detector.
- `MICROLA_UNREACHABLE` and `MICROLA_ASSUME` are optimization contracts, not runtime
  validation. Violating their assumptions can invoke undefined behavior.
- `MICROLA_BIT_CAST` uses `std::bit_cast` with compatible object representations.
- Standard `[[likely]]` and `[[unlikely]]` annotate selected control flow. Their
  presence does not establish a measured performance benefit.

Use `std::is_constant_evaluated()` when an actual C++20 constant-evaluation check
is needed. Verify feature-test macros and library support in cross-compilers,
including compilers whose `__cplusplus` value does not fully describe their support.

## Views And Future Features

The maintained view APIs are `VectorView`/`ConstVectorView` and
`MatrixView`/`ConstMatrixView`. They use the public owning arrays (`value.data`),
not `value.data()`. Copies are provided by the view's conversion methods.

`std::mdspan`, newer subview facilities, and deducing-this refactors are possible
future work, not existing MicroLA APIs. No runtime speedup is promised for them.

## Verification

```bash
cmake --preset constexpr-cxx20
cmake --build --preset constexpr-cxx20
ctest --preset constexpr-cxx20 --output-on-failure
```

Compile the documented expression in the actual supported profile, including
embedded definitions and the selected standard library. Benchmark the compiled
application before attributing performance or size changes to a language feature.
