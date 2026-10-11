# MicroLA Reconstruction Design

## 1. Purpose and interpretation

This document specifies how to recreate MicroLA as a new, independently
implemented, header-only C++ library. It describes the observable design of
version **0.0.4**, inspected at commit
`303c9965500e23d05d04c621e876e23e7b308b10`, rather than assuming that every
claim in older documentation is implemented.

The deliverable is a portable fixed-size mathematics library, not a wrapper
around Eigen, GLM, CMSIS-DSP, or an operating system. A replacement must provide
the value types, numerical operations, geometry, non-owning views, filtering,
sensor fusion, build integration, and validation strategy specified below.

Three labels distinguish requirements from possible improvements:

- **Contract:** observable behavior or an integration boundary that a compatible
  reconstruction must preserve.
- **Implementation:** the algorithm or organization used by the inspected
  version; replaceable if the contract and acceptance tests still hold.
- **Next-version decision:** a limitation or inconsistent behavior that must be
  resolved explicitly, not accidentally copied or silently changed.

Source references are an audit trail, not a requirement to reuse the old
implementation. The mathematical definitions, state transitions, and acceptance
criteria in this document are intended to remain useful without the original
source tree.

### Contents

- [Purpose and interpretation](#1-purpose-and-interpretation)
- [System architecture](#2-system-architecture)
- [Mathematical and representation conventions](#3-mathematical-and-representation-conventions)
- [Core mathematics and view contracts](#4-core-mathematics-and-view-contracts)
- [Generic state estimation](#5-generic-state-estimation)
- [Sensor-fusion data model and configuration](#6-sensor-fusion-data-model-and-configuration)
- [Per-entity orientation state machine](#7-per-entity-orientation-state-machine)
- [Relative-angle estimator](#8-relative-angle-estimator)
- [Numerical utility and platform contracts](#9-numerical-utility-and-platform-contracts)
- [Packaging and build integration](#10-packaging-and-build-integration)
- [Verification and acceptance](#11-verification-and-acceptance)
- [Implementation sequence](#12-implementation-sequence)
- [Reconstruction smoke program](#13-reconstruction-smoke-program)
- [Evidence and maintenance](#14-evidence-and-maintenance)

### 1.1 Intended users

- Embedded firmware using small vectors and matrices in control, robotics,
  inertial sensing, and real-time estimation.
- Host applications simulating, testing, or replaying those workloads.
- Applications requiring fixed storage, compile-time dimension checks, and
  explicit control over floating-point accuracy and optional acceleration.

### 1.2 Goals and non-goals

| Area | Required design | Not promised |
|------|-----------------|--------------|
| Distribution | Header-only templates and inline functions | A mandatory compiled runtime |
| Language | C++20 minimum; later facilities opportunistic | C++11/14/17 compatibility |
| Dimensions | Compile-time positive dimensions | Dynamically sized matrices |
| Storage | Inline value storage; caller-owned views | A general-purpose allocator |
| Portability | Scalar reference implementation first | Identical floating-point bits on every backend |
| Performance | Bounded small-matrix work; measurable resource use | Universal speedups or a hardware-independent WCET |
| Safety | Checked APIs where specified; explicit fusion metadata | Safety certification or automatic recovery from arbitrary misuse |
| Sensors | Attitude and relative angles from accel/IMU samples | Position estimation, gravity-free tilt, or drift-free 6-axis yaw |
| Dependencies | Standard library for the scalar library | Mandatory units, ETL, Eigen, GLM, or an RTOS dependency |
| Interoperability | Contiguous data access and layout traits | Portable serialization of raw object bytes |

“No allocation” applies to the fixed-size core and estimator working paths, not
to every optional convenience overload or to the application as a whole.
“Zero overhead” is an optimization objective, not a guarantee that every
abstraction disappears on every compiler.

### 1.3 Minimal compatibility check

This standalone program is also a documentation-compilation gate:

```cpp
#include <microla/vector.hpp>

int main()
{
    constexpr microla::Vec<float, 3> value(1.0F, 2.0F, 3.0F);
    static_assert(value[2] == 3.0F);
    return value.size() == 3 ? 0 : 1;
}
```

## 2. System architecture

### 2.1 Required module boundaries

Public headers live under `include/microla/`; public names live in `microla`
except for the `microla::fusion` sensor-fusion namespace and documented utility
subnamespaces.

| Header | Responsibility |
|--------|----------------|
| `compiler_features.hpp` | Language/compiler gates, attributes, alignment, embedded policy |
| `constants.hpp` | Typed mathematical constants and precision traits |
| `high_precision.hpp` | Higher-precision intermediate type selection |
| `simd_helpers.hpp` | Optional acceleration and portable scalar kernels |
| `vector.hpp` | `Vec<T,N>` owning vectors |
| `matrix.hpp` | `Mat<T,R,C>`, `SquareMat<T,N>`, transforms and factorizations |
| `quaternion.hpp` | Hamilton quaternions and rotation conversions |
| `vector_view.hpp` | Fixed-size strided vector views |
| `matrix_view.hpp` | Fixed-size matrix views and blocks |
| `safe_math.hpp` | Explicitly guarded scalar operations |
| `fast_math.hpp` | Opt-in approximation functions |
| `numerical_stability.hpp` | Compensated accumulation and stable numerical helpers |
| `resource_checks.hpp` | Compile-time storage and layout inspection |
| `geometry.hpp` | Primitive geometry and intersection tests |
| `kalman.hpp` | Linear Kalman filter |
| `extended_kalman.hpp` | Callback-based nonlinear EKF |
| `sensor_fusion.hpp` | Per-entity attitude and pairwise relative-angle estimators |
| `version.hpp` | Compile-time version macros and runtime version queries |
| `microla.hpp` | Convenience inclusion, concepts, angle helpers, memory helpers |

The umbrella includes vectors, matrices, views, quaternions, guarded/fast/stable
math, resources, both Kalman filters, and sensor fusion. It does **not** include
geometry or version queries; applications include those explicitly.

### 2.2 Dependency direction

```text
compiler features -> constants / precision / SIMD
constants / SIMD -> vector
vector -> matrix
vector + matrix -> quaternion
vector / matrix -> views
vector + matrix + precision -> geometry
vector + matrix -> linear KF / generic EKF
vector + matrix + quaternion -> sensor fusion
all convenience modules -> umbrella
```

Keep dependencies acyclic. Each public header must compile as the first and only
project include in an otherwise empty translation unit. Do not depend on an
umbrella header, accidental include order, or a precompiled header for correctness.

### 2.3 Ownership and concurrency

- Owning values copy their elements; returning a result must not reference a
  temporary or operand.
- Views borrow storage and never extend its lifetime.
- Filters own all persistent state and fixed-size scratch values.
- Const read-only operations may run concurrently when underlying storage is
  not being modified.
- Mutations and borrowed mutable storage require external synchronization.
- Estimators are not internally synchronized. ISR/task sharing requires an
  ownership protocol; no lock, scheduler, driver, or acquisition thread belongs
  in the library.
- Calibration changes, resets, and updates are state mutations, not thread-safe
  configuration operations.

## 3. Mathematical and representation conventions

### 3.1 Scalars and dimensions

The principal defaults are `Vec<float,3>`, `Mat<float,4,4>`, and
`Quaternion<float>`. `SquareMat<T,N>` aliases `Mat<T,N,N>`.
Generic core arithmetic supports
arithmetic scalar types where the operation is meaningful; attitude estimation
requires floating-point `T`.

Dimensions and multiplication shapes are part of the type:

- `Mat<T,R,C> * Vec<T,C> -> Vec<T,R>`.
- `Mat<T,R,C> * Mat<T,C,K> -> Mat<T,R,K>`.
- `transpose(Mat<T,R,C>) -> Mat<T,C,R>`.
- Incompatible dimensions must fail at compile time.
- Floating-point-oriented operations on integral types do not magically become
  fractional arithmetic; a replacement must document the supported domain rather
  than imply that an integer inverse or normalization is generally useful.

### 3.2 Coordinates and composition

- Cross products and positive rotations follow the right-hand rule.
- Matrices store elements in **row-major** order, indexed `r*C+c`.
- Matrix-vector application uses column-vector mathematics: `v_out = M*v_in`.
- `A*B*v` applies `B` first.
- Homogeneous points use `w=1`; directions use `w=0`.
- A quaternion is Hamilton `w + xi + yj + zk`.
- Quaternion scalar constructor order and physical storage order are distinct;
  never infer one from the other.
- `q1*q2` composes rotations by applying `q2` first, then `q1`.
- State-estimator orientations map **body to world**. Their increments multiply
  on the right because angular rates and attitude errors are body-frame quantities.
- Fusion defaults are Z-up world, gravity/down along `-Z`, heading along `+X`.
  They are policy choices, not hard-wired requirements for all geometry clients.

For roll `r`, pitch `p`, and yaw `y`, specify conversions by the actual matrix
identity `R = Rz(y)*Ry(p)*Rx(r)`, not by an ambiguous “XYZ” label. Euler outputs
are ordered `(roll,pitch,yaw)` and measured in radians. Gimbal-lock handling and
range choices are part of quaternion compatibility.

### 3.3 Units

Core vectors and matrices are dimensionless containers; they do not enforce units.
Fusion boundaries use:

| Quantity | Unit |
|----------|------|
| Timestamp, interval, age, pair skew | Seconds |
| Angular velocity and adaptive gyro bias | Radians/second |
| Acceleration | Metres/second squared |
| Angles, drift, attitude error | Radians |
| Magnetic input | Any consistent calibrated field-strength unit |
| MEKF attitude/bias covariance | Products of the corresponding state units |

Applications with a units library convert at the boundary. Do not require a units
package or assume that every magnetometer supplies microtesla. Sensor drivers must
also reconcile acceleration sign with the library's gravity-direction convention.

### 3.4 Memory and ABI

Core values must be standard-layout and trivially copyable for supported ordinary
scalar instantiations. Contiguous scalar storage is the interop boundary. Padding
and alignment remain target-dependent, so use `sizeof`, `alignof`, and the public
storage/accessor (`data`, `data_ptr()`, or quaternion `ptr()`),
not `N*sizeof(T)` or raw object serialization assumptions.

No virtual functions, reference counting, hidden heap ownership, or per-element
indirection are required. SIMD must not read beyond the real element count of a
three-element object just to fill a four-lane register.

All translation units exchanging these types must use the same layout-affecting
configuration macros. Mixing scalar/embedded/SIMD alignment definitions can break
ABI and the one-definition rule even though the library is header-only. Define
configuration at the build-target boundary, before any public header is included.

## 4. Core mathematics and view contracts

The owning vector, matrix, quaternion, geometry, and view specifications are
organized below independently of filtering so they can be implemented and tested
first.

### 4.1 Owning vectors

Evidence: [vector implementation](../include/microla/vector.hpp).

`Vec<T=float,N=3>` owns public aligned `T data[N]`, default initialized to zero.
Require arithmetic `T` and positive `N` at class instantiation. Default special
members preserve trivial copying/layout.

#### Construction and access

Provide default, explicit pointer-copy, fixed-array-copy, initializer-list,
exact-component variadic, and separate one-component scalar constructors.
Pointer construction reads exactly `N` values; fixed arrays require length at
least `N`. Initializer lists truncate excess and zero-fill missing values.
Variadic construction for `N>=2` requires exactly `N` components and converts to
`T`; a single scalar is not a general broadcast constructor.

Expose unchecked `operator[]`, mutable/const `at`, `data_ptr`, `begin/end`,
`cbegin/cend`, static `size`, and component index constants `X,Y,Z,W`.
`at` asserts under `MICROLA_DEBUG` rather than throwing. Named `x/y/z/w`
accessors return references where the dimension permits. Factories are
`zero`, `one`, `filled(value)`, and available `unit_x/y/z/w`.

#### Arithmetic and comparison

Provide same-type/same-size addition/subtraction/negation, scalar right multiply
and divide, left scalar multiply, and compound arithmetic returning `Vec&`.
There is no general vector-vector multiplication operator: `hadamard` and alias
`elem_mult` name componentwise multiplication.

Dot is `sum(a_i*b_i)` and also `operator|`. Three-dimensional cross and
`operator^` use the right-handed determinant formula.
`fma(vector_mul,add)` and `fma(scalar_mul,add)` evaluate ordinary `a*b+c`;
the name does not guarantee a hardware-fused single rounding. `scale` is an
additional scalar-operation helper.

Integral equality is exact. Floating equality and `approx_equal` use absolute
component tolerance, default machine epsilon. `is_zero` and `is_one` default to
eight machine epsilons for floating scalars. Non-finite comparisons are not
special-cased: NaN differences and infinity-minus-itself fail. Tolerances are not
validated. `is_finite` checks all components.

#### Division, norm, and degeneracy

- Value division by zero asserts in debug, otherwise fills with `quiet_NaN`;
  integral `quiet_NaN` ordinarily means zero.
- Floating compound division by zero fills NaNs.
- Integral compound division by zero leaves the vector unchanged.
- Nonzero integral/constexpr division uses direct division. Runtime floating
  paths usually multiply by reciprocal; tiny divisors can overflow that reciprocal.

`length_squared` is dot with self; `magnitude_squared` aliases it.
`length`/`magnitude` first use squared sum/sqrt but switch floating input to chained
hypot when the square is non-finite or below the smallest positive normal.

Floating `normalized` returns non-finite or zero input unchanged; otherwise divide
by the largest absolute component and normalize the scaled vector. This must work
on large, tiny, and subnormal finite directions.
`safe_normalized(epsilon=machine_epsilon)` is distinct: compute length, return zero
when length is below epsilon, otherwise use value division. It does not guarantee
the same scale robustness.

#### Geometric and component helpers

| API | Definition / boundary |
|-----|-----------------------|
| `angle(other)` | `acos(clamp(dot/(lengths product),-1,1))`; zero denominator returns zero |
| `signed_angle(other,normal)` | Signed acos using cross dot normal; 3D; no matching zero-length guard |
| `project(other)` | `other*dot(other)/dot(other,other)`; zero target gives zero |
| `reject(other)` | `self-project(other)` |
| `distance(other)` | Direct sqrt of squared component differences, without hypot fallback |
| `rotate(axis,angle)` | 3D Rodrigues formula; caller must supply unit axis |
| `lerp(other,t)` | `self+t*(other-self)` |
| `cubic_hermite(other,tangent1,tangent2,t)` | Standard endpoint/tangent cubic |
| `catmull_rom(other,p0,p2,t)` | Segment between self/other, with preceding/following controls |

Hermite basis coefficients are `2t^3-3t^2+1`, `t^3-2t^2+t`,
`-2t^3+3t^2`, `t^3-t^2`. Catmull-Rom is:

```text
0.5*(2*a + (-p0+b)*t
     + (2*p0-5*a+4*b-p2)*t*t
     + (-p0+3*a-3*b+p2)*t*t*t)
```

Interpolation parameters are not clamped.
Component utilities include `min_element`, `max_element`, `abs`, componentwise
`min/max`, `clamp`, `clamped`, and `saturated`. The two clamp implementations can
differ on reversed bounds; ordered bounds are the intended domain.

Copy swizzles are `xy,yx`; `xyz,xz,yz,zx,zy`; and
`xzy,yxz,yzx,zxy,zyx`. They are value copies, not writable proxies.
`to_homogeneous` appends one. `from_homogeneous` drops the last component and
divides preceding components only if `abs(w)>machine_epsilon`; otherwise just
drops `w`. Provide `Vec2/3/4` aliases ending `f,d,i,u`.

### 4.2 Owning matrices

Evidence: [matrix implementation](../include/microla/matrix.hpp).

`Mat<T=float,R=4,C=4>` owns public aligned `T data[R*C]`, zero initialized.
`SquareMat<T,N>` is an alias, not an inheritance hierarchy.
Positive dimensions and arithmetic scalar domains are intended; the inspected
class does not impose vector-like class-level assertions on every instantiation.
Adding those checks is an explicit tightening of the next-version contract.

#### Construction and access

Provide default, exactly `R*C` variadic components, flat initializer-list,
and nested row initializer-list construction. Lists assert exact dimensions.
Malformed lists with assertions disabled are not a checked-input API: excess can
overrun storage and missing elements can remain zero. There is no matrix pointer
or array-copy constructor in the inspected public surface.

Expose `rows`, `cols`, `size`, `data_ptr`, row-pointer `operator[]`, and
`operator()(row,col)`. There is no general matrix iterator family.
`mat_traits<T,R,C>` contains scalar/matrix aliases, dimensions, payload byte size,
and square/floating/integral predicates. Payload size excludes padding.

Factories are square `identity`, square `identity_cached`, `zero`, `ones`,
and square `diagonal(vector)` with matching dimension. `identity_cached` returns
a persistent function-static const matrix reference, not a value.

`block<BR,BC>(start_row,start_col)` returns an owning matrix copy.
`set_block`, `row`, `col`, `set_row`, and `set_col` use ordinary assertions for
extent checks. Row and column return `Mat<T,1,C>` and `Mat<T,R,1>`, not vectors
or borrowed views.

#### Arithmetic and norms

Provide elementwise `+,-,+=,-=`, scalar right `*,/` and compound forms,
static `scale(matrix,scalar)`, transpose, typed matrix multiplication, and
typed matrix-vector multiplication. Do not invent unary negation, scalar-left
multiplication, matrix compound multiplication, or `approx_equal` members that
are absent from the inspected interface.

Scalar matrix division is direct per-element division with no zero guard.
Multiplication uses ordinary accumulation in `T`. Float scalar fallback streams
row/inner/column loops; large nonfloat products use tiles of 32 when all dimensions
are at least 32. Complexity is `O(R*C*K)`.

Floating equality accepts absolute error at most epsilon or finite relative error
at most `100*epsilon*max(abs(a),abs(b))`. Other scalar equality is exact.
Provide `is_finite`, square `is_identity(epsilon=8*machine_epsilon)`, square trace,
Frobenius norm, infinity/max-row-sum norm and aliases, one/max-column-sum norm and
aliases, and square `condition_number`.

Frobenius norm is direct sqrt of sum of squares, not scale-safe hypot.
`condition_number` uses `norm_inf(A)*norm_inf(A.inverse())`, inheriting inverse
failure and possible overflow behavior.
Aliases cover 2/3/4 square and all distinct 2/3/4 rectangular shapes in float/double;
there are no corresponding standard integer matrix aliases.

#### Rotation constructors

3x3 `rotation_x/y/z` are active right-handed principal rotations.
2x2 `rotation(angle)` uses cosine/sine. 3x3 `rotation_axis_angle` normalizes its
axis and uses Rodrigues' matrix. `rotation_from_to` normalizes directions,
returns identity near parallel (`dot>=0.999999`), chooses an orthogonal axis for
near antiparallel (`dot<=-0.999999`), otherwise rotates about normalized cross
by acos(dot).

`euler_angles` extracts roll `atan2(A21,A22)`, pitch `asin(-A20)`, and yaw
`atan2(A10,A00)`, without clamp or special gimbal-lock handling.
`look_at(target,up)` constructs forward, right=`normalize(forward cross up)`,
actual_up=`normalize(right cross forward)` and stores these as rows.

**Next-version decision:** this historical look-at basis has determinant minus
one for ordinary nondegenerate inputs; it is not a proper rotation. Define and
test the intended handedness, forward direction, row/column convention, and
determinant before retaining or correcting this behavior.

### 4.3 Inversion, decompositions, and eigenvalues

These APIs are numerical algorithms, not guarantees that every arithmetic scalar
or every ill-conditioned matrix is supported.

#### Determinant and inverse

Square determinant uses direct formulas for sizes 1/2/3. Larger integral matrices
use Bareiss fraction-free elimination with first nonzero pivot and row-swap sign;
work type widens to int64 when `T` is smaller. Larger nonintegral matrices use
partial-pivot Gaussian elimination. Singular pivots are checked for exact zero;
general overflow/non-finite protection is not provided by determinant.

Public inverse variants:

- `inverse(Mat& out) -> bool`.
- `inverse_checked(Mat& out,T threshold=1e6) -> bool`.
- `inverse() -> Mat`.
- Hosted `try_inverse() -> optional<Mat>`.
- Hosted `inverse_checked(T threshold=1e6) -> variant<Mat,InverseError>`,
  errors `Singular,IllConditioned`.

Require square dimensions and inversion payload strictly below 16384 bytes.
Handle output/source aliasing with a source copy. For floating input, reject
non-finite/all-zero values; divide by maximum magnitude, invert normalized matrix,
then rescale inverse. Use explicit 2x2 adjugate, 3x3 cofactors, 4x4 minors/cofactors,
and partial-pivot augmented Gauss-Jordan otherwise, including 1x1.
Return false on zero determinant/pivot or non-finite result.

Unchecked boolean inversion can modify output before detecting final failure.
Checked inversion computes a private candidate and preserves output on rejection.
Its condition estimate avoids overflow in unscaled row sums:

```text
s = max_abs_entry(A)
condition = max_row_sum(abs(A)/s) * max_row_sum(abs(invA)*s)
```

Reject non-finite condition, threshold not at least one, or condition above
threshold. NaN threshold rejects; infinite threshold may accept finite condition.
Variant inversion maps any failed base inversion to Singular, otherwise rejected
condition/threshold to IllConditioned.

Value inverse throws runtime error when exceptions are enabled, otherwise returns
zero matrix; `try_inverse` returns nullopt. Avoid these success-shaped fallbacks
in new checked application paths.

#### Hosted factorization APIs

| API | Result and algorithm |
|-----|----------------------|
| `lu()` | `(L,U,P)`; partial pivoting, `P*A=L*U` |
| `cholesky()` | Optional lower triangular `L`, `A=L*transpose(L)` |
| `qr()` | Square Householder `(Q,R)`, `A=Q*R` |
| `svd(maxIterations=100,tolerance=1e-9,converged=nullptr)` | Full `(U,S,V)`, `A=U*S*transpose(V)` |
| `pseudoinverse(tolerance=1e-9)` | `V*S_plus*transpose(U)` |
| `rank()` | Scaled pivoted elimination for floating input |

LU begins `L=I,U=A,P=I`; row swaps also swap already populated L columns.
Zero pivots skip elimination without an explicit failure result.
Reconstruct with `A=transpose(P)*L*U`, not `P*L*U` in general.

Cholesky rejects non-finite/zero input and asymmetry greater than
`100*epsilon` after normalization by maximum magnitude. Use the ordinary
lower-triangle recurrence; every diagonal residual must be finite and strictly
positive. Positive semidefinite input is rejected. Return nullopt on failure.

Householder QR scales input by maximum magnitude, uses hypot column norms and
stable signed reflector vectors, updates R and accumulated Q, then rescales R.
Zero/unusable scale leaves initial `(I,A)`. Public QR is square only, without
column pivoting or a success flag.

SVD uses one-sided Jacobi:

1. For wide input, transpose the problem and swap full factors on return.
2. Initialize `U=I,V=I,S=0`, convergence false. Invalid data/tolerance returns
   initial factors; zero input reports convergence.
3. Scale work by maximum magnitude.
4. Sweep column pairs. Compute squared norms `alpha,beta` and dot `gamma` in
   selected `hi_t<T>`. Skip when
   `abs(gamma)<=max(tolerance,4*epsilon)*sqrt(alpha*beta)`.
5. Otherwise apply stable Jacobi tangent/cosine/sine rotation to work and V.
   For `zeta=(beta-alpha)/(2*gamma)`, the small root can be expressed as
   `t=sign(zeta)/(abs(zeta)+hypot(1,zeta))`, with the zero case chosen
   consistently with the rotation equations.
6. A sweep with no needed rotations establishes convergence.
7. Compute column magnitudes with hypot; sort descending with matching V/work
   column swaps.
8. Retain singular directions above
   `epsilon*tall_row_count*largest_scaled_magnitude`; rescale singular values.
9. Normalize retained U columns, then complete full U from standard-basis vectors
   using two reorthogonalization passes, accepting norm above sqrt(epsilon).

Report nonconvergence for exhaustion or overflowing singular values; still return
approximate factors. Full factors require `O(R^2+C^2+R*C)` storage. Sweeps cost
`O(iterations*max(R,C)*min(R,C)^2)`; full basis completion can be substantially
more expensive than the thin factorization.

Pseudoinverse calls default SVD and applies the caller's tolerance only as an
**absolute singular-value cutoff**, not as the Jacobi tolerance. It does not
inspect convergence. Rank normalizes floating matrices and skips pivots at or
below `100*epsilon`; non-finite/zero input returns zero. Integral rank uses
truncating division, not fraction-free elimination.

**Next-version decision:** add an explicit convergence/error path before relying
on pseudoinverse for critical workloads. There is no existing dedicated `solve`,
triangular substitution, least-squares, reusable factor object, or eigenvector API.
Inverse-times-vector is composition, not a stable solver promise.

#### QR eigenvalues and allocation boundary

Square `eigenvalues_qr(out,out_size,maxIterations=100,tolerance=1e-6) -> bool`
is caller-buffer based. Reject null/undersized buffer, non-finite input, and
negative/non-finite tolerance.

Repeatedly factor current A by Householder QR and replace A with `R*Q`.
Convergence requires **every** strictly lower-triangular entry within the absolute
tolerance. Only on convergence write diagonal entries to the first N output slots.
Failure preserves output and results are not sorted.
Zero iterations fail for dimensions greater than one, even on diagonal input;
size one is already converged.

The allocating overload returns `variant<vector<T>,EigenError>` and is present
only with dynamic allocation enabled. It allocates N values and maps failures to
`NotConverged`; the declared `InvalidOutputBuffer` error is not emitted there.
This is unshifted real QR: no Hessenberg reduction, shifts, deflation, complex
eigenvalues, or eigenvectors. Complex conjugate pairs generally do not converge
to the required real diagonal.

**Embedded availability:** `MICROLA_EMBEDDED` excludes hosted optional/variant
inverse conveniences, pseudoinverse, LU, Cholesky, QR, SVD, and rank. Boolean/value
inversion and the caller-buffer eigenvalue API remain available. Merely disabling
exceptions does not exclude all hosted decompositions.

### 4.4 Quaternions

Evidence: [quaternion implementation](../include/microla/quaternion.hpp).

`Quaternion<T=float>` owns aligned public `data[4]` in **x,y,z,w** order,
default identity `[0,0,0,1]`. Four-scalar construction takes **w,x,y,z**;
pointer construction copies stored **x,y,z,w**. Preserve this distinction in
interop tests.

Provide axis-angle and 3x3 matrix constructors, identity, `from_axis_angle`,
`from_euler(vector)`/`from_euler(roll,pitch,yaw)`, `from_two_vectors`,
mutable/const component references, indexing, debug-asserting `at`, `ptr`,
nonstatic `size`, `vec`, and `set_vec`. Axis-angle expects unit axis and does not
normalize it; matrix conversion uses trace/largest-diagonal branches without
validating or normalizing the supplied matrix.

Arithmetic includes component addition/subtraction/negation, scalar right and
left multiplication, division, compound scalar arithmetic, Hamilton multiplication
and compound quaternion multiplication, plus vector rotation through `operator*`.

```text
(w1,v1)*(w2,v2)
  = (w1*w2-dot(v1,v2), w1*v2+w2*v1+cross(v1,v2))
```

`rotate(v)` uses `v+2*w*cross(xyz,v)+2*cross(xyz,cross(xyz,v))`;
`rotate_inverse` reverses the first cross term. Both assume unit quaternion,
as does `to_matrix`. They do not divide by squared norm.

Provide `norm`, `norm_squared`, `dot`, `normalized`, in-place `normalize`,
`conjugate`, `inverse_unit`, general `inverse`, `angle_to`, equality,
`approx_equal`, `is_identity`, `to_matrix`, `lerp`, `slerp`, `to_euler`,
and scalar roll/pitch/yaw. Alias is `Quatf`; do not assume `Quatd` is provided.

Norm and normalization use the same finite scale-safe strategy as vectors.
Zero/non-finite quaternion normalization returns input unchanged.
Unit inverse is simply conjugate. General scalar inverse divides conjugate by
raw squared norm; zero yields NaNs after a debug assertion.

**Next-version decision:** scalar `/0` returns a zero quaternion while `/=0`
performs unchecked reciprocal multiplication; zero inverse can also differ in
accelerated branches. Preserve this as documented legacy behavior until choosing
a uniform checked/failure policy. General inverse is not as scale-safe as
normalization.

Equality/approximate equality compare components with absolute epsilon;
identity requires w near positive one and xyz near zero. Thus q and -q are
different representations even though they encode the same rotation.
`angle_to` normalizes inputs, returns NaN for zero quaternion, and computes:

```text
delta = conjugate(normalize(left))*normalize(right)
angle = 2*atan2(hypot(delta.x,delta.y,delta.z),abs(delta.w))
```

This gives shortest rotation distance in `[0,pi]`, invariant to sign and finite
scaling.

From-two-vectors normalizes inputs: dot at least `1-1e-6` gives identity; dot at
most `-1+1e-6` uses `unit_x cross from`, falling back to `unit_y cross from`
when squared axis norm is below `1e-6`, then rotates pi. Otherwise
`s=sqrt(2*(1+dot))`, quaternion `(w=s/2,xyz=cross(from,to)/s)`.
Zero/non-finite input has no dedicated rejection.

Lerp is plain component interpolation, without normalization/sign adjustment.
SLERP flips the second quaternion on negative dot, uses normalized lerp when
adjusted dot is at least `0.9995`, otherwise acos/sine interpolation with dot
clamped to `[-1,1]`. General branch assumes unit inputs; `t` is not clamped.

Euler construction uses half-angle sines/cosines:

```text
w = cr*cp*cy + sr*sp*sy
x = sr*cp*cy - cr*sp*sy
y = cr*sp*cy + sr*cp*sy
z = cr*cp*sy - sr*sp*cy
```

Extraction is roll `atan2(2*(w*x+y*z),1-2*(x*x+y*y))`,
pitch `asin(2*(w*y-z*x))` with saturation to signed half-pi at absolute input
at least one, and yaw `atan2(2*(w*z+x*y),1-2*(y*y+z*z))`.
`ROLL=0,PITCH=1,YAW=2`; gimbal-lock output need not yield a unique roll/yaw pair.

### 4.5 Borrowed matrix views

`MatrixView<T,R,C>` and `ConstMatrixView<T,R,C>` take a base pointer,
`parent_cols`, `start_row`, and `start_col`. Indexing is:

```text
data[(start_row + row)*parent_cols + start_col + col]
```

Provide `operator()(row,col)`, `to_matrix()`, `rows()`, and `cols()`.
Mutable views additionally provide `set(const Mat<T,R,C>&)` and `fill(value)`.
Descriptor copy/assignment copies the view, not its elements; `set` performs the
element copy.

Storage must outlive the view and cover its full extent. Constructors do not
validate matrix extent or pointer validity. Debug indexing checks only local
row/column bounds. Float bulk operations may use contiguous-row SIMD helpers;
other scalars use nested loops. Views do not add alignment or ownership.

### 4.6 Borrowed vector views

`VectorView<T,N>` and `ConstVectorView<T,N>` take pointer, `parent_size`,
`start_index`, and optional `stride=1`. Element `i` is:

```text
data[start_index + i*stride]
```

Provide indexing, `to_vec()`, `size()`, and `stride()`. Mutable views add `set`
and `fill`. Debug constructors assert positive size and that first/last accessed
indices are in the parent; debug indexing checks `i<N`. Release builds rely on
caller preconditions. Unsigned offset arithmetic is not overflow-checked.

Stride is in elements, not bytes. Stride zero is not explicitly prohibited and
repeatedly refers to one element. Mutable contiguous float operations can use
SIMD; strided operations, other scalars, and const-view extraction use scalar loops.
Overlapping `set` source/destination is not a guaranteed memmove operation.

### 4.7 Geometry

Geometry lives in `microla::geometry`, outside the umbrella include.
`Ray<T=float>`, `Plane<T=float>`, `AABB<T=float>`, and `Sphere<T=float>` have
defaults; `Triangle<T>` and `Frustum<T>` require an explicit scalar. Provide
float/double aliases with `f`/`d` suffixes.

#### Rays and planes

`Ray` has public origin/direction, default `(0,0,0)` and `(0,0,1)`,
constructor `(origin,direction)`, and `at(t)=origin+direction*t`.
Do not normalize ray directions implicitly. Parameter `t` is physical distance
only for a unit direction.

`Plane` has public normal and `d`, equation `dot(normal,p)+d=0`,
default normal `+Y` and `d=0`. Provide:

- Constructor `(normal,d)`.
- `from_point_normal(point,normal)` and `from_points(p0,p1,p2)`.
- `distance(point)`, `signed_distance(point)`, and `project(point)`.
- `intersect(ray) -> optional<T>`.

The inspected constructor normalizes the normal but leaves supplied `d`
unchanged. Point-based construction computes `d=-dot(normal,point)`;
three-point winding determines normal direction. Signed distance uses compensated
summation; projection uses the selected higher-precision intermediate and assumes
unit normal. Ray intersection rejects `abs(dot(n,ray.direction)) < epsilon`,
solves `t=-(dot(n,origin)+d)/dot(n,direction)`, and accepts `t>=0`.

**Next-version decision:** normalizing arbitrary plane equation coefficients must
scale both normal and `d`; changing the historical constructor requires an explicit
compatibility decision. Zero/non-finite normals are not robustly rejected by all
existing constructors.

#### Axis-aligned boxes

`AABB` has public `min,max`, default zero box, corner construction,
`from_center_extents(center,half_extents)`, `center`, `extents`, `size`,
point/box `contains`, box `intersects`, `merge`, `expand(point)` mutation,
`expand(amount)` value return, `surface_area`, and `volume`.

Containment/overlap include boundaries; touching counts as intersection.
Corners are not reordered, and negative expansion can invert bounds.
Member `intersect(ray)` returns `optional<pair<T,T>>` by slab clipping from
`[0,+infinity]`. A direction component smaller than epsilon in magnitude is
parallel; reject if the origin is outside that slab. A ray starting inside
has entry zero.

#### Spheres

`Sphere` has public center/radius, default center zero/radius one; constructor
`(center,radius)`, point/sphere `contains`, sphere/AABB `intersects`, `to_aabb`,
`surface_area`, `volume`, and interval-valued member `intersect(ray)`.
Containment and overlap include boundaries. Sphere/AABB overlap clamps the center
to the box and compares squared distance with squared radius.

Ray roots solve the full quadratic for a possibly non-unit direction.
Reject negative discriminant, non-finite roots, or an exit behind the origin.
Return `(max(0,entry),exit)`. Radius validity is not checked, and the direct
quadratic formula is not cancellation-resistant.

#### Triangles

`Triangle` has public `v0,v1,v2`, zero-vertex default, three-point construction,
`normal`, `area`, `centroid`, `to_aabb`, `contains(point)`, and
`intersect(ray) -> optional<T>`.

Normal is the normalized edge cross product; area is half its length.
Containment uses barycentric coordinates and includes edges, but assumes a
coplanar point and does not independently guard a degenerate denominator.

Ray intersection is two-sided Moller-Trumbore with no backface culling. Reject
non-finite determinant or:

```text
abs(det) <= epsilon * length(edge1)*length(edge2)*length(direction)
```

Accept `u>=0`, `v>=0`, `u+v<=1`, and finite `t>=0`. This scale-relative gate is
important for equivalent triangles/rays expressed at different scales.

#### Free intersection functions

Provide free `intersect(ray,sphere/plane/aabb) -> optional<T>`,
`intersect(ray,triangle) -> optional<tuple<T,T,T>>` containing `(t,u,v)`,
and `intersects(sphere,aabb) -> bool`.

Compatibility differences are intentional facts to track:

- Free sphere intersection selects the first nonnegative surface root; an inside
  ray normally returns the exit rather than the member interval's zero entry.
- Free AABB intersection returns entry only and lacks the member's explicit
  near-zero parallel-direction branch.
- Free triangle returns barycentrics; vertex weights are `(1-u-v,u,v)`.

**Next-version decision:** unify shared intersection kernels where practical,
while preserving explicitly different result semantics. Do not assume that
member/free overloads currently have identical degenerate-input handling.

#### Frusta

`Frustum<T>` owns six planes ordered `LEFT,RIGHT,BOTTOM,TOP,NEAR,FAR`.
`from_matrix(vp)` extracts row 3 plus/minus rows 0, 1, and 2, normalizing both
normal and `d`. This implements symmetric clip bounds `-w<=x,y,z<=w`;
it is not a zero-to-one depth frustum.

Point containment requires all signed distances nonnegative; sphere containment
requires distances at least radius, sphere intersection at least negative radius.
AABB intersection checks the positive vertex against each inward plane and is
conservative. There is no extraction guard for degenerate/non-finite planes;
NaN comparisons can evade rejection-style predicates. A replacement must specify
whether invalid geometry remains caller misuse or receives a checked API.

## 5. Generic state estimation

Evidence: [linear filter](../include/microla/kalman.hpp) and
[nonlinear filter](../include/microla/extended_kalman.hpp).

### 5.1 Shared dimensions and state

`KalmanFilter<T=float, StateDim=2, MeasDim=1>` and
`ExtendedKalmanFilter<T=float, StateDim=2, MeasDim=1>` use:

| Alias | Shape |
|-------|-------|
| `StateVec` | `Vec<T,StateDim>` |
| `MeasVec` | `Vec<T,MeasDim>` |
| `StateMat` | `Mat<T,StateDim,StateDim>` |
| `MeasMat` | `Mat<T,MeasDim,MeasDim>` |
| `MeasToStateMat` | `Mat<T,MeasDim,StateDim>`; measurement Jacobian `H` |
| `StateToMeasMat` | `Mat<T,StateDim,MeasDim>`; gain `K` |

The last two alias names are historical; their shapes, not their names, determine
the direction of a linear mapping.

Both filters start with `x=0`, `P=I`, `Q=0.01*I`, `R=0.1*I`.
Custom-noise constructors replace `Q,R`. Reset replaces only `x,P`; it retains
models, callbacks, and noise settings.

Accessors return const references to state/covariance, or selected state,
variance, and standard deviation values. Setters directly store supplied values;
they do not independently validate symmetry or positive definiteness. A negative
variance yields NaN from standard-deviation extraction.

### 5.2 Linear filter

Persistent storage is `x,P,F,H,Q,R`. Default `F=I`, default `H=0`; callers must
set their actual measurement model.

Public operation families:

- Default/custom-noise construction.
- `predict()` and `predict<CtrlDim>(u,B)`.
- `update(z)` and `update_simple(z)`, returning `bool`.
- `get_state()`, `get_state(index)`, `get_covariance()`,
  `get_variance(index)`, `get_std_dev(index)`.
- State, covariance, transition, measurement, and noise setters.
- `get_f()`, `get_h()`, `get_q()`, `get_r()`.
- `reset()` and `reset(x0,P0)`.
- `compute_innovation(z)`, `compute_innovation_covariance()`, `compute_nis(z)`.

Prediction:

```text
x_next = F*x                       [or F*x + B*u]
P_next = F*P*transpose(F) + Q
```

Validate finite inputs, compute candidates, then commit **both** state and
covariance only if both are finite. `predict` returns `void`; invalid prediction
is a no-op in the inspected API.

Measurement update:

```text
y = z - H*x
S = H*P*transpose(H) + R
K = P*transpose(H)*inverse_checked(S)
x_next = x + K*y
A = I - K*H
P_joseph = A*P*transpose(A) + K*R*transpose(K)
P_next = (P_joseph + transpose(P_joseph))/2
```

The ordinary update uses Joseph covariance and explicit symmetrization.
`update_simple` instead uses `P_next=(I-K*H)*P`, without the Joseph calculation
or explicit symmetrization.

Return `false` and preserve both estimates on non-finite input, failed checked
inversion, or non-finite candidates. A small absolute determinant alone is not a
sufficient reason to reject a well-conditioned, differently scaled `S`.

NIS is `transpose(y)*inverse(S)*y`. Return infinity for invalid data,
failed inversion, non-finite NIS, or negative NIS. NIS computation does **not**
automatically gate `update`; the caller owns outlier policy.

Aliases are `KalmanFilter1D<T>` = `(1,1)`, `KalmanFilter2D<T>` = `(2,1)`,
`KalmanFilter4D<T>` = `(4,2)`, and `KalmanFilter6D<T>` = `(6,3)`.
Names do not install a constant-velocity transition model.

### 5.3 Extended Kalman filter

Persistent state is `x,P,Q,R` plus plain function pointers:

```text
StateTransitionFunc:    StateVec(const StateVec&, T dt)
MeasurementFunc:        MeasVec(const StateVec&)
StateJacobianFunc:      StateMat(const StateVec&, T dt)
MeasurementJacobianFunc: MeasToStateMat(const StateVec&)
ResidualFunc:           MeasVec(const MeasVec& measured,
                               const MeasVec& predicted)
```

No `std::function`, closure allocation, numerical differentiation, or automatic
control-vector injection is required. Default callbacks are null.

Prediction evaluates both `f(x,dt)` and `F_jacobian(x,dt)` at the **prior** state,
then uses the same covariance formula as the linear filter. A finite negative
`dt` is not separately rejected by this generic API; the nonlinear model owns
its time semantics. Missing callbacks or invalid candidates make prediction a
transactional no-op.

Update evaluates `z_pred=h(x)` and `H=H_jacobian(x)`. Innovation is
`residual(z,z_pred)` if configured, otherwise `z-z_pred`. This allows wrapped
angular residuals without changing the filter.

- `update(z)` uses the simplified covariance formula.
- `update_joseph(z)` uses Joseph covariance but does not add the linear filter's
  final explicit symmetrization.
- Both return `false` without committing state/covariance on failure.
- Callback/model/noise setters, indexed getters, reset, innovation, and NIS
  parallel the linear filter.
- `compute_innovation` returns the zero measurement vector when `h` is null.
- `compute_nis` returns infinity when required callbacks or numerical checks fail.

Aliases: `EKF2D<T>` = `(2,2)`, `EKF6D<T>` = `(6,3)`,
`EKFAttitude<T>` = `(7,6)`. The attitude alias is only a dimension choice;
it is not the multiplicative quaternion estimator described next.

### 5.4 Resource and failure policy

Dense prediction costs approximately `O(StateDim^3)`. Measurement updates add
`O(MeasDim^3)` inversion and the corresponding rectangular products. Storage is
fixed `O(StateDim^2 + StateDim*MeasDim + MeasDim^2)`, with automatic scratch values.

**Next-version decision:** preserve the legacy `void` prediction interface for
compatibility, but a new explicit status-returning API is preferable to inferring
success from unchanged estimates. Do not silently present a rejected prediction
as a successful state advance.

## 6. Sensor-fusion data model and configuration

Evidence: [sensor-fusion implementation](../include/microla/sensor_fusion.hpp).

### 6.1 Public enumerations

| Enum | Values, in declaration order |
|------|------------------------------|
| `SensorModel` | `accel_only`, `imu6`, `imu9` |
| `FusionBackend` | `mahony`, `mekf` |
| `PrimaryScalarOutput` | `shortest_3d_angle`, `tilt_angle`, `heading_delta`, `hinge_twist`, `swing_angle`, `roll_delta`, `pitch_delta`, `yaw_delta` |
| `Observability` | `none`, `tilt_only`, `heading_with_drift`, `full_3d` |
| `SolutionQuality` | `invalid`, `degraded`, `usable`, `nominal` |

`StatusFlags` is `uint32_t`. Provide `flag_mask`, flag/mask union operators,
`has_flag`, and `set_flag`. `none=0`; remaining bits are:

| Bit | `StatusFlag` |
|-----|--------------|
| 0 | `sample_time_invalid` |
| 1 | `gyro_rejected` |
| 2 | `accel_rejected` |
| 3 | `mag_rejected` |
| 4 | `propagation_only` |
| 5 | `startup_not_initialized` |
| 6 | `left_sample_stale` |
| 7 | `right_sample_stale` |
| 8 | `pair_time_skew_exceeded` |
| 9 | `heading_unobservable` |
| 10 | `drift_exceeds_nominal` |
| 11 | `output_not_supported` |
| 12 | `calibration_invalid` |
| 13 | `configuration_invalid` |
| 14 | `freefall_detected` |
| 15 | `high_linear_acceleration` |
| 16 | `high_rotation_rate` |
| 17 | `orientation_held` |

Flags describe the latest operation; they are not a permanent history of faults.
`propagation_only` is historically also emitted by gravity-only recovery when
there is no accepted absolute heading correction.

### 6.2 Samples, calibration, and results

All sample templates use the same scalar `T` for data and timestamp:

- `AccelSample<T>{timestamp_s, accel}`.
- `Imu6Sample<T>{timestamp_s, gyro, accel}`.
- `Imu9Sample<T>{timestamp_s, gyro, accel, mag}`.

`SensorCalibration<T>` contains:

```text
sensor_to_body : Quaternion<T> = identity
accel_bias    : Vec<T,3> = zero
accel_scale   : Vec<T,3> = one
gyro_bias     : Vec<T,3> = zero
gyro_scale    : Vec<T,3> = one
mag_bias      : Vec<T,3> = zero
mag_soft_iron : Mat<T,3,3> = identity
```

Calibration precedes adaptive gyro-bias subtraction:

```text
accel_body = rotate(sensor_to_body, (accel_raw-accel_bias) .* accel_scale)
gyro_body  = rotate(sensor_to_body, (gyro_raw-gyro_bias) .* gyro_scale)
mag_body   = rotate(sensor_to_body, mag_soft_iron*(mag_raw-mag_bias))
```

`.*` means componentwise multiplication. Cache identity/zero calibration flags to
skip redundant work without changing the result.

Calibration validation requires finite values, mounting quaternion squared norm
greater than machine epsilon, each scale's absolute value greater than epsilon,
and `abs(det(mag_soft_iron)) > epsilon`. Normalize the mounting quaternion.
Negative nonzero scales are allowed. The soft-iron check is the inspected absolute
determinant gate, not a full condition-number or physical-validity test.

`OrientationEstimate<T>` carries orientation, confidence, drift, state timestamp,
observability, quality, flags, validity, `heading_referenced`, and
`tilt_confidence`. Initial orientation is identity; numeric metadata is zero;
observability is none, quality invalid, and validity false.

`ScalarAngleResult<T>` carries mode, angle, confidence, drift, observability,
quality, flags, and validity. `RelativeAngleResult<T>` contains both entity
snapshots, `q_left_to_right`, the primary scalar result, relative Euler angles,
shortest 3D angle, tilt, heading delta, hinge twist, swing angle, pair skew,
reference-active flag, and overall validity. Secondary numeric fields do not
independently guarantee observability.

### 6.3 Orientation policy defaults

Policies are types with static members, not runtime option dictionaries.
`OrientationConfigBase<T>` requires floating-point `T`.

| Member | Default |
|--------|---------|
| `sensor_model`, `backend` | `imu6`, `mahony` |
| `estimate_gyro_bias` | true |
| `enable_accel_correction`, `enable_mag_correction` | true, true |
| `min_dt_s`, `max_dt_s`, `max_sample_age_s` | `1e-6`, `0.2`, `0.05` |
| `expected_gravity_norm` | `9.80665` |
| `accel_norm_min`, `accel_norm_max` | `8.0`, `11.5` |
| `gyro_norm_max` | `35.0` |
| `expected_mag_norm`, `mag_norm_min`, `mag_norm_max` | `50.0`, `20.0`, `70.0` |
| `enforce_mag_norm_limits` | false |
| `mag_relative_norm_tolerance` | `0.35` |
| `mag_reference_learning_alpha` | `0.05` |
| `mag_alignment_max_error_rad` | `2.35619449019` |
| `gravity_filter_alpha` | `0.2` |
| `held_orientation_confidence` | `0.0` |
| `stationary_gyro_norm_max` | `0.05` |
| `stationary_accel_error_max` | `0.15` |
| `kp_accel`, `kp_mag`, `ki_gyro_bias` | `2.5`, `1.5`, `0.05` |
| `drift_rate_without_heading_rad_per_s` | `0.02` |
| `drift_confidence_limit_rad` | `0.35` |
| `q_attitude`, `q_gyro_bias` | `1e-3`, `1e-5` |
| `r_accel`, `r_mag` | `0.03`, `0.05` |
| `accel_nis_gate`, `mag_nis_gate` | `7.81`, `7.81` |
| `body_down_axis()`, `world_gravity_direction()` | `(0,0,-1)` |
| `body_heading_axis()`, `world_magnetic_reference()` | `(1,0,0)` |

Provide these derived starting policies:

| Policy | Overrides |
|--------|-----------|
| `DefaultAccelOnlyConfig<T>` | accel-only, Mahony, no bias adaptation, no magnetic correction |
| `DefaultImu6MahonyConfig<T>` | IMU6, Mahony, no magnetic correction |
| `DefaultImu6EkfConfig<T>` | IMU6, MEKF, no magnetic correction |
| `DefaultImu9MahonyConfig<T>` | IMU9, Mahony |
| `DefaultImu9EkfConfig<T>` | IMU9, MEKF |
| `HighDynamicsAccelOnlyConfig<T>` | accel-only, no bias/mag, accel range `[2,25]`, gravity alpha `0.05`, held confidence `0.25` |
| `HighDynamicsImu6MahonyConfig<T>` | IMU6, no mag, accel range `[2,40]`, gyro limit `150`, gravity alpha `0.05`, stationary gyro limit `0.1` |
| `HighDynamicsImu9MahonyConfig<T>` | IMU9, same high-dynamics numeric overrides |

Accel-only plus MEKF is a compile-time error. Validate applicable policy values
once per orientation-estimator construction: usable axes, finite scalars,
positive required bounds/noise, ordered min/max intervals, nonnegative gains and
ages, coefficients/confidence in `[0,1]`. IMU9 magnetic reference must have a
horizontal component relative to gravity. Invalid policy values cause flagged
sample rejection rather than an apparently usable estimate.

### 6.4 Relative policy defaults

`RelativeAngleConfigBase<T>` and `DefaultRelativeAngleConfig<T>` provide:

| Member | Default |
|--------|---------|
| `default_output` | `shortest_3d_angle` |
| `allow_runtime_output_override` | true |
| `allow_drift_with_quality_flags` | true |
| `apply_reference_pose` | false |
| `max_pair_skew_s` | `0.03` |
| `max_alignment_horizon_s` | `0.05` |
| `min_confidence_to_publish` | `0.35` |
| `nominal_drift_limit_rad` | `0.0872664626` |
| `hinge_axis_left()`, `hinge_axis_right()` | `(0,0,1)` |

Validate relative bounds and the axes required by the selected mode. Require
nonnegative finite timing limits, confidence floor in `[0,1]`, and positive
finite drift limit.

## 7. Per-entity orientation state machine

### 7.1 Public surface

`OrientationEstimator<T,Config>` supplies:

- Default construction or explicit construction from `SensorCalibration<T>`.
- `using Sample`, resolved from `Config::sensor_model`.
- `update(const Sample&) -> void`.
- `reset() -> void`.
- `rebase_timestamp(T new_processed_time) -> bool`.
- `set_calibration(const SensorCalibration<T>&) -> void`.
- `estimate() -> const OrientationEstimate<T>&`.
- `orientation() -> Quaternion<T>`.
- `gyro_bias_estimate() -> const Vec<T,3>&`.
- `predict_orientation(T target_time,T max_horizon) -> Quaternion<T>`.
- `error_covariance() -> const Mat<T,6,6>&`, constrained to MEKF IMU policies.

Persistent state includes calibration, orientation, adaptive bias, filtered
acceleration, `6x6` covariance, diagnostics, initialization/timestamp/heading
flags, calibration fast-path flags, processed and state timestamps, drift,
measurement confidences, learned magnetic norm, and latest prediction rate.

**Implementation:** Mahony and MEKF currently both retain the covariance member.
Do not claim that choosing Mahony reduces persistent object size. Constructors
assert that the complete orientation-estimator type is smaller than 4096 bytes;
this is not a whole-call-stack budget.

### 7.2 Timing and transactional rejection

Maintain two clocks:

1. **Processed sample time:** last accepted-in-order sample examined.
2. **State time:** last sample that actually advanced attitude.

For each update, clear latest diagnostics but begin with the stored attitude.

| Condition | Required transition |
|-----------|---------------------|
| Invalid policy or calibration | Reject, preserve state/clocks, flag reason |
| Non-finite timestamp | Reject, preserve state/clocks, flag time |
| First finite sample | `dt=0`; try measurement initialization |
| `dt<=0` or `dt<min_dt_s` | Reject before propagation/correction; preserve clocks/state |
| `dt>max_dt_s` | Flag time, set propagation interval to zero, advance processed clock, allow measurement-only recovery |
| Normal interval | Advance processed clock; execute selected backend |
| Backend progresses state | Set state time to processed time |
| No state progress after initialization | Hold attitude, flag held, disable prediction, retain old state time |

Rejection must not change attitude, adaptive bias, covariance, or learned field
reference. Diagnostics may change to invalid. During complete dropout, recovery
integrates only the newest processed interval, not the accumulated held-state age.

`reset` restores identity attitude, zero adaptive bias/drift, initial gravity,
`P=0.1*I`, empty diagnostics, no initialization, no timestamps, no learned magnetic
reference, and no prediction rate. Calibration and policy validity are retained.

`set_calibration` validates/caches the new calibration and then resets state.
Even assigning identity calibration is a reset operation.

`rebase_timestamp(t)` returns false unchanged for non-finite `t`; otherwise set
processed time to `t` and state time to
`t-(old_processed_time-old_state_time)` when a clock existed. This preserves the
age of a held attitude along with orientation, bias, covariance, drift, and learned
magnetic state.

`timestamp_needs_reset<T>(t)` tests `t >= 16777.216` seconds when
`sizeof(T)==sizeof(float)`; otherwise it returns false. This is a millisecond
resolution advisory, not automatic timestamp wrapping. Prefer a local origin or
double timestamps for long-running acquisition.

### 7.3 Measurement gates and initialization

- Accel: finite components and inclusive squared-norm bounds.
- Gyro: finite components and inclusive squared-norm upper limit.
- Low accel norm: `accel_rejected | freefall_detected`.
- High accel norm: `accel_rejected | high_linear_acceleration`.
- Non-finite accel: `accel_rejected` without a magnitude subreason.
- Invalid gyro: `gyro_rejected`; finite excessive norm additionally sets
  `high_rotation_rate`.
- Valid gyro skipped at startup or across a large gap is **not** gyro rejection.

Initialization requires valid acceleration. Align normalized measured acceleration
to the configured world gravity using a minimum rotation quaternion.
For IMU9, an accepted magnetic measurement additionally aligns horizontal heading
to the configured world magnetic reference. Otherwise leave heading unreferenced.
Initialize filtered acceleration to the accepted sample.

Use scale-safe direction normalization: divide by the largest absolute component
before norm calculation. Direction-only gates must remain usable for very small
or large finite magnetic units. A zero/non-finite direction is unusable; a magnetic
vector parallel to gravity cannot constrain heading.

### 7.4 Magnetic acceptance and learning

Magnetic correction is available only for enabled IMU9 policies and finite,
nonzero vectors. Measure strength with stable three-component hypot.

Strength confidence:

1. Reject nonpositive/non-finite norm.
2. If absolute limits are enabled, reject outside the declared range.
3. With absolute limits but no learned norm, score against the configured expected
   norm using the norm-confidence formula below.
4. With a learned norm `m0`, define tolerance
   `tau=max(mag_relative_norm_tolerance,0.05)` and
   deviation `d=abs(m-m0)/m0`. Reject if `d>tau`; otherwise confidence
   is `clamp(1-d/tau,0,1)`.
5. Without a learned norm or enforced absolute limits, bootstrap confidence is one.

Directional confidence is one until initialized heading is currently referenced.
Otherwise compare normalized measured magnetic direction with
`inverse_rotate(q,world_mag)`. If the configured angular gate is at most epsilon,
directional gating is disabled. Else reject angle above the gate and score
`clamp(1-angle/gate,0,1)` within it.

Require a horizontal magnetic component relative to accepted acceleration or,
when acceleration is rejected, attitude-predicted gravity. The normalized
cross-product squared length must exceed machine epsilon. Combined magnetic
confidence is strength times directional confidence; acceptance requires it
strictly greater than zero.

Learn the first accepted norm directly; later apply
`m0=(1-alpha)*m0+alpha*m`, alpha clamped to `[0,1]`. MEKF learning occurs only
after its vector update passes NIS and candidate-state checks. Rejected updates
must not poison this reference.

Without an absolute bootstrap limit, the first usable magnetic sample cannot be
classified as disturbed solely from its strength. This is an observability limit,
not an excuse to hard-code magnetic units.

### 7.5 Accelerometer-only backend

For accepted acceleration:

```text
a_filtered = a                  [first accepted sample]
a_filtered = (1-alpha)*a_filtered + alpha*a  [later]
q = minimum_rotation(normalize(a_filtered), normalize(world_gravity))
```

Heading is never referenced and drift is zero. On rejection, freeze orientation
and retain state time. Before initialization, publish invalid with the startup
flag. After initialization, `held_orientation_confidence` may permit a degraded
held result; the default zero yields invalid. Held confidence is itself multiplied
by the entity's `0.95` factor. Never infer fresh tilt from a frozen timestamp.

Extended freefall and arbitrary linear acceleration cannot be corrected by policy
tuning alone. No accelerometer-only algorithm can recover absolute yaw.

### 7.6 Mahony-style backend

On accepted propagation interval and valid gyro:

```text
g_expected = inverse_rotate(q, normalize(world_gravity))
g_measured = normalize(filtered_accel)
e_accel = cross(g_measured,g_expected)*kp_accel*c_accel

m_expected = inverse_rotate(q, normalize(world_mag))
m_measured = normalize(mag)
e_mag = cross(m_measured,m_expected)*kp_mag*c_mag

e = enabled accepted correction terms
bias_next = bias - e*ki_gyro_bias*dt     [only if stationary and enabled]
omega = gyro - bias_next + e
q_next = normalize(q * axis_angle_increment(omega*dt))
```

Stationary means gravity-norm error at most
`stationary_accel_error_max` and gyro norm at most
`stationary_gyro_norm_max`. Cache corrected `omega` for bounded forward prediction.

When propagation is unavailable, require enabled valid acceleration for recovery.
Apply a minimum gravity-alignment correction to the **existing** attitude, not a
new zero-yaw attitude. If magnetic correction is valid, a gravity-plus-heading
target may be constructed. Blend current attitude toward target with SLERP:

```text
blend = max(0.2, max(c_accel,0.25)*0.6)
blend = max(blend, max(c_mag,0.25)*0.6)   [accepted magnetic recovery]
```

Clamp blend to `[0,1]`. With neither propagation nor correction, hold invalid
attitude and disable prediction. Disabling acceleration correction also disables
this measurement-only gravity recovery.

### 7.7 Multiplicative EKF backend

Nominal state is quaternion attitude plus body gyro bias. Error-state ordering is
`[delta_theta_x,y,z, delta_bias_x,y,z]`. Covariance is `6x6`, initially `0.1*I`.
This is a dedicated MEKF, not the generic additive `EKFAttitude` alias.

Propagation:

```text
omega = gyro - bias
q_candidate = normalize(q * increment(omega*dt))
F = I6
F_attitude = I3 - skew(omega)*dt
F_attitude,bias = -I3*dt                [bias estimation enabled]
Q = diag(q_attitude*dt [three times],
         q_gyro_bias*dt [three times])  [bias noise zero when disabled]
P_candidate = F*P*transpose(F) + Q
```

Commit finite propagation candidates. Accepted gravity and magnetic corrections
then run sequentially, including when gyro propagation was unavailable. Do not
replace MEKF recovery with Mahony blending or bypass its innovation gates.

For each normalized vector measurement `z` with world reference `r`:

```text
z_pred = inverse_rotate(q,r)
y = z - z_pred
H = [skew(z_pred) | zero3]
R = variance*I3
S = H*P*transpose(H) + R
S_inv = inverse_checked(S)
NIS = dot(y,S_inv*y)
K = P*transpose(H)*S_inv
delta = K*y
q_candidate = normalize(q * increment(delta_theta))
bias_candidate = bias + delta_bias
A = I6 - K*H
P_joseph = A*P*transpose(A) + K*R*transpose(K)
```

Reject NIS if non-finite, negative, or greater than the selected gate. When bias
estimation is disabled, zero the three bias rows of `K`, omit propagation
attitude/bias coupling, and omit bias process-noise growth.

Apply the right-multiplicative reset Jacobian to **all** covariance blocks:

```text
D = skew(delta_theta)
theta2 = dot(delta_theta,delta_theta)
J = I3 - a*D + b*D*D
G = block_diag(J,I3)
P_candidate = G*P_joseph*transpose(G)
```

For `theta2 < 1e-4`:

```text
a = 1/2 - theta2/24 + theta2*theta2/720
b = 1/6 - theta2/120 + theta2*theta2/5040
```

Otherwise, with `theta=sqrt(theta2)`:

```text
a = (1-cos(theta))/theta2
b = (theta-sin(theta))/(theta2*theta)
```

Reject non-finite delta, bias, quaternion, or covariance candidates and negative
covariance diagonals. Symmetrize each off-diagonal pair. Commit orientation, bias,
and covariance together only after all checks. Updating only the attitude block
of covariance is incorrect because cross-covariance also changes frame.

### 7.8 Drift, confidence, and validity

When an absolute heading correction succeeds, mark heading referenced and reduce
drift by `drift_rate*dt`, floored at zero. Otherwise mark heading unreferenced,
grow drift by `drift_rate*dt`, and set `propagation_only`. Accel-only always has
zero drift and tilt-only observability.

Norm confidence for accepted `[lo,hi]`, expected `e`, and measured norm `v`:

```text
span = max(abs(hi-e),abs(e-lo))
c = 1                                  [span <= epsilon]
c = clamp(1-abs(v-e)/span,0,1)          [otherwise]
```

Outside the range confidence is zero.

Entity confidence begins at `0.95`:

- Accel-only: multiply by accel confidence.
- IMU: multiply by `max(accel_confidence,0.5)`.
- IMU6: additionally multiply by `0.8`.
- IMU9 with referenced heading: multiply by `max(mag_confidence,0.5)`.
- IMU9 without referenced heading: multiply by `0.75`.
- If drift is positive, multiply by
  `clamp(1-drift/drift_confidence_limit_rad,0,1)`.

Tilt confidence is independently `0.95*accel_confidence`. Failed state progression
forces confidence and tilt confidence to zero except for explicitly enabled
accel-only held publication.

Quality mapping:

| Condition, evaluated in order | Quality |
|------------------------------|---------|
| Confidence `<0.15` | invalid |
| Confidence `<0.5` **or any flag is set** | degraded |
| Confidence `<0.85` | usable |
| Otherwise | nominal |

Entity validity requires initialization and non-invalid quality. Observability
is none before initialization, tilt-only for accel-only, heading-with-drift for
IMU6, and full-3D for IMU9 only while heading is currently referenced.
Confidence and drift are policy heuristics, not calibrated posterior probabilities
or statistical error bounds.

## 8. Relative-angle estimator

### 8.1 API and ownership

`RelativeAngleEstimator<T,LeftConfig,RightConfig,
RelativeConfig=DefaultRelativeAngleConfig<T>>` owns two orientation estimators,
one reference quaternion, and a reference-valid flag.

Provide `LeftEstimator`, `RightEstimator`, `LeftSample`, `RightSample` aliases;
`update_left`, `update_right`, `reset`, `set_left_calibration`,
`set_right_calibration`, `rebase_left_timestamp`, `rebase_right_timestamp`,
`capture_reference_pose`, `clear_reference_pose`, `compute`,
`compute_scalar(mode)`, and const entity-estimator accessors.

`compute_scalar` is unavailable by compile-time assertion when runtime overrides
are disabled. If drift is forbidden and the default mode requires heading, both
policies must have absolute-heading capability at compile time.

Changing either calibration clears the reference pose. Reset resets both entities
and clears the reference. Failed reference capture preserves any previous reference.

### 8.2 Alignment and reference transform

Computation is const and does not consume samples:

1. Snapshot both entity diagnostics.
2. For heading-dependent modes, require both entity estimates valid. For tilt,
   require positive tilt confidence on both sides, even if yaw confidence expired.
3. Validate applicable relative policy and frame consistency.
4. Set alignment time to the newer **state** timestamp.
5. Report raw skew and stale flags relative to that newer time.
6. Predict older attitude using the latest usable body rate only for positive
   intervals within the configured horizon; otherwise use stored attitude.
7. Compute `q_rel=normalize(inverse_unit(q_left)*q_right)`.
8. If reference application is enabled and a reference exists, replace it with
   `normalize(inverse_unit(q_reference)*q_rel)`.

Despite the historical field name `q_left_to_right`, the algebra above is the
authoritative convention: with body-to-world orientations, it maps right-frame
components into the left frame.

Capture uses the **same** latest-time alignment as compute. It requires valid
entity estimates but does not apply all relative publishability/skew checks.

There is no external wall-clock argument to `compute`. Two equally old entities
cannot be recognized as stale relative to real current time; age flags are only
pair-relative. Applications must enforce acquisition freshness independently.

### 8.3 Frame and hinge checks

World gravity directions must align with cosine at least
`0.9998476951563913` (approximately one degree). For heading modes where both
policies have absolute-heading capability, world magnetic reference directions
must meet the same gate.

For twist/swing modes, normalize both hinge axes, transform the right axis into
the left frame with `inverse_unit(q_rel).rotate(axis_right)` as in the inspected
implementation, require absolute dot at least `0.7071067811865476`, reverse its
sign if necessary, then normalize their sum to obtain the resolved hinge axis.
An unusable or inconsistent axis produces configuration/output-not-supported
flags and an invalid result.

**Next-version decision:** the existing hinge-axis transform uses the inverse of
the relative quaternion despite the body-to-world composition convention above.
Resolve this explicitly with asymmetric-axis tests before changing it; pure
coaxial twist tests alone cannot detect a frame-direction mistake.

### 8.4 Scalar extraction

Always populate derived angle fields after successful prerequisites:

- Shortest 3D angle: `2*acos(clamp(abs(normalize(q_rel).w),-1,1))`,
  in `[0,pi]`.
- Tilt: angle between each rotated down axis. With reference application, compare
  the left body down axis against the reference-adjusted relative rotation of
  the right body down axis.
- Heading: project heading axes onto the plane normal to world up; with reference
  application, use the left body up plane. Use signed angle about that normal.
- Twist: project the quaternion vector part onto the resolved hinge axis,
  construct/normalize the twist quaternion, then compute its signed angle with
  `2*atan2(length(twist.vec),abs(twist.w))` and wrap to `[-pi,pi]`.
  A degenerate projected twist returns identity.
- Swing: `q_swing=normalize(q_rel*inverse_unit(q_twist))`; return its shortest
  unsigned angle.
- Roll/pitch/yaw: select the corresponding component of relative Euler angles.

Only `tilt_angle` is classified heading-independent. Other modes require either
full-3D observability or explicitly permitted heading-with-drift.
Degenerate heading projections must reject heading-dependent publication;
a zero secondary heading number is not evidence of a valid heading.

### 8.5 Relative diagnostics and publication

Union both entity flags. Set stale flags if one state age exceeds its policy's
sample-age limit. Set pair-skew flag when raw skew exceeds `max_pair_skew_s`.
These flags degrade quality but do not by themselves unconditionally reject.

Effective observability is:

- Tilt-only when tilt is requested and both sides are at least tilt-observable.
- Full-3D when a heading mode is requested and both sides are full-3D.
- Heading-with-drift when both sides support it and drift policy permits it.
- Otherwise none, with heading-unobservable/output-not-supported and invalidity.

For a heading-dependent output, drift is
`hypot(left_drift,right_drift)`; tilt drift is zero.

```text
c_pair = c_left*c_right                 [heading modes]
c_pair = tilt_c_left*tilt_c_right       [tilt mode]
c_pair *= 0.5                          [excess pair skew]
c_pair *= 0.8                          [heading-with-drift]
c_pair *= clamp(1-drift/nominal_drift_limit_rad,0,1)
```

Clamp final confidence and apply the same quality grading. Mark primary/overall
valid only when confidence is at least the configured publish floor and quality
is non-invalid. Drift above nominal also sets `drift_exceeds_nominal`.
At or above the drift confidence cutoff, heading publication may become invalid
even though attitude is still propagated. Tilt must remain separately available
when gravity remains usable.

## 9. Numerical utility and platform contracts

Numerical helpers and acceleration must be independently usable and must not
alter the ownership or dimension contracts of the core.

### 9.1 Constants and precision selection

In `microla::constants`, provide callable/converting `inline constexpr` proxy
objects: both `pi<T>` and `pi<T>()` work. Default numeric template type is double.
Because these are proxies, template deduction is not identical to a plain `T`.

Inventory: `pi`, `two_pi`, `half_pi`, `quarter_pi`, `e`, `golden_ratio`,
`sqrt2`, `sqrt3`, `ln2`, `ln10`, `deg_to_rad`, `rad_to_deg`, `gravity`,
`speed_of_light`, `epsilon_f`, `epsilon_d`, and templated `epsilon`.
Use standard C++20 number constants where available and typed conversion for
other arithmetic types.

| Constant | Value |
|----------|-------|
| `two_pi` | `6.28318530717958647692` |
| `half_pi` | `1.57079632679489661923` |
| `quarter_pi` | `0.78539816339744830962` |
| Degree/radian factors | `0.01745329251994329577`, `57.29577951308232087680` |
| Gravity | `9.80665` |
| Speed of light | `299792458.0` |
| `epsilon_f` | `8*numeric_limits<float>::epsilon()` |
| `epsilon_d` | `4.5e6*numeric_limits<double>::epsilon()` |
| `epsilon<T>` | `100*numeric_limits<T>::epsilon()` |

Relaxed constant tolerances are not raw machine epsilon. Integral constants
truncate and have zero epsilon. Unsuffixed literals are initially double
precision; long decimal spelling alone does not confer long-double precision.

`detail::high_prec_base` and `detail::hi_t<T>` select double by default,
or long double only when the long-double switch is nonzero and long double has
more mantissa bits. Selection is independent of input `T`, so an already wider
input is not automatically preserved.

### 9.2 Stable numerical functions

`microla::numerical` exposes:

| Function | Contract / algorithm |
|----------|----------------------|
| `epsilon<T>()` | Raw machine epsilon |
| `approx_equal(a,b,rel=100*eps,abs=eps)` | Exact equality first; otherwise reject non-finite operands/tolerances or negative tolerances, then absolute-or-relative test |
| `kahan_sum<T>(begin,end)` | Compensated accumulation in `T`; empty range zero |
| `stable_dot_product(a,b,n)` | Products and compensation in `T`; zero count zero |
| `hypot(a,b)`, `hypot3(a,b,c)` | Stable standard hypot |
| `log1p(x)`, `expm1(x)` | Standard cancellation-aware functions |
| `is_representable(x)` | Finite and at least smallest normal magnitude; zero/subnormals false |
| `relative_error(computed,exact)` | `abs(computed)` for exact zero, else `abs((computed-exact)/exact)` |
| `condition_number_2x2(a,b,c,d)` | Scaled 2-norm condition calculation; zero/singular/non-finite input infinity |
| `horner_eval(coeff,n,x)` | Ascending-power coefficients, highest-to-lowest Horner evaluation; empty zero |
| `is_ill_conditioned(cond,threshold=1e10)` | Non-finite or strictly greater than threshold |
| `precision_loss_bits(cond)` | Non-finite `INT_MAX`; `cond<=1` zero; otherwise truncated `log2(cond)` |
| `ulp_distance(a,b)` | Ordered IEEE float/double bit distance, saturated to `INT64_MAX`; equal values zero, NaN maximum |

For the 2x2 condition number, divide all entries by their largest absolute value,
form `A^T*A = [[aa,ab],[ab,bb]]` in `hi_t<T>`, then:

```text
lambda_max = (aa+bb+hypot(aa-bb,2*ab))/2
condition = lambda_max/abs(det(A_scaled))
```

This avoids computing the small singular value by cancellation-prone subtraction.
Kahan summation compensates roundoff but does not prevent product overflow.
Matching infinities compare equal through the initial equality check.
The numerical namespace suppression macro is a historical inclusion mechanism,
not a configurable namespace-name string.

### 9.3 Guarded scalar math

`microla::safe` supplies always-active:

- `is_finite<T=float>` and `is_nan<T=float>`: IEEE-style bit classification for
  four/eight-byte floating representations.
- `safe_divide(n,d,fallback=0)`: floating fallback for `abs(d)<machine_epsilon`;
  integer fallback for zero divisor and signed minimum divided by minus one.
- `safe_reciprocal(x,fallback=0)`: floating-only guarded `1/x`.
- `safe_sqrt(x)`: existing NaN preserved, negative input NaN, otherwise sqrt.
- `safe_acos`/`safe_asin`: clamp beyond `[-1,1]` to endpoint angles.
- Integral `saturating_add`, `saturating_sub`, `saturating_mul`: precheck overflow
  and clamp to type extrema; unsigned subtraction floors at zero.

These are not a universal “make every invalid floating input zero” policy:
NaN/infinity are not separately sanitized by guarded division.

When `MICROLA_SAFE_MODE` is defined, `check_bounds(i,n)` returns `i<n`,
`assert_finite(x)` returns zero for non-finite `x`, and
`is_well_conditioned(cond,threshold=1e6)` requires finite and strictly below the
threshold. Otherwise they return true, unchanged input, and true respectively.
Despite the name, `assert_finite` does not call an assertion handler.

### 9.4 Opt-in fast math

`microla::fast` is explicitly separate from exact/stable APIs:

| API | Algorithm / edge behavior |
|-----|---------------------------|
| `sin<T=float>(x)` | Bhaskara I on `[-pi,pi]`; standard sine for non-finite/out-of-domain input |
| `cos<T=float>(x)` | Shifted approximate sine on `[-pi/2,pi/2]`; standard cosine otherwise |
| `tan<T=float>(x)` | Ratio of fast sine/cosine; near-zero cosine yields signed infinity |
| `rsqrt<Iterations=1>(float)` | Bit seed `0x5f3759df-(bits>>1)` plus 0-3 Newton iterations |
| `sqrt<Iterations=1>(float)` | `x<=0` returns zero; otherwise `x*rsqrt(x)` |
| `atan2<T=float>(y,x)` | Rational approximation and quadrant correction |
| `exp<T=float>(x)` | Base-2 range reduction and Pade approximation |
| `ln(float)` | `frexp` reduction and odd atanh-style series through degree 11 |
| `pow(float x,float y)` | `exp(y*ln(x))` only |
| `abs(float)` | Clear sign bit |
| `sign<T=float>(x)` | `(0<x)-(x<0)`; NaN zero |
| `lerp<T=float>(a,b,t)` | `a+t*(b-a)`; no clamping |
| `smoothstep<T=float>(lo,hi,x)` | Clamp normalized parameter, then `t*t*(3-2*t)` |

Sine formula for reduced magnitude `u=abs(x)`:

```text
sin_approx = 16*u*(pi-u)/(5*pi*pi-4*u*(pi-u))
```

Restore the input sign. Reciprocal-square-root Newton step is
`y*=1.5-0.5*x*y*y`; iteration count outside `[0,3]` is a compile-time error.
Its raw API does not enforce a positive finite input.

Atan uses `z/(1+0.28*z*z)` for `abs(z)<1` and the reciprocal identity otherwise.
For `x=0`, atan2 chooses signed half-pi according to `y`, or zero. It does not
fully reproduce standard signed-zero/NaN/infinity semantics.

For exp, choose `k=round(x/ln2)`, reduced argument `r=x-k*ln2`, approximate
`(12+6*r+r*r)/(12-6*r+r*r)`, then restore with `ldexp`.
Preserve NaN; overflow threshold is `max_exponent*ln2`, underflow floor
`(min_exponent-digits-1)*ln2`.
Log returns negative infinity at zero, NaN below zero, and preserves NaN/positive
infinity. With mantissa `m` in `[1,2)`, use `z=(m-1)/(m+1)` and odd terms
`2*(z+z^3/3+z^5/5+z^7/7+z^9/9+z^11/11)` plus exponent times ln2.

No independent negative-base/zero-power semantics are provided by fast pow.
Smoothstep equal/reversed edges are not explicitly validated.
Approximation error and speed must be measured for the intended input domain;
commented speedups are not certified requirements.

### 9.5 Compiler gates, alignment, and allocation policy

Defining `MICROLA_EMBEDDED` supplies no-exception/no-RTTI/no-dynamic-allocation
macros unless already defined and defaults the per-object stack limit to 2048
bytes. Nonembedded default is 4096. A positive configured limit is required.

`MICROLA_HAS_EXCEPTIONS`, `MICROLA_HAS_RTTI`, and
`MICROLA_DYNAMIC_ALLOC_ONLY(...)` expose compile-time feature policy.
Allocation policy removes designated APIs; it does not replace global allocation
operators or prevent allocations in user code.

Autodetection runs only when enabled, mapping compiler NEON/MVE/AVX512/AVX
macros to configuration switches. CMSIS is application-supplied, not autodetected.
RISC-V has detection-spelling differences in the current feature and cache-line
branches; do not equate a RISC-V configuration marker with a vector kernel.

Default `MICROLA_DATA_ALIGNMENT`, overridable before headers:

| Priority branch | Alignment |
|-----------------|-----------|
| AVX512 | 64 bytes |
| AVX | 32 bytes |
| NEON/MVE | 16 bytes |
| RISC-V configuration | 16 bytes |
| CMSIS | 4 bytes |
| Scalar embedded | 4 bytes |
| Host/default | 16 bytes |

Provide `MICROLA_CONSTEXPR20`, `MICROLA_CONSTEVAL`,
`MICROLA_CONSTEXPR23`, `MICROLA_IF_CONSTEVAL`, `MICROLA_CONSTEXPR26`,
`MICROLA_CONSTEXPR_TRIG`, force-inline, assumption, unreachable, bit-cast,
cache/DMA alignment, ROM/flash-placement, and barrier macros.
Use standard feature-test macros where the inspected implementation does;
current C++26/trigonometry gates use `__cplusplus>=202600L`.

Fallback bit-cast uses `memcpy` with size/trivial-copyability assertions.
False optimization assumptions are undefined-behavior precondition violations,
not error checks. ISR-safe/timing-critical/constant-time markers are documentary.
The MSVC barrier branch is a compiler barrier, not a hardware fence; generic
barrier fallbacks may be no-ops. DMA alignment and placement macros do not ensure
cache maintenance, bus accessibility, or interrupt safety.

### 9.6 SIMD buffer primitives

In `microla::simd`, provide:

- Float/double fill and contiguous copy; generic arithmetic `fill/copy_n`.
- `copy4_float`, `strided_copy_float`, generic strided copy.
- `load_padded4(src,n,out,pad=0)` and `store_extract4(src4,dst,n)`.
- Four-lane `add4`, `sub4`, `mul4`, `fma4_accumulate`.
- Indexed `gather_n_float`, `scatter_n_float`, `gather4_float`,
  `scatter4_float`, optimized gather/scatter, and fixed-stride gather/scatter.

All use caller-owned buffers and element indices/strides. No allocation, ownership,
or bounds validation is added. Padded/extracted counts clamp to four; do not read
the nonexistent fourth component of a Vec3. Indexed scatter is ordered; later
duplicate indices overwrite earlier ones. Copy is not memmove.

Implementation coverage:

- Float bulk: AVX512 16 lanes, AVX 8, NEON 4, then scalar tails.
- Double bulk: AVX512 8 lanes, AVX 4, AArch64 NEON 2, then scalar tails.
- Four-float helpers use 128-bit x86 instructions under the AVX switch or NEON.
  AVX512 alone does not enable every four-float helper branch.
- FMA fuses only with the corresponding FMA feature switch; otherwise separate
  multiply/add is valid.
- Generic helpers specialize float only; explicit double helpers are separate.
- Optimized indexed operations detect an entirely contiguous index sequence;
  other indices generally gather/scatter scalarly.
- AVX2 four-element gathers require indices representable as signed 32-bit
  offsets. x86 loads/stores are unaligned.
- MVE/CMSIS/RISC-V have no specialized branches in this buffer-helper header;
  other core-header dispatch must be inspected separately.

### 9.7 Resource inspection and version

`microla::resource_checks` provides layout and DMA checks for vector, matrix,
and quaternion; common float aliases; and `VecSizeInfo`, `MatSizeInfo`,
`QuaternionSizeInfo` with element/theoretical/actual size, alignment, and padding.
Vectors add dimension; matrices add rows/cols.

Vector/quaternion layout checks bound complete object size, while the inspected
matrix check bounds payload size and verifies actual size is at least payload.
DMA checks require trivial copying and scalar-compatible alignment, not
device-specific DMA certification. Merely declaring an alias need not instantiate
all assertions.

Umbrella `memory_info` additionally supplies actual matrix/vector/quaternion size
queries, matrix alignment, stack-limit comparison, and validation predicates.
Umbrella concepts are `Arithmetic`, `FloatingPoint`, and `Integral`.
Angle helpers are `deg_to_rad`, `rad_to_deg`, `wrap_pi`, `wrap_two_pi`,
`angle_distance`, `clamp`, and `saturate`.
Runtime wrapping uses fmod; constexpr wrapping performs bounded scaled remainder
reduction without an integral quotient cast. Non-finite floating inputs produce
NaN. `wrap_pi` retains endpoints in `[-pi,pi]`; `wrap_two_pi` produces `[0,2*pi)`;
angular distance is wrapped `to-from`.

`version.hpp` defines `MICROLA_VERSION_MAJOR/MINOR/PATCH/STRING/NUMBER`.
Numeric packing is `major*10000+minor*100+patch`. `Version` has public component
fields but static `string`, `number`, and component getters return the library
macros, not instance fields. Free functions are `get_version_string`,
`get_version_number`, and `version_at_least(major,minor,patch)`.
The latter compares packed numbers without validating component ranges.

## 10. Packaging and build integration

Evidence: [CMake configuration](../CMakeLists.txt),
[presets](../CMakePresets.json), and [Zephyr integration](../zephyr/CMakeLists.txt).

### 10.1 CMake interface

Create an `INTERFACE` library named `microla` and alias `microla::microla`.
Propagate the build/install include directories and `cxx_std_20`. No compiled
library artifact is needed.

Read version from `VERSION`. Install the complete header directory, exported
interface target, package config, and same-major compatible version config under
the standard GNU installation directories.

| Option | Default / effect |
|--------|------------------|
| `MICROLA_EMBEDDED` | OFF; enable embedded macro and exception/RTTI restrictions |
| `MICROLA_USE_LONG_DOUBLE_INTERMEDIATES` | ON; numeric `0/1` definition |
| `MICROLA_AUTODETECT_SIMD` | ON; propagate autodetect switch |
| `MICROLA_DISABLE_EXCEPTIONS` | OFF; policy definition and compiler flags |
| `MICROLA_DISABLE_RTTI` | OFF; policy definition and compiler flags |
| `MICROLA_ENABLE_NEON` | OFF; target validation and NEON definition/flags |
| `MICROLA_ENABLE_CMSIS` | OFF; definition; application provides CMSIS-DSP |
| `MICROLA_LINEAR_BUILD_EXAMPLES` | OFF |
| `MICROLA_LINEAR_BUILD_TESTS` | OFF |
| `MICROLA_LINEAR_BUILD_BENCHMARKS` | OFF |
| `MICROLA_LINEAR_BUILD_TARGET_SMOKE` | OFF; compile-only resource object/report |
| `MICROLA_LINEAR_BUILD_RESOURCE_PLAN` | OFF; ABI-specific persistent-size probe |
| `MICROLA_LINEAR_BUILD_DOCS` | OFF; optional Doxygen |

GCC/Clang no-exception/no-RTTI consumers receive `-fno-exceptions` and `-fno-rtti`;
MSVC and IAR use their equivalents. Direct-header consumers must supply matching
compiler flags themselves. AArch64 NEON uses architectural support; 32-bit Arm
may need `-mfpu=neon`; reject non-Arm explicit NEON configuration.

**Next-version decision:** the inspected long-double CMake definition is
directory-scoped rather than exclusively target-scoped. Prefer interface target
definitions so installed consumers and unrelated siblings receive the intended
policy without accidental leakage.

The root configuration minimum is CMake 3.13.1; enabling tests needs 3.14.
The checked-in presets declare a 3.23 minimum and schema version 6; using those
presets requires a CMake implementation that supports the schema, rather than
assuming the low package minimum suffices.

### 10.2 Zephyr and package managers

- Zephyr module integration conditionally creates the interface library under
  `CONFIG_MICROLA`; Kconfig controls debug, NEON, CMSIS, MVE, and long-double
  intermediates. Link against Zephyr's interface and CMSIS when configured.
- Do not assume the Zephyr path automatically applies every standalone embedded
  CMake policy; applications must configure runtime/compiler restrictions.
- Conan exposes tests, benchmarks, examples, NEON, and CMSIS options and exports
  a header-only package with target name `microla::microla`.
- vcpkg metadata declares the version, Apache-2.0 license, packaging helpers,
  and optional tests/benchmarks/NEON/CMSIS features.
- Keep package metadata and public version queries synchronized with `VERSION`.
- **Next-version decision:** audit source-package completeness, especially the
  canonical version file, and avoid conflicting test-dependency provisioning.

### 10.3 Licensing and generated documentation

Retain Apache-2.0 licensing and applicable notices in redistributions. Generate API
reference with Doxygen and architecture diagrams from their source format;
generated HTML/images are not the authoritative behavioral specification.
Do not copy external library implementations merely to reproduce their algorithms.

## 11. Verification and acceptance

Reconstruction is complete only after the public API and numerical/state contracts
are implemented and verified. Compiling a demonstration alone is insufficient.

### 11.1 Build matrix

Maintain these independent validation layers:

1. Host GCC, Clang, and MSVC C++20 builds.
2. Individual public-header compilation and multiple-translation-unit linkage.
3. Constexpr/static-trait compilation.
4. Embedded host policy tests.
5. ASan/UBSan/alignment diagnostics and static analysis.
6. Zephyr QEMU/native execution where supported.
7. Compile-only Cortex-M0+, M3, M4F, M7F, AArch64, and RV64GC resource probes.
8. On-device tests for any claimed target timing/SIMD/resource property.

Compile-only probes and emulators do not establish board startup, peripherals,
interrupt behavior, WCET, whole-image flash/RAM, or physical sensor correctness.

### 11.2 Numerical test methodology

- Use exact checks for indexing, integer arithmetic where defined, defaults,
  traits, flags, dimensions, and transactional unchanged-state behavior.
- Use absolute/relative tolerances scaled by scalar precision and problem
  conditioning for floating-point results; do not impose one epsilon on all
  physical units and algorithms.
- Verify identities: dot/cross orthogonality, quaternion/matrix rotation
  equivalence, `A*inverse(A)`, decomposition reconstruction, solve residuals,
  and eigenpair residuals.
- Exercise zero, near-zero, very large/small finite values, singular and
  ill-conditioned matrices, NaN, infinities, and failed convergence.
- Compare optional kernels against the scalar reference over deterministic
  randomized inputs, odd sizes, tails, and unaligned borrowed storage.
- Confirm rejection leaves **all** protected state unchanged, not just the
  primary scalar or quaternion.
- Approximation tests must assert measured domain/error envelopes separately
  from exact/stable helper tests.

### 11.3 Filter and fusion acceptance cases

Core mathematical regression targets, in addition to the identity tests above:

- Float/double extreme vector and quaternion normalization, including finite
  values near maximum, minimum normal, and minimum subnormal: finite unit result
  within `16*epsilon(T)`.
- Rectangular float products with non-vector-width tails, including dimensions
  `(2,3,5)`, `(3,7,4)`, `(6,6,6)`, and `(33,31,35)`: agreement with the
  double-reference fixture within `1e-6`.
- Double checked inversion at overflow-prone row-sum scales and aliasing:
  identity-product residual within `1e-12`.
- Double rank/QR under uniform scaling by `1e-200` and `1e200`: unchanged rank,
  normalized reconstruction and orthogonality within `1e-12`.
- Late-pivot LU: `P*A=L*U` within `1e-5`.
- Tall/wide/rank-deficient/zero SVD: convergence and reconstruction/full-factor
  orthogonality within `1e-4`; a nontrivial zero-sweep request must report failure
  to converge.
- Quaternion rotation distance: sign/scaling invariance in double within `1e-12`.

Add new coverage, rather than pretending it already exists, for look-at
handedness, non-unit/zero rotation axes, asymmetric hinge frames, malformed
release-mode inputs through checked APIs, quaternion backend failure parity,
full Moore-Penrose pseudoinverse identities, and non-real QR spectra.

| Case | Required observable outcome |
|------|-----------------------------|
| Linear/EKF bad prediction | State and covariance both unchanged |
| Singular/non-finite innovation | Update false; estimates unchanged; NIS infinity |
| Scaled well-conditioned innovation | Accepted regardless of small absolute determinant |
| Nonlinear angular measurement | Custom residual takes the short wrapped difference |
| Invalid fusion startup | No initialized state; invalid diagnostic and startup/reason flags |
| Duplicate/reordered timestamp | No correction, propagation, bias/reference change, or clock advance |
| Large time gap | No integration across gap; valid gyro not mislabeled rejected |
| Complete IMU dropout | Held quaternion, retained state time, invalid/zero confidence, no prediction |
| Dropout recovery | Integrate only latest processed interval |
| Timestamp rebase | Preserve attitude, covariance, drift, learned field, and held age |
| Calibration replacement | Reset affected estimator and invalidate captured pair reference |
| Equivalent magnetic units | Equivalent attitude and gating for consistently scaled fields |
| Vertical/zero magnetic field | No absolute heading lock |
| Magnetic NIS rejection | No state/reference learning commit; heading-with-drift |
| MEKF bias disabled | No bias correction/noise growth/coupling |
| MEKF reset covariance | Full Joseph-plus-reset result including cross-covariance |
| Heading confidence expiry | Gravity-valid tilt remains available separately |
| Reference capture with skew | Capture and compute use identical predicted alignment |
| Frame/hinge inconsistency | Configuration/output-not-supported flags and invalid result |
| Excess pair skew | Skew flag plus prescribed confidence penalty |
| Accel-only held publication | Explicit configured degraded policy, never a fresh timestamp |

Existing measurable regression targets include:

- For each axis, both IMU9 backends reduce induced attitude error below half
  its initial value after 300 stationary corrections at `dt=0.01`.
- MEKF reset covariance agrees with independently computed expected values
  within `100*epsilon(T)`.
- Across 200 repeated corrections for IMU6/IMU9 float/double, covariance remains
  finite and symmetric within `10*epsilon(T)`, with positive elimination pivots.
- Gravity-only dropout preserves existing heading within `1e-6` radians in
  the double-precision fixture and reduces newly introduced tilt error.
- Pure 45-degree twist and 20-degree swing fixtures agree within `0.05` radians.

These are regression tolerances, not a universal deployed-sensor accuracy claim.
Replay deterministic lifecycle traces for stationary/yaw motion, magnetic
disturbance, saturation, freefall, high-g acceleration, timestamp reordering,
rebasing, paired skew, and reference invalidation.

### 11.4 Tooling and reproducibility

The host suite uses GoogleTest fetched at a pinned commit. Compile each public
header into both normal and embedded-contract targets. Test compilation explicitly
disables fast-math under GCC/Clang so `isfinite` assertions retain IEEE semantics.
When examples and tests are enabled together, execute the maintained sensor-fusion
example as a smoke test.

Representative commands, run from the repository root:

```bash
cmake --preset host-tests
cmake --build --preset host-tests
ctest --preset host-tests --output-on-failure

cmake --preset embedded-tests
cmake --build --preset embedded-tests
ctest --preset embedded-tests --output-on-failure

cmake --preset sanitizers-ci
cmake --build --preset sanitizers-ci
ctest --preset sanitizers-ci --output-on-failure

python3 scripts/verify_snippets.py docs/Design.md --verbose --require-compiled
markdownlint-cli2 --config .markdownlint.json docs/Design.md
```

Documentation snippets must include at least one complete program and actually
compile; incomplete fragments must not count as positive validation evidence.

### 11.5 Performance and resource acceptance

- Benchmark with a pinned Google Benchmark dependency in Release mode.
- Capture compiler, flags, CPU, architecture, environment, and raw JSON results.
- Compare like-for-like workloads; the existing baseline comparator uses a `1.05`
  ratio threshold. Historical timings are not specifications for other machines.
- Developer-native `-march=native` results must not substitute for portable CI
  or target measurements.
- Measure persistent object size on the target ABI, not a guessed host formula.
- Compiler `.su` reports give individual function-frame sizes, not total
  call-path/interrupt nesting bounds.
- Compile-only object section sizes are not linked-image flash budgets.
- Enforce process-wide heap bans in the application as well as excluding the
  library's allocating overloads.
- Set application-specific RAM, flash, stack, timing, and accuracy budgets before
  deployment; none are universally guaranteed by the header-only design.

## 12. Implementation sequence

1. Define language gates, scalar traits, alignment, constants, version interface,
   and independent-header build checks.
2. Implement scalar vector and matrix storage/arithmetic and exact dimension
   constraints.
3. Implement quaternion conventions and cross-check rotations against matrices.
4. Implement checked inversion/solvers, decompositions, stable arithmetic, and
   explicit allocating/non-allocating algorithm boundaries.
5. Implement views and geometry with ownership/stride/degeneracy tests.
6. Implement generic Kalman and EKF transactional updates.
7. Implement calibration, measurement gating, clocks, and accel-only attitude.
8. Add Mahony, MEKF, covariance reset, and forward prediction.
9. Add relative alignment, reference capture, observability, scalar extraction,
   flags, and confidence.
10. Add accelerated kernels only after scalar correctness; validate dispatch and
    compare resource/performance evidence.
11. Package standalone/installed/Zephyr integrations and execute the validation
    matrix.

Avoid building the new library around the old implementation's incidental helper
names. Preserve public contracts, mathematical conventions, deterministic state
transitions, and failure behavior; document intentional incompatibilities.

## 13. Reconstruction smoke program

This complete program exercises core arithmetic, generic filtering, both fusion
layers, and explicit geometry/version includes without external runtime packages.

```cpp
#include <microla/microla.hpp>
#include <microla/geometry.hpp>
#include <microla/version.hpp>

#include <cmath>

struct TiltPolicy : microla::fusion::DefaultRelativeAngleConfig<float>
{
    static constexpr auto default_output =
        microla::fusion::PrimaryScalarOutput::tilt_angle;
};

int main()
{
    using microla::Mat;
    using microla::Vec;
    using namespace microla::fusion;

    const Vec<float, 3> x(1.0F, 0.0F, 0.0F);
    const Vec<float, 3> y(0.0F, 1.0F, 0.0F);
    const auto z = x.cross(y);
    if (std::abs(z[2] - 1.0F) > 1e-6F) {
        return 1;
    }

    Mat<float, 3, 3> inverse;
    if (!Mat<float, 3, 3>::identity().inverse_checked(inverse)) {
        return 2;
    }

    microla::KalmanFilter<float, 2, 1> filter;
    filter.set_measurement_matrix(Mat<float, 1, 2>({1.0F, 0.0F}));
    filter.predict();
    if (!filter.update(Vec<float, 1>(0.0F))) {
        return 3;
    }

    OrientationEstimator<float, DefaultImu6MahonyConfig<float>> orientation;
    const Vec<float, 3> gravity(0.0F, 0.0F, -microla::constants::gravity<float>());
    orientation.update(Imu6Sample<float>{0.0F, Vec<float, 3>(), gravity});
    if (!orientation.estimate().valid) {
        return 4;
    }

    RelativeAngleEstimator<float, DefaultAccelOnlyConfig<float>,
                           DefaultAccelOnlyConfig<float>, TiltPolicy> pair;
    pair.update_left(AccelSample<float>{0.0F, gravity});
    pair.update_right(AccelSample<float>{0.0F, gravity});
    const auto result = pair.compute();
    return result.valid && std::abs(result.primary.angle_rad) < 1e-5F ? 0 : 5;
}
```

## 14. Evidence and maintenance

The reconstruction specification is grounded in the public headers and tests,
not inferred from the library name or historical benchmark claims.

- [Core headers](../include/microla/).
- [Host behavioral tests](../tests/google/).
- [Embedded tests](../tests/zephyr/).
- [Release support boundary](RELEASE_CONTRACT.md).
- [Sensor-fusion operational guide](SENSOR_FUSION.md).
- [Build presets](../CMakePresets.json).
- [Benchmark configuration](../benchmarks/CMakeLists.txt).

When creating a new version, update this document alongside changes to defaults,
layouts, supported scalar domains, algorithm convergence, failure signaling,
sensor conventions, policy thresholds, or package boundaries. Keep regression
tests for every intentionally retained behavior and identify changed behavior in
release notes.
