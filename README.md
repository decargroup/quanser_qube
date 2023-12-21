# Quanser _QUBE-Servo_

This repository contains the C code required to generate the experimental
dataset used in
[Closed-Loop Koopman Operator Approximation](https://arxiv.org/abs/2303.15318),
which can be found in the
[companion repository](https://github.com/decargroup/closed_loop_koopman).

This dataset uses the
[Quanser _QUBE-Servo_](https://www.quanser.com/products/qube-servo-2/) 
(version 1), but makes use of the
[Quanser HIL SDK](https://github.com/quanser/hil_sdk_win64)
to use the device, rather than Simulink.

## Requirements

This software has only been tested with the Windows version of the Quanser
HIL SDK, though a
[Linux version](https://github.com/quanser/hil_sdk_linux_x86_64) does exist that
should be compatible with the _QUBE-Servo_.

First, install the [Quanser HIL SDK](https://github.com/quanser/hil_sdk_win64)
and ensure that the environment variable `%HIL_DIR%` is set to the appropriate
location. This should be `C:\Program Files\Quanser\HIL SDK`.

Next, install
[Microsoft Visual Studio Community](https://visualstudio.microsoft.com/vs/community/),
as the HIL SDK only supports the `msvc` compiler.

Finally, install [CMake](https://cmake.org/), which will be used to build the
project.

Optionally, to run the unit tests, install
[Catch2](https://github.com/catchorg/Catch2) with CMake integration via `vcpkg`.
To do so, follow
[these instructions](https://github.com/catchorg/Catch2/blob/devel/docs/cmake-integration.md#installing-catch2-from-vcpkg)
in the `x86 Native Tools` terminal. However, be sure to install the 64-bit
version of Catch2 instead, by running
`vcpkg.exe install catch2:x64-windows`.

## Usage

To compile the project, first create a directory called `build/` inside the
root of the repository, navigate to it, and run CMake to generate the required
Visual Studio projects:
```
> mkdir build
> cd build
> cmake ..
```

Then run
```
cmake --build .
```
from within `build/` to compile the project.

The main executable is `build/src/Debug/QubeBalance.exe`, which can be run
as-is, or with seeds for the pseudorandom binary sequences as arguments:
```
> QubeBalance.exe 123 456 789
```

If the _QUBE-Servo_ is connected, it will wait for the user to position the
pendulum upright before running the controller. After execution, the program
will save a CSV file containing the results wherever the program was run from.
It will have a name like
`qube_<TIMESTAMP>_<THETA_SEED>_<ALPHA_SEED>_<FEEDFORWARD_SEED>.csv`

Optionally, to run all the unit tests, run
```
> ctest -C Debug
```

## Outputs

The CSV output has the following columns

| Column | Description |
| --- | --- |
| `t` | Timestamp (s) |
| `target_theta` | Target motor angle (rad) |
| `target_alpha` | Target pendulum angle (rad) |
| `theta` | Measured motor angle (rad) |
| `alpha` | Measured pendulum angle (rad) |
| `control_output` | Control signal calculated by the controller (V) |
| `feedforward` | Feedforward signal to be added to `control_output` (V) |
| `plant_input` | `control_output` summed with `feedforward` (V), saturated between -10V and 10V |
| `saturation` | Signal indicating if saturation is active (_i.e._, -1 if saturating in the negative direction, +1 if saturating in the positive direction) |