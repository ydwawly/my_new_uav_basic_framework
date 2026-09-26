# VQF upstream record

- Project: VQF (Versatile Quaternion-based Filter)
- Upstream: https://github.com/dlaidig/vqf
- Commit: `86ba56bdd3158b9b05f9f9fe5596866ba326438c`
- Imported files: `vqf.cpp`, `vqf.hpp`
- License: MIT, see `LICENSE.MIT.txt`
- Copyright: 2021 Daniel Laidig <laidig@control.tu-berlin.de>

The upstream algorithm files are kept unmodified. The local `VQF_C.*` files provide a
heap-free singleton C interface and convert VQF's ENU output to the flight controller's
NED convention. `VQF_SINGLE_PRECISION` is supplied only to these C++ translation units
by CMake; VQF's numerically sensitive Butterworth state remains double precision as
implemented upstream.
