# Vendored rsplan runtime

Source: https://github.com/builtrobotics/rsplan

Pinned commit: `47b3ab572a4b7ffc314dfc2aa68b8349f061fe13`

Copyright (c) 2023 Built Robotics. Distributed under the MIT License; see LICENSE.

The five Python runtime files in `rsplan/` are unmodified upstream files. Their Git blob hashes were checked against the pinned upstream revision. LICENSE has only an added trailing newline. Optional upstream plotting/demo/test files are not included. Runtime dependency: NumPy (already used by this project).

Integration, scene construction, sampled footprint checks, vehicle tracking, speed selection, UI and physical evaluation are implemented separately in `rs_course.py`. The planner is geometric Reeds-Shepp planning and does not learn or avoid obstacles itself. It assumes car-like paths with a chosen minimum turning radius; the simulated skid-steer proxy has different dynamics.

Upstream runtime Git blob hashes:

| File | SHA-1 |
| --- | --- |
| __init__.py | 3dbc0fe82a509328c0c34dc15a543b263c91cd89 |
| planner.py | 323e7de72edb28d6cc8dbd85f99be2f511bc487e |
| primitives.py | 19396eae0b43877ec55312bc2ae2f3753ccff438 |
| helpers.py | 709aa8b1e34a87a5d2a81afe16b26a00aa86354f |
| curves.py | 46e03eea15c18de5d63ad89f4c812fe8162ce632 |
