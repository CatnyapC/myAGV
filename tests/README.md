# Tests

Run Python tests from the repository root:

```sh
.venv/bin/python -m unittest discover -s tests -t .
.venv/bin/python -m unittest tests.test_navigation
```

Frontend tests live in `web/tests/`. Run them with `npm --prefix web test`;
`npm --prefix web run build` also type-checks them.

The C++ navigation tests require ROS development headers and libraries.
See `HARDWARE_WEBUI.md` for the trajectory planner test command.

`tests/manual/move_test.py` and `tests/manual/move_test_extended.py` drive
real hardware. Run them explicitly only when physical motion is intended;
they are excluded from automated test discovery.
