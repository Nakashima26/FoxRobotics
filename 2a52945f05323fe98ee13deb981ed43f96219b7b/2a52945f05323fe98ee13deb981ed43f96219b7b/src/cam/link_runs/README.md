# Link-Runs Variant

This folder keeps the same Python flow as the current `wro.py` version, but replaces the contour step with `findContoursLinkRuns` when OpenCV provides it.

Run on Windows:

```powershell
py -3.12 codes/link_runs/wro.py
```

Run on Raspberry Pi:

```bash
python3 wroPI.py
```

This variant still computes location and center, so it is the best fit if the robot needs to steer using object position.
