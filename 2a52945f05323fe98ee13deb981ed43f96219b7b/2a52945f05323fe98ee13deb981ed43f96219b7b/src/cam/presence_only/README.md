# Presence-Only Variant

This folder keeps the same Python structure as the current `wro.py` flow, but removes center/location logic.

Run on Windows:

```powershell
py -3.12 codes/presence_only/wro.py
```

Run on Raspberry Pi:

```bash
python3 wroPI.py
```

This variant uses BGR thresholding plus pixel counting, because for solid red and green blobs it was the fastest option we measured for presence-only detection.
