# drive.magic

Open-hardware harmonic (strain wave) drive actuators, and the tools to design
them. Printable gearboxes with real measured data — no gatekeeping.

## Pages

- `index.html` — landing (actuators · harmonic maker · measured)
- `actuators.html` — drive index (HDP30 current, HD20 archived)
- `hdp30.html` — 30:1 pancake harmonic drive, NEMA 17, ~$4 hardware
- `hd20.html` — archived 20:1 drive (the only one with load-cell data)
- `hd20-torque-test.html` — full HD20 load-cell report (Plotly)
- `gearmaker.html` — parametric harmonic drive tooth profile generator
  (cycloid / S-tooth, cup / pancake, DXF export)
- `learn.html` — harmonic drives, plainly

## Run locally

```bash
python -m http.server 8080
```

then open http://localhost:8080

## Stack

- Plain HTML + CSS + vanilla JS — no build step, no framework
- `js/ctp-revB.js` — clean-room cycloid tooth profile solver (Yao et al.,
  *Actuators* 2025, 14, 187)
- `js/dxf.js` — DXF export
- `js/gearmaker.js` — the maker UI
- `js/bom.js` — renders `data/*-bom.json` (local snapshot, no external fetch)

## License

CC BY-SA 4.0 for the hardware; MIT for the code.
