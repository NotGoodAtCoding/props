# props

Open-source software research, architecture, and lean MIT-licensed implementation (**`op-core`**) for practicing clinicians and technicians in **Prosthetics & Orthotics (O&P)**.

## Repository Index
- **Research Synthesis:** [`research/practitioner_software_needs.md`](./research/practitioner_software_needs.md) — Audit of practitioner pain points, existing open-source tools (`ufit-blender`, `orthopen`, `ampscan`, `SocketMixer`, `OpenLimbTT`/`TF`), and priority rankings.
- **Action Plan (v2.1):** [`o_p_technology_action_plan.md`](./o_p_technology_action_plan.md) — Reviewer-refined architecture (`op-core` + `.oprecipe.json`), project portfolio, tech stack, and 16-week TT-first MVP roadmap.
- **Validation Experiments:** [`experiments/high_value_experiments.md`](./experiments/high_value_experiments.md) — Time-boxed, falsifiable technical & clinical-UX experiments (`EXP-1` through `EXP-6`).
- **Core Package (`op-core`):** [`src/op_core/`](./src/op_core/) — Zero-dependency Python 3.12+ computational geometry kernel, `.oprecipe.json` v0.1 engine, Scan QA, TT/AFO rectification & shelling, dual-schema `.3mf` export, carver positive prep, and safety linter.
- **License:** [`LICENSE`](./LICENSE) (MIT).

## Quick Verification
```bash
PYTHONPATH=src python3 -m unittest discover -s tests -v
PYTHONPATH=src python3 scripts/run_experiments.py
```
