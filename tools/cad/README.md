# CAD to description meshes

`step_to_description_meshes.py` regenerates the 2.0 base visual meshes of
`ros2/src/openamrobot_description` from the hardware CAD. The chassis comes from the CAD; the
components come from the 2.0 bill of materials (BOM Issue 7.11, 25 Oct 2026) and are simplified
envelopes; the battery cell-group envelope follows the project-lead decision of 25 Oct 2026.

- Input: the v0.2 full-assembly STEP in a checkout of `openAMRobot/openamr-platform-hw`
  (`mechanical/cad/Full_assembly_STEP/MMP.00.00.00.000_full_assembly.STEP.zip`), the production
  camera mount STEP, and the ZLLG80ASM250-L-B2 double-shaft hub motor STEP.
- `base_mesh_map.yaml` classifies every leaf instance of the assembly as `chassis` (exported),
  `release_001` (excluded), `component` (replaced by its envelope) or `fastener_small` (excluded).
  The run fails while any instance is unclassified. It also holds the envelope parameters with
  their dimension source, the STEP corrections and the design values of the engineering checks.
- Output: `meshes/visual/*.stl` (binary, metres), `meshes/PROVENANCE.yaml`,
  `build/base_visual.glb` (base_link, for the docs roadmap pages; not committed; one node per
  CAD part instance or component envelope, named by a stable id, with glTF `extras`: id, name,
  class `chassis`, `component` or `envelope`, source, BOM 7.11 row and `provisional: true` for a
  provisional allocation) and
  `build/cad_checks.md` / `build/cad_checks.yaml` (CAD versus design, reported only).

## Run

```bash
pip install --break-system-packages cadquery-ocp==8.0.1.1.0 trimesh==5.2.0 \
  fast-simplification==0.2.0 numpy pyyaml
git clone --depth 1 https://github.com/openAMRobot/openamr-platform-hw ../openamr-platform-hw
# The B2 hub motor STEP is on the platform-hw PR #28 branch until it is merged:
git -C ../openamr-platform-hw fetch --depth 1 origin refs/pull/28/head:pr28
mkdir -p /tmp/hub && git -C ../openamr-platform-hw show \
  "pr28:datasheets/ZLTech/wheel_ZLLG80ASM250/ZLLG80ASM250-L-B2 V1.01 double-shaft.STEP" > /tmp/hub/B2.STEP
python3 tools/cad/step_to_description_meshes.py --hw-repo ../openamr-platform-hw \
  --hub-step /tmp/hub/B2.STEP --hub-commit "$(git -C ../openamr-platform-hw rev-parse pr28)"
```

Once the B2 STEP is on platform-hw main, `--hub-step` and `--hub-commit` can be dropped. The STEP
is unzipped into a temporary directory outside the repository (`--scratch` to choose it). The
run takes about four minutes. Then run the description tests:

```bash
python3 -m pytest ros2/src/openamrobot_description/test/test_description_meshes.py
```

## Method

- OCP / OpenCascade XCAF keeps the assembly tree, instance names (cp1251 and cp866 names are
  decoded), placements and colours. Tessellation: linear deflection 0.5 mm, angular 0.35 rad.
- base_link: the STEP has no 2.0 wheels, so the release 001 wheel axis gives the axle line, the
  castor wheels give the floor, and the axle height is checked against 0.098 m. The 2.0 hub
  motors are placed at the register values (track 0.400 placeholder).
- Decimation: fast-simplification quadric decimation per part to a common ratio, so that every
  STL is at most 2 MB, all visual meshes at most 8 MB and the GLB at most 1.5 MB.
- Engineering checks compare the transformed CAD with the register; they never change a frame
  or a register value. The hub motor fit uses the B2 STEP solids against the chassis meshes
  (brake side inboard, decided 25 Oct 2026; the outboard case is still reported).
- Battery cell-group envelope (provisional, 314 (y) x 140 (x) x 210 (z) mm: eight EVE LF105
  cells upright in one row across the robot plus 5 mm per outer face; project-lead decision of
  25 Oct 2026; not the complete battery assembly): the run places it on the deck, centred in y,
  front face 2 mm behind the rear face of the centre bracket (`placement_rule`), and reports
  both face positions, the clearances to the centre bracket, the top-cover supports, the cover
  underside, the rear sloped flange, the cover rear edge and the base plate, every chassis part
  within 60 mm, and the free space behind (BMS and power distribution zone, not modelled) and
  beside the envelope in `build/cad_checks.md` and `PROVENANCE.yaml components.battery.rear_deck_check`.
- Envelope shapes (25 Oct 2026): the key base envelopes follow their documented shapes, built from
  boxes and cylinders only: battery as eight LF105 cell boxes (130 x 38 x 200 mm, 1 mm gaps from the
  5 mm allowance) with terminal studs inside a translucent envelope shell (GLB only, `stl: false`);
  J401 carrier, Orin NX module and an illustrative finned heatsink block; NUCLEO-144 board with the
  two ST morpho header rows (UM2407); ZLAC8015D housing with base plate and fins (manual V1.03);
  RPLIDAR S3 base, head and window band; Gemini 336L bar with three lens windows; MB7060 with hex lock
  nut; TF-Luna body, ears and two lenses. Part options: `repeat` (written out as separate parts so
  that small boxes are not decimated), `feature` (counted in `PROVENANCE.yaml components.<id>.features`),
  `alpha`, `stl`, `sections` (6 for a hex) and `xyz_rel_mm` (relative to the battery envelope centre).
  Schematic details (fin count, lens and terminal diameters, header height) are marked as such in each
  `dimension_source`. No vendor geometry is imported; positions read from the STEP are stated.
- GLB node ids: chassis rules carry `node` or `nodes` (id, name, CAD part, BOM row); instances
  sharing an id get a position suffix (`_front`, `_rear`, `_left`, `_right`). Components carry
  `node_ids` per placement link.
