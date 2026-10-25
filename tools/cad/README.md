# CAD to description meshes

`step_to_description_meshes.py` regenerates the 2.0 base visual meshes of
`ros2/src/openamrobot_description` from the hardware CAD. The chassis comes from the CAD; the
components come from the 2.0 bill of materials (BOM Issue 7.11, 25 Oct 2026) and are simplified
envelopes; the battery position comes from B-02 Battery Guide rev18.8.

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
- Battery enclosure (provisional, 340 x 250 x 230 mm): the run recomputes the rear limit from
  the CAD (base plate rear tab and rear panel mounts at deck level), fails when the map's rear
  face no longer sits 5 mm ahead of it, and reports the clearances to the cover underside,
  supports, centre bracket, rear panel parts and rear castor mounts, with the largest box that
  fits when a clearance is negative (`build/cad_checks.md`).
- GLB node ids: chassis rules carry `node` or `nodes` (id, name, CAD part, BOM row); instances
  sharing an id get a position suffix (`_front`, `_rear`, `_left`, `_right`). Components carry
  `node_ids` per placement link.
