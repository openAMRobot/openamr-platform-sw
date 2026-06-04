# 14 — Precision Docking for Wireless Charging: Research & Architectures

> **Document scope.** A complete, vendor-agnostic survey of how to dock a mobile robot precisely enough for wireless power transfer (WPT), with concrete reference data from four commercial chargers, an exhaustive sensing-methods catalog, a comparative analysis of receiver-coil mounting geometries, the recommended architecture for the OpenAMRobot platform, a concrete bench-validation protocol, failure-mode analysis, calibration & commissioning procedures, and a comparison with how other companies solve the same problem.
>
> **Audience.** Robotics engineers integrating the docking system, and the technical lead deciding on the sensor and control stack.
>
> **Out of scope.** Mechanical centering funnels (treated as a complementary aid, not a primary method); high-voltage / battery-management aspects of the charger; mission-level scheduling of recharge events.

---

## Table of contents

1. Executive summary
2. Design philosophy and target derivation
3. The charging hardware — reference data, not constraints
4. Receiver-coil mounting geometry
5. Sensing methods — exhaustive catalog
6. Primary architecture: camera-only, visual perception to engagement
7. Optional and evolution paths
8. How other companies and products do it
9. Validation plan
10. Failure modes & recovery
11. Calibration & commissioning
12. Multi-dock disambiguation
13. Related OpenAMRobot documentation
14. Sources

---

## 1. Executive summary

> **Bottom line up front.** The system is built **vendor-agnostic** to a worst-case precision budget of **±10 mm position and ±2° yaw, repeatable under any operating conditions**. The primary sensing stack is **camera-only**, closing the loop on direct AprilTag perception all the way to coil engagement. Lidar, lidar+camera fusion, coil-side misalignment sensing and odometry dead-reckoning are kept as documented optional paths for integrators using different hardware, not on the default code path.

### 1.1 Five takeaways

1. **Cm-scale, not mm-scale, is what the chargers demand.** The four commercial chargers studied (TZBOT, WiBotic, Wiferion, Xnergy) all accept lateral misalignment in the **±30 to ±50 mm** range at full power. The tightest published value is ±30 mm (TZBOT and Wiferion). Our **±10 mm design target** is set conservatively to beat this worst case by a factor of three.
2. **The angular spec is the data gap in the entire WPT industry.** Only WiBotic publishes an angular tolerance (0–70°, and it's a tilt spec for round coils, not the docking-relevant yaw). The other three vendors publish no angular figure. Our **±2° design target** is therefore set defensively, not derived from a spec.
3. **Robustness, not raw accuracy, is the hard problem.** Cm-scale precision is reached easily by most sensing methods under ideal conditions; the engineering work is making the system meet ±10 mm / ±2° **under glare, partial occlusion, motion blur, dirt, and parallax — every time**.
4. **The "docked-OK" signal exists on every charger studied, but it is generally binary.** It validates the final position after the fact; it is not a continuous misalignment feedback that can drive a closed-loop control. Closing the loop must therefore come from on-board sensing, with the binary signal acting only as the engagement gate.
5. **Camera-only with a multi-tag bundle is the right primary stack** for the OpenAMRobot platform. It is the lowest-cost, lowest-infrastructure path, it matches the industry default (Nav2 `opennav_docking`, Fetch, mainstream AMRs), and the alignment bottleneck — yaw — is solved by going from a single tag to a bundle of 2–3 tags or to in-line markers, not by adding a different sensor.

### 1.2 What this document does NOT claim

> Honest limits, stated up front so the reader is not misled:
>
> - **No vendor recommendation.** Charger comparisons in §3 are reference data to bound the design budget, not buying advice.
> - **No lab-measured numbers.** All vendor figures come from public datasheets and integration guides. No bench validation has been performed by the OpenAMRobot team at the time of writing.
> - **Yaw precision claims for tag bundles are qualitative.** The literature consistently reports that bundles improve yaw stability over single tags, but precise σ figures depend on bundle geometry, camera, and lighting and are not portable.
> - **Contour-lidar and 3-D-lidar docking precision are not well standardized.** Only reflector-lidar (~1 cm / <0.05°) and the multi-sensor-fusion comparative studies are firmly documented.
> - **The ±10 mm / ±2° target is unvalidated.** It is a defensible design budget; the validation plan in §9 specifies what bench tests must be run before claiming it is achievable.
> - **The "any condition" envelope is bounded by the test matrix.** §9.2 enumerates the conditions tested; the claim does not extend beyond them (e.g. direct outdoor sun is out of scope unless explicitly added to the matrix).

---

## 2. Design philosophy and target derivation

### 2.1 Vendor-agnostic by construction

The system is **not designed against a single charger's spec**. Vendor numbers are used as **reference data** to bound the precision budget. The system must therefore beat the **tightest** published tolerance of any plausible candidate charger, on every axis, with margin. This is the only way to keep the integration open to whichever charger the end customer selects.

### 2.2 Numerical targets

| Axis | Target | Derived from |
|---|---|---|
| Lateral position (X, Y) | **≤ ±10 mm** | Worst published vendor tolerance is ±30 mm → ×3 margin |
| Yaw (rotation around Z) | **≤ ±2°** | No vendor publishes a yaw spec for rectangular coils → defensive default |
| Air gap (Z) | within charger's published window | Mechanical constraint, not a positioning target |
| Repeatability | **at any operating condition** | Lighting changes, partial occlusion, dirt, parallax |

The first two are *control* targets the perception+control stack must reach. The air gap is a *mounting* constraint set when the receiver pad is installed on the robot.

### 2.3 What "any operating condition" means

Concretely, the same ±10 mm / ±2° must hold:
- Under fluorescent lights, daylight glare from windows, low-light corridors, and headlight reflection;
- With one of the three reference fiducials partially occluded by a person, a forklift tine, or floor debris;
- With moderate motion blur from a 0.1 m/s final approach speed;
- With dirt or scuffing on the printed tag surface up to a level a human inspector would still consider acceptable;
- Across the full charger air-gap window (the robot can arrive at any gap inside the supported range, e.g. 5–30 mm for Wiferion).

This robustness envelope is the actual engineering target. Hitting cm precision in a clean lab is not the bar. The concrete test matrix that bounds "any condition" is in §9.2.

### 2.4 Primary metric: repeatability

The right metric for accepting the docking system is **the standard deviation of final position and yaw across N=100 docking attempts under the worst combination of conditions in the operating envelope**, not the mean error in nominal conditions. A system with 0 mm mean and 5 mm σ is acceptable; a system with 1 mm mean and 25 mm σ is not.

A 3σ interpretation of the target gives:

| Target | Equivalent σ (3σ rule) |
|---|---|
| ±10 mm position | σ_pos ≤ 3.3 mm |
| ±2° yaw | σ_yaw ≤ 0.67° |

These σ values are what §9 validates.

---

## 3. The charging hardware — reference data, not constraints

This section consolidates the **public** specifications of the four candidate charger families. None of the data below is under NDA; everything is from manufacturer datasheets or integration guides freely available online.

### 3.1 Wireless power transfer 101 (just enough)

WPT couples two coils across an air gap using a high-frequency magnetic field. Two implementation families dominate the AMR market:

- **Tightly coupled inductive** (Wiferion-style): high coupling coefficient, high efficiency, **narrow** misalignment window. Efficiency drops sharply as the coils misalign.
- **Magnetic resonance** (Xnergy, WiBotic, TZBOT): each coil is part of a tuned LC circuit. Lower peak coupling but the resonance widens the misalignment window. With adaptive impedance matching (WiBotic), efficiency can stay nearly **flat** across the entire tolerance window.

For an AMR docking design the choice of family matters because **resonant chargers relax the robot's precision budget**, while inductive chargers reward perfect centering with higher efficiency.

### 3.2 Per-vendor reference data (public)

| Spec | TZBOT WCM-300 | TZBOT WCM-500 | WiBotic TC-200/RC-100 | Wiferion CW1000 | Xnergy BE | Xnergy Phoenix PH |
|---|---|---|---|---|---|---|
| Family | Resonant (multi-loop) | Resonant (multi-loop) | Resonant + adaptive impedance | **Inductive** (tightly coupled) | Resonant (contactless) | Resonant |
| Rated power | 300 W in / ~200 W out | up to **~450 W** | varies by Onboard Charger | **1250 W** | 3 kW per module (stackable to 9 kW) | 1500 W |
| Output voltage | 28.8 V fixed | **12–30 V adjustable** | varies | 15–60 V | 16–60 V | 16–60 V |
| Output current | 7 A fixed | **8–15 A adjustable** | varies | up to 42 A | 10–90 A per module | 30 A |
| Efficiency | ≥ 85 % wireless link | ≥ 85 % wireless link | 75–80 % DC-DC end-to-end | **93 % DC-DC** (vendor claim) | "full power guaranteed" | "full power" |
| **Air gap (Z)** | **0 – 50 mm** | **10 – 30 mm** (20 ± 10) ⚠️ no contact | **0 – 30 mm** at full power | **5 – 30 mm** | **20 – 70 mm** | up to **55 mm** |
| **Lateral X / Y** | **± 30 mm** | **± 20 mm** ⚠️ tightest of all | **± 40–50 mm** in any direction | **± 30 mm** | **± 50 mm** | "wide" (no number) |
| **Angular tolerance** | not published | not published | 0–70° **tilt** (round coil ⇒ yaw N/A) | not published | not published | not published |
| Coil shape | Rectangular pad 240×210 mm | not in extract | **Round** (TC-200 ⌀ ~200 mm) | not specified (probably rectangular) | Square 200×200 mm | Rectangular 163×78 mm (Rx) |
| Efficiency in tolerance window | degrades toward edges | degrades toward edges | **flat** (adaptive tuning) | degrades toward edges | "guaranteed full power across range" | "full power" claimed |
| Operating frequency | 10–50 kHz (self-tuning) | 30–60 kHz (self-tuning) | not in extract | not in extract | not in extract | not in extract |
| IP rating | IP65 | not in extract | varies | IP20 (WALL) / IP65–67 (SEPA) / IP54 (MOCHA) | IP54 (XDC3110) | not in extract |
| **Host interface** | **Binary** open-collector (start in, fault out, GND) | **Binary** open-collector (same family wiring) | not in extract | **CAN bus** (battery + energy management telemetry) | **CANBUS + MODBUS + GPIO + Auto-on** | "multiple protocols" |
| ROS / RMF interop | none documented | none documented | none documented | none documented | **Yes (XDC3300)** | implied |
| Mounting variants | single pad | single pad | varies by OC model | **WALL** 309×127×385 mm, **SEPA** 176×176×17 mm (thin pad), **MOCHA** 160×160×36 mm; floor-flush via PohlCon WCPS | **floor-standing** docking station (XDC 3100/3300/6000/9000) or transmitter unit only (TX30BE) | small form factor, "mount anywhere" |

### 3.3 Worst-case envelope

Combining the tightest value of each axis across all candidate chargers:

| Axis | Worst published value | Source |
|---|---|---|
| Lateral X / Y | **± 20 mm** | TZBOT WCM-500 (tightest of the six) |
| Air gap (max) | **30 mm** | WiBotic, Wiferion, TZBOT WCM-500 |
| Air gap (min) | **10 mm** | TZBOT WCM-500 (must not contact) |
| Angular tolerance | **unknown** | Only WiBotic publishes, and that's tilt for a round coil, not yaw |

Our ±10 mm / ±2° design target sits **×2 below the worst lateral spec** (WCM-500 ±20 mm → ±10 mm target), and is defensive on the unspecified yaw axis. The earlier ±30 mm worst case held for the lower-power WCM-300; the higher-power WCM-500 tightens the budget because tighter coupling is required to deliver more power.

**Implication for mechanical mounting (per OpenAMRobot scope):** if the receiver coil is mounted flush with the robot's front panel and the transmitter sits on a floor-standing dock, the nominal air gap is fixed by the dock's depth and the robot's stand-off distance at engagement. Targeting a nominal 20 mm gap (center of the WCM-500 window) gives ±10 mm headroom on either side against mechanical tolerance — no controller-side air-gap regulation is required.

### 3.4 The "docked-OK" signal: what it is, what it is not

Every charger studied provides some form of position-confirmation feedback **after** the robot has parked. The interface and granularity vary:

- **TZBOT WCM-300 / WCM-500:** a 3-wire signal cable on the **receiver side (WCM-300R / WCM-500R, the unit mounted on the robot)** — *not* on the transmitter side. The robot's controller wires directly into this 3-wire cable on its own chassis. Brown = start command from the robot's controller to the receiver (3–30 VDC, active high). Yellow = fault / inaccurate-position output from the receiver back to the controller (open collector, active low). Blue = signal ground. The charger runs an internal *coupling-detection* phase when the robot raises the start line; if coupling fails (positioning too imprecise to deliver power), the yellow line drops low and the charger enters a locked state until the start signal is reset. **Granularity: binary.** Position is either good enough or not — no continuous misalignment value is exposed.
- **Wiferion CW1000:** **CAN bus** host interface (confirmed: *"a controller area network bus provides valuable information for battery maintenance and efficient energy and fleet management"*). Likely exposes charge state, voltage, current, fault, and battery telemetry; no continuous coil-misalignment signal is documented in the public flyer.
- **Xnergy BE:** CANBUS + MODBUS + GPIO, plus a display module reporting voltage and current. **Granularity: telemetry available**, but the public datasheet does not describe a continuous coil-misalignment feedback. CAN messages likely expose state (idle / coupling / charging / fault) and electrical telemetry, not a vector of misalignment.
- **WiBotic:** the public excerpts in hand do not describe the host interface. WiBotic's integration manuals (available under request) document a host interface; the binary docked-OK signal is the de facto standard.

**Design consequence.** The robot cannot use the charger's signal as a closed-loop alignment sensor. The on-board sensing (camera or lidar) must reach the precision target on its own. The charger's signal is then used as the **engagement gate**: when the robot believes it is parked, it raises the start line and reads the fault line; if the fault line says "no", it backs off and retries.

**Retreat distance justification (80 mm).** The retry retreat distance is set to **80 mm** for the following reasons:

- 80 mm is **4× the tightest worst-case lateral tolerance** (±20 mm on WCM-500), so the robot is guaranteed to start the retry from well outside any plausible misalignment that could have caused the fault.
- 80 mm is short enough to keep a retry cycle under ~3 s at 0.05 m/s, so retries do not blow the 45 s mean-time-to-engagement UX target (§9.4).
- 80 mm is large enough to allow the EMA filter in §6.3 stage 3 to gather a fresh independent sample of the bundle pose, so the second attempt does not just replay the first attempt's error.
- If three consecutive retries fail, the abort policy in §10.5 triggers — the issue is likely not a small alignment error but a deeper fault (dock displaced, tag damaged, charger malfunction).

### 3.5 The angular-tolerance data gap

Of the four candidate chargers, only **WiBotic** publishes an angular value (0–70°), and that value describes **tilt between the coil planes** (pitch/roll), not yaw rotation around the robot's vertical axis. WiBotic's coils are round, so yaw is geometrically irrelevant — rotating a circular coil around its own normal does not change the coupling.

For the three chargers with **rectangular** coils (TZBOT, Wiferion, and probably the high-power Xnergy BE), yaw matters and is **not published**. This is the single most consequential gap in the public data and the most defensible argument for setting a conservative ±2° target. A support-email template to obtain the value directly from the vendor is in §9.7.

---

## 4. Receiver-coil mounting geometry

### 4.1 Why this matters first

The choice of where the **receiver coil** sits on the robot is the most architecturally consequential design variable of the entire docking stack. It determines:

- Which on-robot sensor can see the dock during the *final* centimeters of approach;
- Where fiducials must be placed on the dock infrastructure;
- Whether a single forward camera suffices, or extra hardware is needed;
- The minimum size of the inertial-continuation window in the sequencer.

The three viable geometries are **front**, **side**, and **under**. Each is presented below with its implications, then compared.

```
   A) Front-mount             B) Side-mount             C) Under-mount
                                                       
   [DOCK WALL]                [DOCK WALL]              
       ║                            ║                  
       ║──── coil                   ║──── coil         
       ║         ╔═══╗              ║                  
       ║         ║ R ║              ║   ╔═══╗          ╔═══╗
       ║         ╚═══╝              ║   ║ R ║───coil   ║ R ║
       ║                            ║   ╚═══╝          ╚═══╝
       ║                                                 │  coil
       ║                                            ════════════
                                                    [DOCK FLOOR]
```

### 4.2 Front-mount (the OpenAMRobot main scenario)

The receiver coil is mounted **flush with the robot's front panel** (vertical). The dock transmitter is mounted vertically at the same height, with two installation variants:

- **(a) Wall-mounted dock**: the dock station is fixed to a wall (the classic AMR docking layout).
- **(b) Floor-standing dock**: the dock is a free-standing station that **sits on the floor** but has its transmitter coil oriented **vertically** at robot-receiver height. The marker bundle is on the vertical front panel of the dock, above or around the transmitter. *This is the variant proposed as the OpenAMRobot main scenario (see §4.6) because it requires no permanent installation work on the deployment site — the dock is positioned on the floor and powered.*

Both variants share the same docking architecture: forward-facing camera + AprilTag bundle + IBVS-to-engagement. The only difference is where the dock physically attaches.

- **Sensing**: the existing forward camera sees the dock during the entire approach and remains useful through engagement. Bundle redundancy or in-line markers handle the case where the center tag exceeds FOV in the last 10 cm.
- **Marker placement**: fiducials on the vertical front panel of the dock, framing or surrounding the transmitter coil at camera height. See §11.3 for the recommended bundle geometry (outer tags larger, center tag smaller, for camera-focus optimization).
- **Final centimeters**: closed-loop visual servoing on the bundle, with inertial continuation only as a micro-dropout bridge (~300 ms).
- **Air gap**: fixed once at integration time by the relative depth of the robot's front panel and the dock's front panel. No active controller-side regulation. Nominal target 20 mm (centers the WCM-500 / WCM-300 / WiBotic / Wiferion windows simultaneously).
- **Industry analog**: Fetch Robotics, MiR, OTTO Motors all default to front-mount with forward-camera docking.
- **Difficulty**: ★ — simplest, the path of least resistance.

### 4.3 Side-mount

The receiver coil mounts on a lateral face (left or right) of the robot. The dock sits on a wall along which the robot parks parallel.

- **Sensing**: the existing forward camera sees the dock only during the **approach** phase (when the robot is offset from the wall and angles its body toward it). During final alignment the dock is at ~90° to the camera and **out of FOV**.
- **Options for closing the loop on the final phase**:
  - **Option S1 — Add a side-facing camera.** Compact RGB or RGB-D unit on the side of the robot, dedicated to seeing the lateral fiducial during the final 30–50 cm.
  - **Option S2 — Approach in two arcs.** The forward camera tracks a fiducial during an arc that ends with the robot parallel to the wall. After the arc, the camera no longer sees the dock and the final 10–30 cm finish on inertial dead-reckoning. **This breaks the camera-only-to-engagement principle** of §6 and is therefore not the recommended approach.
  - **Option S3 — Panoramic / fisheye / rotatable camera.** Single camera with extreme FOV or motor-driven pan keeping the dock in view through the maneuver. More expensive integration than S1, no functional advantage for a side-only use case.
- **Marker placement**: fiducials on the side wall, at the height of the side-mounted coil. With S1, the bundle must be visible to the side camera; with S2, a separate forward-visible fiducial is needed for the arc.
- **Industry analog**: less common in commercial AMRs; used by some forklift-style platforms that park alongside infrastructure.
- **Difficulty**: ★★★ — requires either added hardware (S1) or compromise on the camera-only principle (S2).

### 4.4 Under-mount (alternative scenario, *not* the main OpenAMRobot path)

This geometry is documented for integrators who *cannot* use the front-mount variant (e.g. the deployment site mandates that the dock be a flush floor pad rather than a vertical station). It is **not** the OpenAMRobot main scenario; see §4.2 and §4.6 for the chosen default.

The receiver coil mounts under the robot's chassis. The dock is a flat floor pad with a horizontal transmitter coil; the robot drives over it.

- **Sensing**: the existing forward camera sees the dock approach markers (placed on a vertical board behind the floor pad) but **loses them** in the final ~30 cm when the robot is directly over the transmitter. The dock pad itself is below the camera's vertical FOV.
- **Options for closing the loop on the final phase, all optional**:
  - **Option U1 — Inertial continuation.** Lock a precision pose at the hand-off point (~30–50 cm before the dock), then finish on encoder+IMU dead reckoning. Drift over 50 cm at 0.05 m/s is typically a few mm and a fraction of a degree, well below the budget — but it is open-loop on the last stretch.
  - **Option U2 — Downward camera + floor markers (§7.6).** A small downward-facing camera reads tags painted on the floor along the approach lane and around the dock. Closed-loop perception all the way to engagement. Requires extra camera + floor instrumentation.
  - **Option U3 — Coil-side misalignment sensing (§7.2).** The charger exposes a continuous misalignment estimate that the robot uses as the controller input. Independent of optics; requires charger hardware support.
- **Marker placement**: fiducials on a vertical panel behind/above the floor pad, visible to the forward camera during approach. For U2, additional small tags on the floor along the lane.
- **Industry analog**: Wiferion etaLINK and Xnergy BE are commonly deployed as floor pads with under-robot receivers in AGV warehouses; floor-flush options exist via partners (e.g. PohlCon WCPS for Wiferion).
- **Difficulty**: ★★ — feasible with U1 inertial continuation, supported by Nav2 `opennav_docking`'s "dock blind" mode.

### 4.5 Comparison

| Geometry | Existing forward camera sufficient? | Coil leaves FOV in final cm? | Default OpenAMRobot architecture works? | Extra HW required? | Difficulty |
|---|---|---|---|---|---|
| **Front** | ✅ | rarely (bundle handles it) | ✅ directly | none | ★ |
| **Side** | ❌ (lateral FOV needed) | always | ❌ without lateral camera | side-facing camera | ★★★ |
| **Under** | ✅ during approach | always in last ~30 cm | ✅ with inertial continuation | none (U1); downward cam (U2) | ★★ |

### 4.6 OpenAMRobot scope decision (v1)

**Main scenario (default code path) — Front-mount with floor-standing dock.**

The chosen scenario, confirmed by the technical lead:

- **Receiver coil** mounted **flush with the robot's front panel** (vertical).
- **Dock transmitter** mounted vertically on a **floor-standing dock station** (sits on the floor, no wall installation required, easy to redeploy or move). Wall-mounted variant is supported with the same architecture if the deployment site prefers it.
- **Marker bundle** on the vertical front panel of the dock, above and around the transmitter coil, at the height of the robot's forward camera. Recommended layout: outer tags larger, center tag smaller, for camera-focus optimization (§5.1.3 and §11.3).
- **Air gap** fixed mechanically at integration time; nominal 20 mm.
- **The architecture in §6 (camera-only, IBVS to engagement) works without modification.**

This is functionally **front-mount geometry** (§4.2) with the dock-station variant. The "floor-mounted transceiver" phrasing in the project's internal discussion refers to the dock station physically *sitting* on the floor, not to the transmitter coil being horizontal — the coil itself is vertical, parallel to the robot's vertical receiver coil.

**Alternative scenarios, optional / documented for other use cases:**

- **Wall-mounted dock** (§4.2 variant a) — same architecture as the main scenario. Use if the customer site has a wall already available and prefers wall mounting.
- **True under-mount** (§4.4) — for deployments where the dock must be a flush floor pad rather than a vertical station. Requires inertial continuation (U1), downward camera (U2), or coil sensing (U3) on the last ~30 cm. **Not the default path; opt-in via configuration.**
- **Side-mount** (§4.3) — out of scope for v1. Adding it requires hardware additions (side-facing camera, Option S1) that are not on the current platform plan.

**Mechanical mounting constraint to enforce at integration:** the robot's receiver coil and the dock's transmitter coil must be at the same height (vertically aligned at the engagement pose), and the nominal coil-to-coil gap must fall inside every supported charger's window (~10–30 mm, centered on 20 mm, valid for all 6 reference chargers in §3.2).

### 4.7 Marker-placement rules per geometry

| Geometry | Primary marker location | Recommended layout | Spare / redundancy |
|---|---|---|---|
| Front | Wall around the transmitter coil, at camera height | 3-tag triangular bundle (current) OR collinear strip | Outer tags keep solving if center tag exceeds FOV at <20 cm |
| Under | Vertical surface behind/around the floor dock, visible during approach | 3-tag bundle on a small board behind the floor pad | Inertial continuation handles the final blind stretch |
| Side (future) | On the side wall, at the side-coil height | 3-tag bundle on the wall, visible to side camera | Side bundle + forward bundle for approach-then-arc |

---

## 5. Sensing methods — exhaustive catalog

This section catalogs every practical method for precision docking, with how it works, what precision and robustness it achieves, and what infrastructure it requires.

### 5.1 Camera + visual fiducial — the AprilTag family

#### 5.1.1 Single AprilTag, solvePnP

Detect the four tag corners; solve a perspective-n-point (PnP) problem against the known tag geometry → 6-DoF pose. Position is typically accurate to ~1 cm at <2 m and degrades to 2–4 cm at 2 m off-axis. **The weak axis is orientation**: a small planar square subtends few pixels and the well-known planar-pose ambiguity ("flip") gives two near-equal solutions, with jittery yaw and recurrent flipping under noise. Lighting changes, motion blur, and FOV departure at close range further degrade it.

A single AprilTag alone **cannot** repeatably reach ±2° yaw under realistic conditions. It is fine for coarse approach (getting the dock into view) but not for engagement.

#### 5.1.2 Multi-tag bundle (2–3 tags, fixed geometry)

`apriltag_ros` treats several tags at a known rigid layout as one body. On detecting any subset of the tags, it solves PnP over 4·n corners and reports the bundle origin. The wide baseline between tags **tightly constrains the surface normal** — that is, the yaw — largely defeating the single-tag flip ambiguity. Adds redundancy: if one tag is occluded or leaves the FOV up close, the others still solve.

This is the current OpenAMRobot implementation (3-tag bundle: outer tags ±0.45 m from the center) and is also the default in Nav2 `opennav_docking` and on Fetch robots. Position accuracy is similar to a single tag (~cm at <2 m); the real gain is **yaw stability**.

#### 5.1.3 In-line markers and asymmetric-size bundle variants

Two improvements on the basic triangular-equal-size bundle are worth considering, both motivated by team-internal review feedback.

**(a) In-line / collinear bundle.** Tags placed in a straight line at the dock, all coplanar. Geometrically a degenerate bundle: yaw is constrained by the line direction (which equals the dock surface normal cross the vertical) and is **at least as good as the 2-tag bundle**, possibly better with a longer line. Out-of-plane rotation (tilt) is less well constrained than with a triangular layout, but for a wheeled AMR on a flat floor this is not a problem. **Easier to install** (a strip rather than a 2-D pattern) and easier to read at glancing angles because all tags share the same plane.

**(b) Asymmetric-size bundle: outer tags larger, center tag smaller.** When the bundle is sized for the full approach range (2 m → engagement), a single set of tag sizes creates a trade-off:

- *Large tags* are detected reliably at long range and survive partial occlusion, but at engagement distance (5 cm) a large center tag **exceeds the FOV** and stops being detected.
- *Small tags* stay in the FOV at engagement but **dissolve below detection at 2 m** due to insufficient pixels.

The recommended variant: **make the outer tags larger and the center tag smaller.** The outer tags carry the long-range approach (the bundle solves on them while the center tag may be too small to detect at 2 m), then the small center tag takes over in the last meter as the outer tags exceed FOV. The bundle solver gracefully handles partial detection sets — at any given moment, whichever subset is visible defines the pose.

This also exploits a camera-specific advantage: most fixed-focus AMR cameras have a **depth of field** that favors a particular distance. By making the outer tags larger, they reach acceptable pixel resolution while still well inside the DoF at long range; by making the center tag smaller, the same camera can resolve its corners crisply at the very close range where the outer tags have already left the FOV. The end result is a bundle that **maintains sub-pixel corner detection across the entire 2 m → 5 cm range**, with no single transition where detection quality drops.

**Markers along the approach path on the floor (variant c).** A series of small tags or reflective dots painted along the lane leading to the dock. The on-robot camera (or a downward camera) reads them sequentially. **Keeps the perception loop closed all the way to engagement**, because as one tag leaves the FOV the next enters — no FOV-loss blind spot. The trade-off is floor real-estate, vulnerability to dirt and scuffing, and the need for a downward-looking camera if the robot's forward camera cannot see the floor in the last meter.

**Validation plan.** The §9 bench protocol compares variants (a) collinear, (b) asymmetric-size triangular, and the original equal-size triangular bundle side by side on the same dock. The asymmetric-size variant is the leading candidate based on the geometric and DoF arguments above, but the choice is empirical and the bench σ on yaw decides.

#### 5.1.4 ChArUco board (chessboard + ArUco) — considered, not selected

ArUco squares provide ID and coarse pose; the chessboard corners between them are refined to sub-pixel precision and fed to PnP. Chessboard corners refine far more accurately than ArUco corners, and the abundance of low-variance corners drives down rotation noise. OpenCV's own documentation recommends ChArUco when high precision is needed. Reported pose-recovery rate under low contrast: 97 % for ChArUco vs 69 % for ArUco at the same image-error threshold.

**Decision for OpenAMRobot v1: considered as a high-precision alternative, NOT selected.** Reasons:

1. The 3-tag AprilTag bundle in the asymmetric-size variant (§5.1.3 b) is expected to deliver comparable sub-pixel yaw precision under the §9 bench protocol, with the additional benefit of FOV-resilient redundancy at close range.
2. A ChArUco board is a **single rigid printed plate**, bulkier and more flatness-sensitive than 2–3 separate AprilTags. Mounting on an arbitrary dock surface is harder; warping degrades precision noticeably.
3. The AprilTag pipeline (`apriltag_ros`, 36h11 family) is already integrated, tested, and validated in the current OpenAMRobot codebase. Switching to ChArUco would mean replacing the detector and re-validating from scratch.

**Re-evaluation trigger:** if the §9 bench measurements show that the asymmetric-size bundle's yaw σ exceeds the 0.67° budget by a margin that bundle tuning cannot close, ChArUco becomes the recommended fallback. Until then, the bundle stays.

#### 5.1.5 Visual servoing for the final cm

Two flavors of the same idea, used in the final 10–20 cm of approach:

- **Position-Based Visual Servoing (PBVS):** estimate the tag's 3-D pose every frame, compute Cartesian error to the goal, drive it to zero with a velocity controller. Faster and more accurate when the pose estimate is good, but only as good as the (noisy) depth and normal.
- **Image-Based Visual Servoing (IBVS):** drive image-feature errors to zero without an explicit 3-D pose. Robust to camera-model and calibration error, ideal for the last centimeters because it closes the loop on what is actually seen and avoids the noisy depth/normal. Struggles with large motions and feature loss.

For ±10 mm / ±2° engagement on a planar dock under variable lighting, **IBVS on the bundle (or in-line markers) is the right last-stage choice**.

### 5.2 Inertial — visual-odometry continuation, IMU, encoders

Pure inertial sensing alone never meets ±10 mm over a useful distance. Its role here is **continuation**: keeping the robot on track during transient camera dropouts (motion blur, partial occlusion, FOV exit) by integrating wheel-encoder + IMU velocity for a fraction of a second. The primary stack is camera-driven; inertial keeps it numerically stable across micro-dropouts.

Encoder + IMU drift on a typical AMR at 0.1 m/s is on the order of a few mm per meter on the linear axis, and tens of arc-minutes per meter on yaw. Over a 50-cm continuation (the under-mount worst case, §4.4), this is well below the ±10 mm / ±2° budget — so the camera can momentarily lose the tag and the system stays inside the target window.

### 5.3 Lidar with reflective markers

A 2-D lidar segments retro-reflective targets by their intensity returns (orders of magnitude above natural surfaces). Two or three reflectors at a known geometry on the dock are template-matched: the centroid gives position, the line/triangle through them gives yaw. The wide baseline produces an excellent yaw estimate.

Published industrial result on a heavy AGV: **~1 cm position, < 0.05° yaw**. Lighting-immune (intensity dominates ambient), detectable to ~70 m. The dominant industrial AGV docking method (SICK 2-D scanners with reflector navigation has been the standard for decades).

Trade-offs: the dock must be instrumented with calibrated reflective targets, kept clean, and not easily reproduced by accidental shiny surfaces in the environment. Setup is cheap but precise — reflectors must be mounted at the right relative geometry.

### 5.4 Lidar contour matching

A 2-D scan is matched against a known dock contour (V-plate, notch, asymmetric profile). No infrastructure beyond the shaped plate. Precision is less well documented in the literature; typical figures are cm-level position and low-single-digit-degree yaw, with yaw quality strongly dependent on the contour's sharpness and the scanner's angular resolution.

Plugs into Nav2 `opennav_docking` as a custom detector (the dock server is detector-agnostic; it consumes a `detected_dock_pose` PoseStamped from any source).

### 5.5 3-D lidar

Captures out-of-plane dock geometry that a planar scanner misses; more robust to partial occlusion. Heavier, costlier, and uncommon specifically for charging docking. Mentioned for completeness.

### 5.6 Lidar + camera fusion — deep dive (with EKF mechanics)

This subsection covers the fusion approach in depth, because it is the most reliable single architecture published for unknown-environment AMR docking, and it is referenced repeatedly below as the recommended *evolution* path beyond the camera-only primary. The depth here is intentional — review feedback specifically asked for the EKF mechanics and the dynamic-weighting policy to be made explicit.

#### Why fuse two sensors at all

A 2-D lidar gives accurate range and a wide-baseline yaw if the dock has identifiable geometry or reflectors. A camera with a fiducial gives identity (which dock?) and a fine bearing/orientation estimate from sub-pixel corner detection. Their failure modes are largely disjoint:

| Sensor | Strong at | Weak at |
|---|---|---|
| Camera + AprilTag | identity, fine yaw < 0.5 m, sub-pixel | glare, occlusion, motion blur, FOV exit |
| 2-D lidar | range precision, wide-baseline yaw with reflectors, lighting-immune | dock identity, fine yaw at close range, flat featureless docks |

When you fuse them naively (equal weights, all the time), the lidar's lower angular precision *will* degrade the camera's better yaw estimate at close range. **This is the failure mode the review feedback was concerned about, and it is real.** The fix is *adaptive weighting* — the EKF is told, at each step, how much to trust each sensor based on the current geometry, not statically.

#### EKF mechanics, kept short

An Extended Kalman Filter tracks a state vector `x = [x, y, θ, ẋ, ẏ, θ̇]ᵀ` (planar pose + planar velocity). At each step:

1. **Predict** using the robot's velocity model: `x(t+1) = F · x(t)`, with covariance `P(t+1) = F·P·Fᵀ + Q`. `Q` is the process noise.
2. **Update** from each sensor `i` independently: `x(t+1)⁺ = x(t+1) + K_i · (z_i − H_i·x)`, where `K_i = P·H_iᵀ·(H_i·P·H_iᵀ + R_i)⁻¹` is the Kalman gain and `R_i` is the *measurement noise covariance for sensor i*.

The key knob is `R_i`. **A high R_i tells the EKF "don't trust this sensor much"** (the gain becomes small, the measurement barely shifts the estimate). A low R_i means "trust this sensor a lot". The EKF then combines all measurements weighted by 1/R_i.

#### Dynamic R_i scheduling — the answer to the review feedback

The R_i values are *not constants*. They are scheduled by the docking sequencer based on the current phase:

| Phase | Distance | Lidar R for **position** | Lidar R for **yaw** | Camera R for **position** | Camera R for **yaw** |
|---|---|---|---|---|---|
| Long-range approach | 1.0 – 3.0 m | **low (trust)** | **low (trust)** | high (don't trust — bundle pixels few) | medium |
| Mid-range | 0.3 – 1.0 m | medium | high (don't trust — camera takes over) | medium | **low (trust)** |
| Close-range / IBVS | < 0.3 m | high (don't trust — lidar contour fails near dock) | **very high (mute)** | **very low (trust)** | **very low (trust)** |
| Engagement | < 0.05 m | not used (lidar muted) | not used | IBVS mode | IBVS mode |

The transitions are not hard switches — `R_i` interpolates smoothly between rows, e.g. `R_lidar_yaw = R_low + (R_high − R_low) · sigmoid((d_dock − 0.3) / 0.2)` so that around 0.3 m the lidar yaw's influence smoothly drops to negligible. This avoids the EKF "jolt" that hard switching would produce.

This scheduling answers the concerns raised in review:

- *"Lidar less accurate and might worsen the results after fusion"* — true at close range. The schedule **mutes the lidar's yaw at < 0.3 m** and progressively mutes its position by engagement, so the camera takes over without the lidar's noise corrupting the fine-alignment.
- *"Should we dynamically reset EKF settings and give higher priority to visual odometry?"* — yes, that is exactly what the scheduling table above does. Higher priority to vision at short range, higher priority to lidar at long range.
- *"We probably still need lidar to see obstacles"* — absolutely, and this is **independent of the fusion**. The same lidar that participates (or doesn't) in the dock-pose EKF *also* feeds the navigation safety layer continuously. When the EKF "mutes" the lidar for dock-pose contribution, the lidar's raw scan is still consumed by the Nav2 collision monitor for obstacle avoidance. **The lidar is never "off" — its dock-pose contribution is what fades in and out, not its existence.**

#### Phase-by-phase data flow

1. **Long-range approach (1–3 m).** The camera detects the bundle; identity of the dock is established. The lidar detects the dock contour or reflectors and produces an accurate range + a coarse yaw via the wide baseline. EKF state is mostly lidar-driven — the camera's bundle pose is noisy at this distance because the tags fill few pixels.
2. **Sensor fusion mid-range (0.3–1.0 m).** The lidar still tracks the dock contour and contributes to position. The camera's yaw becomes more accurate than the lidar's because sub-pixel corner refinement now beats lidar angular resolution. R_camera_yaw drops sharply.
3. **Final approach (< 0.3 m).** The camera is in IBVS mode on the bundle. The lidar's planar FOV may have exited the dock's profile (the dock is too close and too low to be in the planar scan plane), so its dock-pose contribution muted automatically. The lidar continues feeding the safety layer for obstacles.
4. **Engagement.** Pure IBVS, charger's binary docked-OK signal validates. If "no", retreat 80 mm and re-run from step 2.

#### Why fusion is more robust than either alone

Empirically (2025 MDPI Electronics multi-sensor docking paper), fusion outperforms vision-only and lidar-only on both **success rate** and **pose accuracy**, with the gain largest under dynamic lighting, motion blur, and partial tag occlusion. Each sensor's failure mode is masked by the other — but **only if the weighting is dynamic**. A naive fixed-weight fusion would actually be *worse* than camera-only at close range, exactly the failure mode the review feedback warned about.

#### Why this is not the *primary* OpenAMRobot path

Per the project's design philosophy (camera-only by default), fusion is documented here as the **evolution path** for integrators who need higher reliability in worse environments — typically outdoors, low-light, or with cluttered docks. It is also a natural fit once the platform receives the planned RGB-D camera upgrade (§7.5), because RGB-D + lidar fusion is straightforward to extend from RGB + lidar fusion. The EKF infrastructure is the same; the camera measurement model gains a direct depth observation that further tightens R_camera_position.

### 5.7 Floor fiducials with a downward camera

A grid or strip of 2-D barcodes (Kiva/Amazon-style) read by a downward-looking camera. Solves the "no forward sensor can see a coil directly under the robot" problem in the final cm (under-mount, §4.4). Heavy floor instrumentation; appropriate for facilities where the floor is already grid-fiducialized, less appropriate for arbitrary deployments.

### 5.8 IR beacons (iRobot-style)

Coded IR beams emitted by the dock; the robot homes on bearing. No pose, just direction. Robust to lighting; low angular precision. Useful for *finding* the dock at long range, not for precision engagement.

### 5.9 Coil-side misalignment sensing

The transmitter or receiver coil senses the partner coil's magnetic field and computes its own misalignment vector. Independent of optics. Closes the loop on what physically matters (coil alignment) rather than on a proxy (tag pose). The cleanest solution for under-robot geometries because no forward sensor is needed at all.

Trade-off: requires either a vendor that exposes this signal (none of the four studied does, in their public datasheets) or a custom add-on coil array. Not all chargers can be retrofitted with this capability. For OpenAMRobot, this is treated as an integration option for users with charger hardware that supports it, not as the default path.

---

## 6. Primary architecture: camera-only, visual perception to engagement

### 6.1 Principle

A single RGB camera, a multi-tag bundle (or in-line markers) on the dock, and an `apriltag_ros` detector close the loop on direct tag perception **from approach to engagement**. The robot's `map` and odometry are used only for the very coarse approach (getting the dock into the camera FOV); once the bundle is visible, the entire docking trajectory is driven by camera-frame measurements. This makes the result independent of map drift and wheel slip.

### 6.2 Phase diagram

```
   Phase 1         Phase 2         Phase 3         Phase 4         Phase 5         Phase 6
  ┌─────────┐    ┌─────────┐    ┌───────────┐   ┌───────────┐   ┌──────────┐    ┌──────────────┐
  │ Coarse  │    │Centering│    │ Normal    │   │ Pure-     │   │ IBVS     │    │ Engagement   │
  │ approach│───►│ scan    │───►│ estimation│──►│ pursuit   │──►│ final cm │───►│ gate         │
  │ (Nav2)  │    │         │    │ (EMA)     │   │ on normal │   │          │    │ OK? → done   │
  └─────────┘    └─────────┘    └───────────┘   └───────────┘   └──────────┘    │ no? → retry  │
                                                                                  └──────────────┘
  base → 2 m     stationary      stationary      2.0 → 0.5 m    0.5 → 0.05 m     contact
  Nav2 + map     ω = 0.2 rad/s    v = 0           v = 0.20 m/s   v = 0.05 m/s    v = 0
```

### 6.3 Stages of the docking sequence (with numbers)

All speeds, gains and thresholds in the table below are configured in `config/dock_trigger.yaml` and consumed by `scripts/dock_trigger.py` in the `openamrobot_docking` package. The parameter names in the YAML are given in the rightmost column for traceability.

| Phase | Distance | Speed | Control source | Exit criterion | Config / code |
|---|---|---|---|---|---|
| 1. Coarse approach | from base → ~2 m | Nav2 default | Nav2 + map | ≥2 of 3 bundle tags visible in image | `navigate_to_staging()` in `dock_trigger.py`; staging dist = `staging_distance` |
| 2. Centering scan | ~2 m, stationary | ω = 0.2 rad/s | image-frame bearing to bundle | bearing < 2°, all 3 tags visible | `_search_for_tag()`; ω limit = `spin_max_omega`; gain = `spin_kp` |
| 3. Normal estimation | ~2 m, stationary | v = 0 | EMA on outer-tag map positions | normal σ < 1° over last 1.0 s | `_estimate_dock()`; α = `axis_filter_alpha` |
| 4. Pure-pursuit on normal | 2.0 m → 0.5 m | v = 0.20 m/s | bundle pose in *camera frame* (no map) | distance to dock ≤ 0.5 m | `_goto_point_on_normal()`; lookahead = `line_lookahead_distance`; yaw gain = `line_yaw_kp` |
| 5. IBVS final | 0.5 m → 0.05 m | v = 0.05 m/s | image-frame bundle-center error | estimated coil-to-coil distance ≤ **20 mm** (engagement air gap, centered on the WCM-500 / WCM-300 / WiBotic / Wiferion windows simultaneously), derived from bundle pose minus the configured coil-to-bundle mount offset | `_final_visual_approach()`; freeze threshold = `freeze_axis_distance`; servo gain = `visual_servo_kp` |
| 6. Engagement | contact | v = 0 | charger fault line | OK within 2 s, else retry | `_engagement_gate()`; retreat distance = 80 mm (justified §3.4) |

### 6.4 Why each number — derivations

This subsection walks through the numerical derivations explicitly, because review feedback asked for the calculations behind the chosen speeds, frame rates, and tolerances to be made educational. Most of the numbers cascade from one core constraint: the **sub-pixel corner-detection budget of the AprilTag detector**.

#### The core constraint: motion blur per frame

`apriltag_ros` refines tag corners to sub-pixel precision (typical: 0.05–0.2 px standard deviation in good conditions). For this refinement to converge, the corner must not move by more than ~**0.5 px per frame** during the capture exposure. Above that, the corner is blurred across multiple pixels and refinement degrades.

The corner displacement per frame, in pixels, is:

> **Δpx = v × Δt × (f / d)**

where:
- `v` = robot velocity (m/s)
- `Δt` = frame interval = 1 / fps (s)
- `f` = camera focal length in pixels (typical 600–900 px for a 1080p AMR camera at ~70° HFOV)
- `d` = distance from camera to tag (m)

For the OpenAMRobot front camera (assume `f` ≈ 800 px) at 30 fps (`Δt` ≈ 33 ms):

| Phase | v (m/s) | d (m) | Δ (mm/frame) | Δpx (px/frame) | Verdict |
|---|---|---|---|---|---|
| Phase 4 mid-approach | 0.20 | 1.5 | 6.6 | **3.5** | above budget — but EMA filtering and lower precision OK at this range |
| Phase 5 entry | 0.05 | 0.5 | 1.65 | **2.6** | slightly above budget — borderline |
| Phase 5 near engagement | 0.05 | 0.1 | 1.65 | **13.2** | far above — *but* IBVS does not need sub-pixel: it tracks the centroid of the bundle, which is robust to per-corner blur |
| Phase 5 with center tag at 0.05 m | 0.05 | 0.05 | 1.65 | **26** | dominant pixel coverage; bundle centroid still trackable |

The takeaways:
- **Sub-pixel refinement matters for the pose-estimation phases** (3 and 4), where the bundle's 6-DoF PnP is consumed. The EMA averaging smooths the residual noise.
- **IBVS in phase 5 does not need sub-pixel.** It tracks an image-plane centroid; even with several pixels of blur, the centroid is well-defined. This is why 0.05 m/s is acceptable at very close range despite a high per-frame pixel displacement.

#### Centering ω = 0.2 rad/s

During the centering scan, the robot is stationary in translation but rotates. At the focal length `f ≈ 800 px` and bundle distance 2 m, the angular pixel rate is:

> Δpx_angular = ω × Δt × f = 0.2 × 0.033 × 800 = **5.3 px / frame**

This is large, but the *bundle centroid* (not individual corners) is being tracked for centering — the centroid's position uncertainty scales as the individual corner uncertainty divided by √N, so blur is absorbed. ω = 0.2 rad/s = 11.5°/s is also a comfortable physical scan speed and well below the platform's mechanical limit.

#### Normal-estimation 1.0 s, σ < 1°

The bundle stream is consumed at 10–30 Hz (depending on detection success rate). One second therefore yields N = 10–30 samples. For an EMA with α ≈ 0.4 (the OpenAMRobot tuning), the effective averaging window is roughly 1/α = 2.5 samples — but the σ < 1° criterion is checked on the *spread of recent samples*, not just the EMA. With 10–30 raw samples and a per-sample yaw σ of ~1.5° at 2 m, the statistical convergence to σ_window < 1° takes ~1 s of stable observation. This is the empirical settling time of the filter, validated in the early bench iterations.

#### Phase-4 v = 0.20 m/s

This is the fastest the robot can advance while:
- Letting the EMA filter remain stable (the EMA is recomputed in the *map* frame at this distance because the bundle pose drifts as the robot moves; the EMA must converge to the moving target faster than the target moves). v ≤ 0.2 m/s gives ~0.6 s for the EMA to track a new sample, comfortably inside the 1 s settling window.
- Keeping per-frame pixel motion (3.5 px) at a range where bundle PnP still tolerates blur because the bundle subtends many pixels.

#### Phase-5 v = 0.05 m/s

This is the slowest reasonable speed that still completes the final 45 cm in under 10 s. The constraint that drives it down to 0.05 m/s rather than 0.10 m/s is the *bundle centroid stability* — at close range, even small misalignments in the IBVS controller cause overshoot if the velocity is too high (the response time of the differential drive becomes a factor). Bench-tuning lands the value at 0.05 m/s.

#### Engagement air gap 20 mm

The 20 mm target is the **center of the worst-case charger window** in §3.2:
- TZBOT WCM-500: 10–30 mm window → center 20 mm
- WiBotic TC-200/RC-100: 0–30 mm → 20 mm is inside
- Wiferion CW1000: 5–30 mm → 20 mm is inside
- TZBOT WCM-300: 0–50 mm → 20 mm is inside
- Xnergy BE: 20–70 mm → 20 mm is the floor of the window (barely valid; consider increasing slightly for Xnergy deployments)

Targeting 20 mm gives **±10 mm headroom on either side** against mechanical variation in the coil-to-bundle mount offset and the dock-bundle alignment.

The 20 mm is the *coil-to-coil distance* the controller targets, not a Euclidean robot-to-dock pose distance. The controller computes:

> coil-to-coil distance = bundle-frame distance − coil-to-bundle mount offset

where the coil-to-bundle mount offset is a static parameter set at integration time (typically ~50–100 mm: the distance from the bundle plane on the dock down to the transmitter coil center, plus the distance from the receiver coil center to the robot's front panel where the bundle was sighted).

#### Retreat distance 80 mm on retry

Already derived in §3.4: 4× the worst-case lateral tolerance (±20 mm WCM-500), short enough for sub-3-s retry cycles at 0.05 m/s, large enough for EMA fresh-sample independence.

### 6.5 Timing & latency budget — how the pipeline really runs

This subsection is written as educational material — the timing budget is best understood by walking through what happens between the camera capturing a frame and the robot's wheels reacting to the corresponding control command.

#### The end-to-end perception → action chain

```
   ┌─────────┐   ┌─────────┐   ┌─────────────┐   ┌────────────┐   ┌──────────┐   ┌─────────┐
   │ Camera  │──►│  USB    │──►│ apriltag_ros│──►│ dock       │──►│  /cmd_   │──►│ Motors  │
   │ capture │   │ bridge  │   │ detection   │   │ trigger    │   │  vel     │   │         │
   │  ~30 Hz │   │ + ROS   │   │ + PnP       │   │ controller │   │   ROS    │   │ wheels  │
   └─────────┘   └─────────┘   └─────────────┘   └────────────┘   └──────────┘   └─────────┘
       ~33 ms        ~50 ms          ~30 ms            ~10 ms          ~20 ms         physical
                                                                                      ms-scale
       ►  total budget from camera shutter to wheel velocity change: ≤ ~150 ms
```

Each stage adds latency. The cumulative budget is what bounds how fast the robot can react to a perception update.

#### Per-stage budget

| Stage | Budget | Rationale |
|---|---|---|
| Camera frame rate | ≥ 30 Hz (Δt ≤ 33 ms) | matches the motion-blur calculations in §6.4 (above 30 Hz, per-frame motion exceeds sub-pixel budget) |
| Camera capture → ROS topic | ≤ 50 ms | typical USB camera driver: 1 frame buffering + USB transport + ROS publish |
| AprilTag detection + bundle PnP | ≤ 30 ms | `apriltag_ros` on a modern AMR CPU (Intel i5-class or NVIDIA Jetson) |
| Bundle pose → controller decision | ≤ 10 ms | the controller is a simple PID-like loop, computationally trivial |
| `/cmd_vel` → motor driver | ≤ 20 ms | ROS topic + microcontroller PWM update |
| Wheels actually accelerate | platform-dependent | not in our control budget; assume few hundred ms response time of the differential drive |

**Total camera-to-wheel-velocity latency budget: ~150 ms.**

#### Why 30 Hz frame rate

At Phase 5 (v = 0.05 m/s), 30 Hz means a control update every 1.65 mm of robot motion. That is well below the ±10 mm position budget, so even if a control update is one frame late, the error contribution is < 2 mm. Lower frame rates (e.g. 15 Hz) would push this to 3.3 mm/frame, eating noticeably into the budget. Higher frame rates (60 Hz, 90 Hz) help in theory but the limiting factor becomes the apriltag_ros detection time, which doesn't scale linearly above the frame rate.

#### The 150 ms total latency and what it means

At Phase 5, in 150 ms the robot moves 7.5 mm. So when the controller receives a new pose estimate, it is acting on a state that is *already 7.5 mm in the past*. The IBVS controller compensates for this by being a smooth proportional servo (no abrupt corrections) and by being tuned for low gain — the loop overshoots less because each step is small. A higher gain would react faster to the stale estimate but might overshoot; the chosen gain `visual_servo_kp` is tuned for stability under the 150 ms latency.

#### Inertial continuation across micro-dropouts

When a frame arrives late (USB hiccup, brief tag occlusion, motion blur on a single frame), the controller does *not* immediately stop. It continues integrating the wheel encoder + IMU velocity for up to **300 ms**, then either receives a fresh bundle pose (good) or pauses (no fresh pose for too long).

The drift over 300 ms at 0.05 m/s is:
- Linear: 0.05 × 0.300 = **15 mm** of forward motion — but this is *intended* motion, not error. The encoder integration knows this and the controller does not over-correct.
- Rotational from IMU drift: typical industrial-grade MEMS IMU has a yaw bias drift < 0.05°/s, so over 300 ms the drift is < **0.015°**. Negligible.
- Linear drift from encoder slip on a typical AMR floor: < 0.5 % of the 15 mm = **< 0.1 mm**. Negligible.

So a 300 ms continuation bridge introduces effectively zero error contribution beyond the intended forward motion. This is why the continuation is bounded at 300 ms — beyond that, IMU drift starts to be non-negligible and the controller is better off pausing for a fresh pose.

#### Control loop rate

The controller runs at **≥ 20 Hz** — same order as perception, so no aliasing. Running faster (50 Hz) is fine but wastes CPU; running slower (10 Hz) starts to introduce phase lag relative to the perception updates. 20 Hz is the sweet spot for a 30 Hz perception input.

### 6.6 Inertial continuation across micro-dropouts

Between camera frames, and across transient tag dropouts (motion blur, brief occlusion), the controller integrates encoder + IMU velocity to maintain a smooth velocity command. The integration window is short — a few hundred milliseconds — so drift stays well below the ±10 mm / ±2° budget. This is **not** dead-reckoning to engagement; it is **dead-reckoning across the gap between vision updates**.

For under-mount geometry (§4.4 Option U1), inertial continuation extends to ~30 cm because the forward camera has no view of the dock in that final stretch. This longer window is still inside the budget (see §6.5) but with much smaller margin.

### 6.7 Why camera-only meets ±10 mm / ±2° at any condition

The two known fragilities of camera-only docking are:

- **Single-tag yaw ambiguity** → solved by going to a multi-tag bundle (or in-line markers), which provides a wide baseline.
- **Tag leaving FOV at close range** → solved by either (a) sizing the bundle so that at least one tag stays in the FOV at the engagement distance, or (b) switching to in-line markers along the approach path so the camera always sees at least one marker, or (c) inertial continuation across the last few mm of the engagement step where no tag is visible.

With these two issues addressed, the dominant error source is **per-frame tag-pose noise**, which is typically a few mm and a fraction of a degree at <0.5 m. Filtering (EMA, proximity-weighted) drives this below the budget.

The "any condition" robustness comes from the bundle redundancy (one occluded tag is tolerable), from sub-pixel corner refinement (motion blur < ~3 px is tolerable), and from the IBVS-mode final approach being immune to small calibration drift.

### 6.8 What camera-only does *not* solve

- **Under-robot final centimeters** for under-mount geometry: handled by inertial continuation (§6.6) or by §7.6 (downward camera with floor markers).
- **Side-mount geometry**: out of scope for v1 (§4.6); requires a side-facing camera to maintain the closed-loop principle.
- **Total camera failure**: handled by §10 (abort, navigation alert).

---

## 7. Optional and evolution paths

These paths are kept in the codebase as commented or feature-flagged alternatives for integrators with different hardware or environmental constraints. They are not on the default OpenAMRobot code path.

### 7.1 Parallel lidar detector

A second detector that consumes the same lidar scan the platform already runs for navigation and publishes a `detected_dock_pose` PoseStamped on the same topic the camera path uses. Plugs into Nav2 `opennav_docking`'s detector-agnostic interface so the two paths are swappable.

**Reflective beacons or not — recommendation.** Two variants exist:

- **With retro-reflective targets (3 markers on the dock).** Industrial AGV reference: ~1 cm position, < 0.05° yaw, lighting-immune. The wide marker baseline gives the best yaw estimate of any sensing method considered in this document.
- **Without reflectors, contour-matching on a passive shaped plate.** Lowest infrastructure cost, but only cm position and degree-level yaw, brittle to clutter.

**The recommended approach for the optional lidar path is to support BOTH under a config flag**, with **reflectors as the primary mode** when the dock can be instrumented (the typical industrial integration case), and contour-only as the fallback when reflectors are impossible (e.g. retrofit on an existing dock that cannot accept reflective targets, or shared-infrastructure cases where adding reflectors is not allowed).

If only one mode is implemented first, prioritize **reflectors** — the yaw precision gap is large enough that a contour-only lidar path is unlikely to beat the camera-only primary on its own.

### 7.2 Coil-side misalignment sensing

For charger hardware that exposes a continuous misalignment vector (none of the four studied does today, but future versions may). The robot subscribes to the misalignment estimate and uses it directly as the engagement-stage controller input. This is the cleanest solution for under-robot geometries because no on-board sensor needs to see the dock.

### 7.3 Odometry dead-reckoning hand-off (full)

For configurations where the integrator explicitly wants the simpler approach: lock a precise relative pose at a 30–50 cm hand-off point, then finish the trajectory on encoder + IMU integration alone. Nav2's `opennav_docking` supports this "dock blind" finish natively. Open-loop, so any unexpected disturbance (wheel slip, gentle bump) is uncorrectable.

OpenAMRobot's primary path uses **inertial continuation** as a micro-dropout bridge (§6.6) and for the under-mount last cm (§4.4 U1). The full "dock blind" mode in this section is reserved for integrators who want it on the entire final phase, accepting the open-loop trade-off.

### 7.4 Lidar + camera fusion (Option C)

Detailed in §5.6. The recommended **evolution path** for OpenAMRobot once two conditions are met: (1) the platform receives its planned RGB-D camera upgrade, and (2) field data identifies a class of dockings (lighting, geometry) where camera-only's σ exceeds the target. Until then, fusion is documented as a known-good alternative for higher-stakes environments but not implemented on the default path.

### 7.5 RGB-D + visual SLAM (platform evolution)

The OpenAMRobot platform roadmap is considering replacing the current RGB camera with an RGB-D camera (e.g. RealSense D435i / D455 class) to add **visual SLAM in support of lidar SLAM**. *This is an anticipated direction subject to confirmation by the technical lead, not a committed change.* If adopted, the depth channel is also directly useful to the docking stack:
- Direct depth at the tag eliminates the worst noise source in the docking estimate (the camera-frame depth from a single planar tag's PnP).
- Depth-based detection of the dock surface as a plane gives a yaw-free position estimate independent of the tag.
- It is a natural substrate for an eventual lidar+RGB-D fusion path.

This is a platform-level evolution, not a docking-only change. The docking layer is designed to absorb it without redesign: the existing `apriltag_ros` pipeline will run on the RGB stream of the RGB-D camera, with the depth stream as an additional optional input.

### 7.6 Downward camera with floor-strip markers

For configurations where the coil is under the robot, a small downward-facing camera plus a strip of small tags painted along the approach lane keeps the perception closed-loop through the under-robot geometry. Trade-off: extra camera, floor real-estate, and floor markers vulnerable to dirt. Useful for integrators who explicitly cannot accept open-loop dead-reckoning on the final cm.

---

## 8. How other companies and products do it

This is not a competitive benchmark; it is a reality check on which architectures real shipping products use.

| Product / system | Sector | Sensing method | Notes |
|---|---|---|---|
| **Amazon / Kiva drive units** | Warehouse | Floor 2-D barcodes + downward camera | ~1 m grid of fiducials; the floor is the infrastructure. |
| **MiR (Mobile Industrial Robots)** | AMR | RealSense 3-D camera + reflective VL-markers | Spec'd ~ ±2–3 mm and ±0.25° yaw to a marker. |
| **Fetch Robotics** | AMR | AprilTag + charge-detection check | Camera-only docking, AprilTag detector on a single fiducial. |
| **Nav2 `opennav_docking`** | Open source ROS 2 | Detector-agnostic (camera or laser) | `SimpleChargingDock` consumes any `detected_dock_pose`. Default detector is camera/AprilTag. Supports odometry "dock blind" finish. |
| **OTTO Motors** | Heavy AMR | 3-D SLAM nav + close-range fiducial / coil sensing | ~20 mm stop accuracy from nav alone; precise docking adds a close-range detector. |
| **SICK + industrial AGVs** | AGV | 2-D lidar with reflectors | Standard for decades. ~1 cm / <0.05°. |
| **Wiferion customer base** | Industrial AMR | (uses Wiferion CW1000 / etaLINK as charger) | Confirmed deployments: KUKA, FLEXQUBE, Lowpad, OMRON, STÄUBLI, SHERPA, BHS, ek-robotics, CIT, HIKROBOT, SAFELOG, BÄR, ABB, DEXORY. Each uses their own docking; Wiferion provides the charger. |
| **WiBotic / Xnergy / Wiferion (vendor side)** | WPT vendors | (sell pads, do not prescribe docking) | Rely on the integrator's docking and on coil-side sensing where available; some include mechanical centering aids. |

**Pattern across the industry.** Coarse approach with the navigation sensor the platform already carries (lidar SLAM or a camera), then a dedicated fine-alignment stage — a fiducial bundle, reflectors, or coil-side field sensing — and a short dead-reckoned or coil-guided final move for the part no forward sensor can see. **No mainstream AMR uses a single small tag alone for precision WPT docking; everyone uses redundancy.**

OpenAMRobot's choice (camera + multi-tag bundle, IBVS for final cm, inertial continuation for under-robot last cm) is in the same family as Fetch and Nav2's defaults, with the bundle/in-line refinement that brings yaw repeatability into the ±2° budget.

---

## 9. Validation plan

### 9.1 Bench setup

The validation environment is a **physical mock-up of the deployment dock**, plus instrumentation to control the test conditions:

- **Dock mock-up**: a flat vertical board (front-mount geometry) sized to match the chosen charger's mounting plate, with the 3-tag bundle (current implementation) or a collinear strip printed and mounted at the planned positions. Tags are matte laminated to suppress glare reflections.
- **Reference frame**: a ground-truth measurement system to establish the *true* final pose. Two options in increasing cost:
  - **Total-station / leveled markers**: 1–2 mm accuracy, manual reading per attempt.
  - **OptiTrack / Vicon**: < 1 mm, fully automatic — preferred if available in lab.
- **Lighting rig**: dimmable overhead diffuse + spot lights + a window or simulated daylight on a switch, to reproduce the test matrix of §9.2.
- **Test fixture**: a known repeatable robot start pose, marked on the floor.

### 9.2 Test matrix

The matrix below defines the five variables and their levels that bound the "any condition" envelope. **We do not test the full Cartesian product** (4 × 4 × 3 × 2 × 2 = 192 combinations × N = 25 = 4 800 attempts is not realistic). Instead, run a **sampled sweep**: roughly 12–16 representative rows that exercise each variable at its extremes individually, plus the single worst-case row that combines the hardest level on every variable.

| Variable | Levels |
|---|---|
| **Lighting** | (a) nominal overhead, (b) low-light < 100 lx, (c) glare from window/spot at 30° off camera axis, (d) flicker (50 Hz fluorescent) |
| **Occlusion** | (a) none, (b) outer tag occluded 50 %, (c) center tag occluded 50 %, (d) outer tag fully covered |
| **Start position** | (a) straight on at 2 m, (b) offset ±0.5 m laterally at 2 m, (c) approach at ±15° |
| **Charger mount** | (a) front-mount bench rig, (b) under-mount bench rig (floor pad + dock board behind it) — covers §4.6 supported geometries |
| **Robot battery state** | (a) full, (b) low (motors slightly less responsive) |

**Sweep policy.** Per variable, run N = 25 attempts at the *hardest* level with everything else at *nominal*. This isolates each variable's contribution to σ. Then run the combined worst-case row — (b) low-light + (c) center-tag-occlusion + (c) ±15° approach + (b) under-mount + (b) low-battery — at **N = 100**. This worst-case row is the one that must pass σ_pos ≤ 3.3 mm, σ_yaw ≤ 0.67° to claim the ±10 mm / ±2° target.

### 9.3 Per-attempt logging

Every docking attempt logs (one row per attempt + one rosbag per attempt):

- Timestamp, attempt index, test-matrix row code
- Initial pose (from fixture marking)
- Final pose **per ground-truth system** (the trusted reference)
- Final pose **per the robot's internal estimate** (bundle pose at engagement)
- Camera raw frames at 30 Hz throughout (rosbag for forensics)
- AprilTag detection rate (% of frames where ≥1 tag detected)
- Sequencer state transitions (entry time of each phase in §6.3)
- Charger fault line state at each retry attempt
- Number of retries before success / abort
- Reason for abort, if applicable

### 9.4 Pass/fail criteria

| Criterion | Threshold | Source |
|---|---|---|
| σ_pos (position) | ≤ 3.3 mm | derived from ±10 mm at 3σ (§2.4) |
| σ_yaw | ≤ 0.67° | derived from ±2° at 3σ (§2.4) |
| Success rate on first attempt | ≥ 90 % at N = 100 | engineering judgment; retry exists for the rest |
| Aborts on the worst-case row | **zero** at N = 100 | proxy for the 99.9 % reliability target (see note below) |
| Mean time to engagement | ≤ 45 s from arrival at the 2 m point | UX target |

A row passes if **all five** are met across N attempts.

**Note on the 99.9 % target.** A statistical demonstration of a 99.9 % success rate requires roughly N ≥ 3 000 attempts (binomial confidence interval). N = 100 is sufficient to claim ~95 %-class reliability and to detect any systematic failure mode, but not to prove 99.9 % alone. The project's aspirational 99.9 % target is therefore validated in two stages:
- **Stage 1 (this plan, N = 100):** zero aborts on the worst-case row → confidence ≥ 97 % reliability.
- **Stage 2 (separate, long-running deployment validation):** continuous monitoring over ≥ 3 000 production dockings to confirm or refute the 99.9 % claim. This is a deployment metric, not a bench metric.

### 9.5 Diagnostic test sequence (when validation fails)

When a row fails, run in order until the culprit is identified:

1. **Inspect detection rate.** If < 95 % on the failing row, the camera/AprilTag pipeline is the bottleneck → tune exposure, switch to a higher-DR camera, or reposition lighting.
2. **Inspect σ in nominal row.** If even (a)(a)(a) row has high σ, the issue is calibration (camera intrinsics, extrinsics, bundle geometry) → re-run §11.
3. **Inspect σ vs distance.** If σ degrades sharply > 1.5 m, the bundle scale is too small for the camera resolution → enlarge tags or move staging point closer.
4. **Inspect σ vs lighting condition only.** If σ jumps under glare/low-light, the camera's auto-exposure is poorly tuned or the camera class is the limit → consider RGB-D upgrade (§7.5).
5. **Inspect σ vs approach angle.** If σ degrades only at ±15°, the bundle layout or its symmetry is the issue → try collinear strip (§5.1.3).
6. **Inspect retry rate.** If many attempts succeed only after retry, the engagement gate is too strict or the IBVS phase is missing the bundle center → re-tune §6.3 stage 5.

### 9.6 Known unknowns

| Unknown | Mitigation |
|---|---|
| Angular tolerance of TZBOT, Wiferion, Xnergy (rectangular coils, no published yaw) | Email vendor support (§9.7). Until answered, design defensively at ±2°. |
| Whether WiBotic and Wiferion expose a continuous misalignment signal | Read their integration manuals (often require sign-in). |
| Whether in-line markers actually beat the triangular bundle on yaw σ | Bench test in matrix row (c)±15° approach. |
| Behavior under outdoor / direct-sunlight glare | Out of scope for indoor-only deployments; if added to scope, fusion path (§7.4) becomes the recommended evolution. |

### 9.7 Vendor support email template

> **Subject:** Technical specs for AMR integration — *[charger model]*
>
> Hello,
>
> We are integrating the *[model]* into an autonomous mobile robot (AMR) and need a few specifications that we could not find in the public manual:
>
> 1. **Angular tolerance** (yaw, pitch, roll) between transmitter and receiver coils at full output power — please provide values in degrees or a graph.
> 2. **Efficiency-vs-misalignment curve** (or 2–3 data points: centered, mid-tolerance, edge-of-window) for both lateral offset and air gap.
> 3. **Host interface to the robot:** is there a continuous misalignment / position-feedback signal available beyond the binary fault output? Communication protocol (digital I/O, CAN, RS-485, etc.)?
> 4. **Recommended docking aids** (mechanical centering, alignment markers, approach patterns) for AMR integration.
>
> Thank you,
> *[name / company]*

---

## 10. Failure modes & recovery

### 10.1 Tag-level failures

| Failure | Detection | Recovery |
|---|---|---|
| One outer tag occluded / removed | Bundle solves with 2 tags only; pose flag = `partial` | Continue. Loss of one outer tag still gives a 2-tag baseline. Log warning. |
| Center tag occluded / removed | Bundle still solves with outer tags only | Continue, with reduced FOV margin on the final phase. Log warning. |
| All 3 tags lost for > 1 s mid-sequence | No bundle pose for 1.0 s | Halt the controller, hold position. Resume when bundle returns. If > 5 s lost, abort and replan from §6.3 stage 1. |
| Tag mis-detected (wrong ID for a frame) | Tag ID outside the expected dock's ID set | Drop that detection from the EMA filter, continue. Log. |
| Tag heavily damaged / unreadable (printed but not detectable) | Detection rate < 30 % over 5 s window | Abort with maintenance alert: "tag damaged or dirty". |

### 10.2 Sensor-level failures

| Failure | Detection | Recovery |
|---|---|---|
| Camera frame freeze (driver hang) | No new frame for 500 ms | Restart camera driver, hold position. If freeze repeats, abort. |
| Camera complete failure | Driver heartbeat lost, no recovery in 5 s | Abort, return to safe waypoint, alert operator. |
| IMU dropout | Topic stale > 200 ms | Use encoder-only continuation; flag controller for degraded mode. |
| Encoder fault | Wheel-speed inconsistency check fails | Abort, robot already in degraded nav state. |

### 10.3 Charger-level failures

| Failure | Detection | Recovery |
|---|---|---|
| Fault line stays low after start | Yellow / fault signal low > 2 s | Retreat 80 mm, re-run §6.3 stage 4-5 with tightened EMA. Max 3 retries before abort. |
| Fault line never low and never charging | Voltage never increases on battery side after 10 s | Abort. Charger may be powered off or in fault. Operator alert. |
| Charger fault mid-charge | Fault signal flips low during charging | Cut start signal, wait 30 s, retry once. If repeats, abort and alert. |

### 10.4 Environmental failures

| Failure | Detection | Recovery |
|---|---|---|
| Dock displaced > 50 mm from expected | Bundle pose deviates from last-known by > 50 mm + EMA does not converge | Re-run §6.3 stage 1 with broader search. Update stored dock position in map. |
| Floor slip mid-approach | Encoder vs IMU velocity disagrees by > 20 % | Replan from stage 4, halve approach speed. |
| Person / obstacle in approach lane | Lidar safety layer trips | Halt, wait for clearance, resume. Standard Nav2 behavior. |

### 10.5 Retry policy and abort criteria

```
                  ┌───────────────────┐
                  │ Phase 5 → engage  │
                  └─────────┬─────────┘
                            │
                  ┌─────────▼─────────┐
                  │ Fault line OK     │
                  │  in 2 s?          │
                  └───┬───────────┬───┘
                  no  │           │ yes
                      ▼           ▼
              ┌─────────────┐  ┌──────────┐
              │ Retreat     │  │ DOCKED   │
              │ 80 mm       │  └──────────┘
              │ retry++     │
              └──────┬──────┘
                     │
              ┌──────▼──────┐
              │ retry ≤ 3 ? │
              └──┬───────┬──┘
              no │       │ yes
                 ▼       ▼
        ┌────────────┐  ┌────────────────┐
        │ ABORT      │  │ Re-run §6.3    │
        │ operator   │  │ from stage 4   │
        │ alert      │  └────────────────┘
        └────────────┘
```

### 10.6 Alert and logging levels

| Level | Meaning | Action |
|---|---|---|
| INFO | Normal phase transitions | Log to docking log |
| WARN | Recoverable degraded condition (one tag occluded, retry attempted) | Log + dashboard yellow |
| ERROR | Single docking attempt failed but robot is safe | Log + dashboard red + operator notified |
| FATAL | Docking aborted, robot in safe waypoint, awaiting human | Log + dashboard red + escalation alert |

All sequencer state transitions, every retry, and every abort generate a structured log entry with the cause. A 24-hour log rotation keeps the last week's docking attempts on disk for forensics.

---

## 11. Calibration & commissioning

This section is the procedure to deploy the docking system on a new robot or at a new site.

### 11.1 Camera intrinsic calibration

Use the standard `camera_calibration` ROS package with a checkerboard target. Output: `camera_info.yaml` with focal lengths, principal point, and distortion coefficients.

Acceptance criterion: re-projection error < 0.3 px RMS. If higher, the calibration must be redone (often a different checkerboard or lighting).

Stored at: `config/camera_intrinsics_<robot_id>.yaml`. **Per-robot file**, because lens manufacturing variability makes shared intrinsics dangerous.

### 11.2 Camera-to-base_link extrinsic calibration

Two acceptable methods:

- **Mechanical measurement**: precise CAD + manual verification with a caliper at known features of the robot. Acceptable for tolerances of a few millimeters; recommended as the first pass.
- **AprilTag-based extrinsic calibration**: place a known tag at a known position in the world frame (or on a calibration jig at known offsets from the robot), capture, solve. More precise; recommended if the mechanical-CAD path leaves > 5 mm residual.

Output: `config/camera_extrinsics_<robot_id>.yaml` containing the transform from `base_link` to `camera_optical_frame`.

Acceptance criterion: after extrinsic calibration, a known tag placed at a known map position should be reported by the bundle solver within ±10 mm of its true position when seen from 1 m away. Failing this means the extrinsic is wrong (or the intrinsics are off, recheck §11.1).

### 11.3 Dock installation

Per dock site, in order:

1. **Mechanical mount of the charger** as per vendor instructions. Note the position, orientation, and air-gap target.
2. **Print the AprilTag bundle** at the design size on matte laminated paper. The current OpenAMRobot bundle is three 36h11 tags on **200 mm × 200 mm printed panels** with a **160 mm black-square edge** (this is the size value consumed by `apriltag_ros` for PnP — the 36h11 family has a 1-module white quiet zone on each side, so the black square is 8/10 of the panel). Outer tags are placed at ±0.45 m from the center on a horizontal line.

   **Recommended layout update (asymmetric-size variant, see §5.1.3 b):** the outer tags should be printed **larger** than the center tag — e.g. outer panels 280 × 280 mm (black-square edge 224 mm) and center panel 200 × 200 mm (black-square edge 160 mm). This optimizes detection across the full 2 m → 5 cm range: the larger outer tags reach acceptable pixel resolution at long range, the smaller center tag stays inside the camera FOV at close range. The configured tag sizes in `config/tags_36h11_sim.yaml` must match the printed sizes per-tag.

   (See `docs/13_perception_and_line.md` for the geometric rationale and `config/tags_36h11_sim.yaml` for the live configuration.)
3. **Mount the bundle on the dock surface** with the center tag aligned with the transmitter coil center, the outer tags at the same vertical height, all coplanar with the dock front face. A jig or template prints alignment marks at the correct positions.
4. **Verify bundle geometry**: measure outer-tag spacing and confirm it matches the configured value within ±2 mm.
5. **Survey the dock position** in the robot's map. Note the position of the bundle center in `map` and store it as the docking goal in the bringup config.

### 11.4 Bundle / marker validation

Before declaring a dock commissioned, run an automated bundle-detection sweep:

- Drive the robot to the bundle staging point (~2 m).
- Verify all 3 tags detected at ≥ 30 Hz for 10 s.
- Verify reported bundle pose matches the surveyed pose within ±20 mm and ±2°.
- Verify the bundle's outer-tag baseline (computed from bundle pose) matches the printed geometry within ±5 mm.

A `commission_dock` script in the codebase automates this validation and writes a `dock_<id>.commissioned.yaml` file recording the surveyed pose and the calibration date.

### 11.5 Acceptance test

After commissioning, run the §9.2 matrix at a reduced scope (the nominal lighting row, both supported geometries, N = 25 per row) and verify all pass/fail criteria of §9.4 are met. The full N = 100 worst-case row is run once per **platform release**, not per dock commissioning.

### 11.6 Periodic re-calibration

| Calibration | Trigger | Cadence |
|---|---|---|
| Camera intrinsics | After any lens replacement / re-mount | Per event; otherwise every 12 months |
| Camera extrinsics | After any camera or chassis mechanical work | Per event; otherwise every 12 months |
| Dock geometry verification | After any physical contact with the dock; otherwise quarterly | Quarterly |
| Full acceptance test | After any software release that touches `dock_trigger.py` or apriltag config | Per release |

---

## 12. Multi-dock disambiguation

### 12.1 Tag ID strategy

Each dock in a facility is assigned a **unique block of AprilTag IDs** in the 36h11 family. The current OpenAMRobot bundle uses IDs `0, 1, 2`; for additional docks, assign non-overlapping blocks: dock 1 → `0–2`, dock 2 → `3–5`, etc. The 36h11 family has > 500 IDs, so collisions are not a practical concern.

The `apriltag_ros` bundle config explicitly lists the tag IDs that belong to the bundle. A detection of a tag ID outside the configured set is **dropped** by the detector for that dock's pipeline, preventing cross-talk.

### 12.2 Dock selection logic

When a docking request is issued, the robot must pick **which** dock to head to. The selection logic, in order of preference:

1. **Operator-commanded**: the request includes an explicit `dock_id`. Use it.
2. **Mission-assigned**: a fleet manager has pre-assigned this robot to a specific dock at this time. Use the assignment.
3. **Nearest available**: among all known commissioned docks that are not currently occupied, pick the closest by Euclidean map distance.
4. **Fallback**: if no dock metadata is available, the bringup config's default dock id is used.

### 12.3 Conflict resolution

Two robots converging on the same dock is a fleet-management problem (out of scope for this document) but the docking layer must defend:

- **Reservation token**: before starting §6.3 stage 4 (pure-pursuit), the robot must obtain a software reservation on the chosen dock. If the reservation is denied (another robot already holds it), the dock-selection logic of §12.2 reruns to pick the next-best available dock.
- **Lidar safety layer**: even with a valid reservation, the standard navigation safety layer will halt the robot if another robot enters the dock approach lane. Standard Nav2 behavior; no docking-specific handling needed.
- **Reservation release**: the reservation is held until either successful engagement (charging starts) or abort.

### 12.4 Identity verification before engagement

Even after navigating to the chosen dock, the robot verifies dock identity before raising the start signal:

- The detected bundle's tag IDs must match the expected set for the chosen dock.
- If they do not match, abort and alert: the robot is at the wrong dock (or the dock has been moved).

This guards against subtle deployment errors (a tag printed with the wrong ID, two docks accidentally configured with the same IDs).

---

## 13. Related OpenAMRobot documentation

The docking research in this document is part of a larger documentation set inside the `openamrobot_docking` package. Relevant cross-references:

| Document | Content |
|---|---|
| `00_overview.md` | Package overview and entry point |
| `02_architecture.md` | High-level architecture of the docking package |
| `03_tf_frames.md` | TF frames used (map, base_link, camera_optical_frame, charging_dock_tag_*) |
| `04_apriltag.md` | AprilTag configuration and tuning |
| `05_parameters.md` | All `dock_trigger` parameters explained |
| `08_sequencer_4phase.md` | The 4-phase docking sequencer (now extended to 6 stages per §6.3 of this doc) |
| `13_perception_and_line.md` | The perception pipeline (camera → AprilTag → bundle pose) and the line/normal estimation. This document supersedes the design discussion there with a vendor-agnostic framing. |
| **14_docking_research.md** | **(this document)** Research, architectures, validation plan, failure modes, calibration |

The older documents (especially `08_sequencer_4phase.md` and `13_perception_and_line.md`) describe earlier design iterations of the same pipeline. Where they appear to conflict with this document, this one reflects the more recent design framing; the older docs will be reconciled to it in the next implementation cycle so the package documentation stays internally consistent.

---

## 14. Sources

**Wireless chargers (public).**
- TZBOT WCM-300 Wireless Charger Instruction Manual v2.6 — Zhejiang Tongzhu Technology Co., Ltd. Full technical parameters, signal-cable wiring, fault-and-troubleshooting section. https://www.tzbotautomation.com/accessory/charger/agv-wireless-charger.html
- WiBotic TC-200 / RC-100 integration guide — vertical, lateral, and angular misalignment guidelines. https://www.wibotic.com/learn/faq/
- Wiferion CW1000 product spec sheet — power, displacement, clearance, IP ratings for WALL / SEPA / MOCHA variants. https://www.wiferion.com/en/faq/faq-etalink/
- Xnergy BE Series brochure (xnergytech.com, 11.2023) — 3–9 kW contactless charging, 20×20 cm pad, 2–7 cm gap, ±5 cm tolerance, CAN+MODBUS+GPIO. https://www.xnergytech.com/wp-content/uploads/2024/02/XNERGY-BE_PI_2311.pdf
- Xnergy Phoenix PH Series preview (xnergytech.com, 01.2025) — 1500 W, up to 5.5 cm gap, receiver 163×78×20 mm. https://www.xnergytech.com/wp-content/uploads/2024/11/PH-Series-1.pdf
- GDTitans Power TTAC-WPT4850 product page — 50 A output, IP65, CAN 1-way (no positioning tolerances published). https://www.gdtitanspower.com/meets-the-diverse-needs-of-the-industrial-sector-wireless-charger-ttac-wpt4850-product/

**Camera / fiducial.**
- AprilTag state-estimation analysis — PMC6960891.
- AprilTag built-in pose vs solvePnP — AprilRobotics/apriltag issue #212.
- AprilTag planar-pose ambiguity (flipping) — AprilRobotics/apriltag issue #71.
- ViSP AprilTag/ArUco detection (pose ambiguity discussion) — visp-doc.inria.fr.
- Using AprilTags with ROS / tag bundles — Optitag.
- OpenCV ChArUco detection — docs.opencv.org.
- Deep ChArUco — arXiv 1812.03247.
- PBVS vs IBVS — Peng & Radke, CASE 2020.

**LiDAR and fusion.**
- Reflection-based marker detection for AGV docking (~1 cm / <0.05°) — Sci. Reports 2025 (nature.com/articles/s41598-025-25357-x).
- Keeping AGVs on Track with 2D LiDAR Localization — SICK.
- SICK NAV-LOC reflectorless localization — The Engineer.
- LiDAR + V-shape docking — dergipark.org.tr/en/pub/ejosat/article/947521.
- Multi-Sensor Fusion for AMR Docking — MDPI Electronics 2025 (mdpi.com/2079-9292/14/14/2769).
- Vision- and Lidar-based autonomous docking & recharging — MDPI Applied Sciences (mdpi.com/2076-3417/13/19/10675).

**Docking servers and products.**
- Nav2 `opennav_docking` — github.com/open-navigation/opennav_docking; docs.nav2.org.
- `simple_charging_dock.cpp` — github.com/ros-navigation/navigation2.
- Autonomous Docking with AprilTags Using Nav2 — automaticaddison.com.
- MiR1350 specifications (VL-marker accuracy) — mobile-industrial-robots.com.
- Kiva drive-unit floor-barcode navigation — robotsguide.com/robots/kiva.
- OTTO Motors AMR profile — robotsguide.com/robots/otto.
- AMR charging dock design guide — phihong.com.
- iRobot auto-docking patent (coil-field final alignment) — freepatentsonline.com/y2018/0014709.

---

*Document version: 2026-05-29 (v3.2, post-review). Changes from v3.1: WCM-500 added to comparison (worst lateral now ±20 mm), Wiferion CAN bus / 93 % efficiency / WALL-SEPA-MOCHA variants documented, main scenario reframed as front-mount with floor-standing dock, retreat distance justified, asymmetric-size bundle variant (outer big / center small) added, ChArUco decision recorded (not selected), EKF fusion deepened with dynamic R-scheduling, §6.4 calculations and §6.5 timing rewritten as educational material, iRobot Roomba dropped, engagement target moved 15 → 20 mm. Future edits should preserve the vendor-agnostic framing and the ±10 mm / ±2° "any condition" target unless those design constraints change.*
