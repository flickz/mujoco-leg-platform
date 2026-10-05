# Leg and test platform — MuJoCo

Prepared from the live `leg_test_platform v12` Fusion assembly on 6 October 2026, including the local articulated leg and the knee enclosure clearance revision. This is an independent snapshot; it does not update when Fusion changes. The hidden linked source leg is excluded. All 318 visible CAD bodies are preserved in 24 colored visual meshes.

## Start

On this Mac, double-click **Launch Leg Platform.command**. 

- **Space**: pause/resume.
- **A**: automatic motion/manual actuator controls. In manual mode, expand **Control** on the right. Hip/knee position targets use radians; wheel input is torque in Nm.
- **R**: reset.
- Drag to orbit and scroll to zoom.

The demo cycles hip 0–70° and knee 0–40°, inside the configured limits, with the carriage held at the exported height. Passive pivots are solved by physics, not animated by overwriting their positions. The hold is a simulated fixture, not a powered carriage.

`platform.xml` releases the vertical carriage and exposes hip, knee and wheel **torque** actuators. Run the launcher with `--free` to inspect it. Without a controller it falls under gravity until the tyre contacts the plywood; it is not a jumping demonstration.

## Model files

| File | Purpose |
|---|---|
| `scene_demo.xml` | Supported servo demonstration, two equality constraints |
| `platform.xml` | Fixed rig, passive carriage slider, closed linkage, three torque actuators |
| `viewer.py` | Interactive demo; also supports `--free` and `--render` |
| `config.json` | Editable joint limits, timestep, servo settings, torque limits and mass assumptions |
| `build_model.py` | Regenerate meshes, inertias and MJCF after configuration edits |
| `validate.py` | CAD pose comparison, dynamic demo and free-carriage contact tests |
| `cad_export.json`, `cad_poses.json` | CAD dimensions, appearance, joint references, shape tensors and five measured reference poses |
| `raw_meshes/`, `meshes/` | Metre-unit source and localized meshes |
| `mass_audit.json` | Per-body mass assumptions and aggregated COM/inertia |
| `validation_report.json` | Test results |

All lengths are metres, masses kilograms, angles radians and motor commands Nm. +Z is up. Hip and knee positive axes are +Y. Positive knee angle rotates the horn and shin relative to the thigh; the upper pushrod pivot counter-rotates. The lower rod-to-shin pivot is a site-to-site `connect` equality.

Hip limits: **0–90°**. Knee actuator limits: **0–45°**. Wheel: continuous. Carriage travel: **0–0.361581858547 m** relative to its CAD lower stop. The export starts at 0.250458610691 m carriage displacement. Model has six scalar coordinates: carriage, hip, knee, upper rod pivot, shin pivot, wheel. Refer to joints/actuators by name rather than hardcoding indices.

The simulator adds the wheel hinge: CAD grouped the wheel into the shin. Tyre and the DD9015 rotating output (local Body37, corresponding to original output Body3) move with the wheel; remaining motor parts stay with the shin. Guide wheels are visually carried by the carriage and represented mechanically by an ideal prismatic joint.

## Verified

- Five CAD poses, including hip90°/knee45°, reproduced hinge-origin locations within 0.000053 mm.
- 12-second/12,000-step dynamic demonstration: no warnings/nonfinite states, maximum loop closure error 0.119 mm and maximum hip/knee tracking error 1.455°.
- Five-second unpowered free-carriage test: no warnings/nonfinite states; tyre/plywood contact established.
- Native viewer opened and inspected; offscreen preview included.

## Mass and contact assumptions

Estimated moving assembly mass is **4.214 kg**; fixed frame/base geometry is estimated at **4.787 kg**. These are simulation inputs, not measured weights. Native CAD material masses are not blindly adopted. Solid shape inertias are normalized and rescaled with explicit densities and mass budgets. RS06 motor totals are manufacturer-specified 0.621 kg each; DD9015 remains an estimated 0.534 kg; output splits and print fractions remain unmeasured. Aluminium is 2700 kg/m³, PETG 1270 kg/m³ with 60% solid-equivalent fraction, TPU 1200 kg/m³ with 85% fraction. The fraction is a mass assumption, not a slicer infill prescription. Small motor surface bodies have no separate solid mass; their mass is included once in the motor budget. Cable mass and motor electromagnetic/reflected rotor effects are not calibrated.

Collision geometry uses a plywood box, rectangular extrusion/carriage envelopes, narrow link capsules and a rigid tyre cylinder. Visual triangle meshes do not collide. Carriage/frame contact is excluded because the ideal slider represents the guides. Small fasteners, housing bulges, physical stop faces and cable flex are not detailed collision models. MuJoCo joint limits are soft numerical constraints and can show small penetration/overshoot. This model does not prove all CAD clearances, frame rigidity, landing strength, motor capability or jump performance. Calibrate mass, friction and actuator curves before interpreting forces or designing hardware controllers.

## Rebuild / transfer

With Python 3.12 and the dependencies in `requirements.txt`:

```sh
python build_model.py
python validate.py
```

Create `.venv` inside this repository and install `requirements.txt` (`python3 -m venv .venv` then `.venv/bin/pip install -r requirements.txt`). On macOS the launcher invokes `mjpython` explicitly to handle Cocoa and paths with spaces. On Linux/Windows run `python viewer.py`.

Controller example:

```python
import mujoco
m = mujoco.MjModel.from_xml_path('platform.xml')
d = mujoco.MjData(m)
d.ctrl[m.actuator('hip_motor').id] = 0.0
d.ctrl[m.actuator('knee_motor').id] = 0.0
d.ctrl[m.actuator('wheel_motor').id] = 0.0
mujoco.mj_step(m, d)
```

Implementation reference: [MuJoCo connect equality and MJCF](https://mujoco.readthedocs.io/en/stable/XMLreference.html#equality-connect), [Python viewer](https://mujoco.readthedocs.io/en/stable/python.html#passive-viewer).

## RS06 motor specifications and gravity update

Hip and knee use the [RobStride manufacturer specification table](https://github.com/RobStride/Product_Information), dated 2026-07-13 and checked 2026-10-06: 0.621 kg each, 11 Nm rated at 100 rpm, 36 Nm peak, 9:1 reduction, 48 V nominal (15–60 V range), 480 rpm ±10% no-load speed. These are output-shaft ratings: MuJoCo gear is 1, so torque is not multiplied by nine again.

Both models default to an 11 Nm force cap. In config.json, change `free_RS06_torque_mode` or `demo_RS06_torque_mode` from `rated` to `peak` and rebuild to use 36 Nm. Peak is not a continuous rating. Manufacturer rated torque requires specified heat sinking. No thermal/overload timer, voltage-dependent torque-speed curve, current limit or rotor inertia calibration is implemented. Speed/voltage specifications are recorded as reference data, not enforced as a hard mechanical speed stop. The actual 40 V battery will not reproduce all nominal 48 V performance. Servo gains remain simulation tuning. DD9015 wheel torque is still provisional.

Gravity is explicitly [0, 0, -9.81] m/s². Every rigid body has explicit mass, centre of mass and inertia, including the fixed frame. Child parts are aggregated into rigid bodies, avoiding duplicate mass on visual meshes. Mass totals are 4.214 kg moving and 4.787 kg fixed; printed geometry uses estimated solid fractions, so weigh the real parts before load predictions.

The supported demo holds the carriage mechanically; it therefore does not fall. Double-click **Launch Gravity Test.command** to open the free-carriage model with all motors initially unpowered. Press R to repeat the drop. This demonstrates gravity/contact, not controlled landing. Validation compares gravity-on/off acceleration (difference −9.81 m/s²), verifies positive body masses/inertias and rated torque caps, and repeats the pose and motion tests.

## Environment floor

The ground is an infinite static primitive plane at z=0, aligned with the plywood underside. The plywood top remains at z=0.018 m and retains its separate box collision; the floor does not replace or overlap that landing face. A matte procedural checker uses 0.10 m squares for visual scale. Floor contact parameters are explicit and editable in config.json: sliding friction 0.8, torsional friction length 0.005 m, condim 4 (no rolling resistance), solref [0.008, 1], solimp [0.9, 0.95, 0.001], zero contact margin. Floor priority 1 makes these settings deterministic for floor contacts. These are stable starting assumptions, not calibrated material properties. The 8 ms contact time constant exceeds twice the 1 ms integration timestep. Plane geoms avoid mesh seams and triangulated ground contacts. See [MuJoCo contact modeling](https://mujoco.readthedocs.io/en/stable/modeling.html) and [texture/material reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html#asset-material).

Validation includes a temporary 20 mm radius sphere dropped outside the plywood footprint onto the floor; it settles within 0.001 m of the expected height without warnings. The probe is not included in delivered XML. Updated native gravity viewer and preview inspected.

## Repository visibility

Private project; no open-source license is granted. See NOTICE.md.
