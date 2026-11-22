# Huion tablet drawing on UFACTORY Lite 6

`examples.teleop_tablet` maps Linux stylus position and tilt onto a calibrated paper
plane. It runs through RIO's `XarmArm`/`task_pos` driver and
recorder. It does not require a leader arm, desktop mouse mapping, or an additional
Python dependency. Stylus pressure is recorded but does not command downward motion.

**Experimental: Lite 6 motion and paper contact remain unverified on physical hardware.**
The user has verified stylus coordinates, contact, and both side buttons with DIGImend
13 on Ubuntu 20.04/kernel 5.15. Automated motion tests use simulated robot feedback.

## 1. Find the stylus input node

Connect the Inspiroy 2 by USB. First check Ubuntu's USB and input layers:

```bash
lsusb
sudo apt install evtest
sudo evtest
```

Select the Huion pen/stylus node in `evtest`. Moving the pen should produce `ABS_X`
and `ABS_Y`; touching/lifting should switch `BTN_TOUCH` between 1 and 0. Pressure
should change in `ABS_PRESSURE`. Ctrl+C exits. USB enumeration alone does not prove
that stylus input works.

Then run from the repository root, as your normal user:

```bash
uv run -m examples.teleop_tablet devices
```

Choose the node with pen proximity (`BTN_TOOL_PEN`), tip contact (`BTN_TOUCH`), and
absolute X/Y. Tablet buttons and mouse emulation nodes are not suitable. The command
reads axis ranges from the device, so it supports the S/M/L variants without hardcoded
tablet dimensions. Use a stable `/dev/input/by-id/...-event...` symlink when available.
`--device auto` works when exactly one readable pen node exists.

Test the exact reader used by drawing, without calibration or a robot connection:

```bash
uv run -m examples.teleop_tablet check-input --device /dev/input/eventN --seconds 15
```

During those 15 seconds, move on both axes, touch the tip, then lift it into hover.
The command prints normalized coordinates, pressure, contact, and proximity. It prints
`PASS` only after observing both axes move and a contact-to-hover transition. An
incomplete check exits with a nonzero status; repeat the gestures before diagnosing
a driver failure. This command never starts a robot node.

To include the two stylus side buttons in the required checks:

```bash
uv run -m examples.teleop_tablet check-input --device /dev/input/eventN --seconds 20 --require-buttons
```

Keep the pen hovering close enough for `proximity=1`. Press and release each side
button separately. The reader maps Linux `BTN_STYLUS` (331) to `button1` and
`BTN_STYLUS2` (332) to `button2`. It prints every button transition, even a quick
press/release between normal diagnostic print intervals, and reports whether the
kernel advertises each button. Button states are also included in recorded tablet
metadata and input JSONL; they do not command the arm or gripper.

If these buttons do not appear in `sudo evtest /dev/input/eventN`, inspect the other
Huion mouse/keyboard input nodes shown by `sudo evtest`: the installed driver may
map the buttons to mouse clicks or keyboard shortcuts. Restore the driver's pen-button
defaults and repeat the check. If `evtest` itself never receives the button events,
the drawing reader cannot recover them from that input node.

If permissions are denied, add a rule restricted to your tablet. For a Huion device
whose USB vendor ID is `256c` (verify with `lsusb`), create
`/etc/udev/rules.d/70-huion-tablet.rules` containing:

```udev
SUBSYSTEM=="input", KERNEL=="event*", ATTRS{idVendor}=="256c", TAG+="uaccess"
```

Reload with `sudo udevadm control --reload-rules`, unplug/replug, and rerun discovery
from your logged-in Ubuntu desktop session. This avoids granting access to every
keyboard/input device. If no pen node is exposed, install the driver appropriate to
your Ubuntu version and verify the tablet in `evtest` first. Do not enable relative
mouse mode. The reader opens evdev directly and does not grab the desktop device.

## 2. Teach the paper rectangle

Keep the paper, pen mount, and robot TCP configuration fixed throughout calibration
and recording. Choose a reachable rectangle entirely on the paper.

```bash
uv run -m examples.teleop_tablet calibrate --robot-ip 192.168.1.111 --output paper.json
```

Stop any running RIO teleoperation scripts first. Calibration prompts you to support
the arm with the pen clear of the paper, then enables motors and **joint teaching
mode 2** (`motion_enable(True)`, `set_mode(2)`, `set_state(0)`). This is gravity-compensated
hand guiding, so you can move the follower directly by hand to each contact point:
tablet top-left, top-right, then bottom-left. It sends no pose/joint trajectories and
does not release the motors/brakes. Teaching mode allows rotation as well as translation.
Configure the pen's actual payload in Studio so gravity compensation is appropriate.

### Teaching with a known TCP-to-tip offset

Edit `tip_offset_mm` near the top of `examples/cfg/lite6_tablet.py` to set the measured
TCP-to-pen-tip vector `(X, Y, Z)`. Calibration reads this file automatically; no offset
command-line arguments or interactive offset entry are needed. Setting it to `None`
means the offset is unknown. All three values are in **millimeters in the reported
TCP's local coordinate frame**, not world coordinates. A scalar pen length is enough
only when its direction along that frame's axes is also known. If Studio's TCP is
already at the physical pen tip, set `tip_offset_mm = (0.0, 0.0, 0.0)`.

After editing the file, calibrate:

```bash
uv run -m examples.teleop_tablet calibrate \
  --robot-ip 192.168.2.176 --output paper.json
```

In this mode the wrist may rotate freely between contact points. The code
computes each tip position as `p_tip = p_tcp + R_tcp @ offset`, then converts the
three contacts into drawing targets at the **last taught orientation**. Choose the
reference drawing angle at the final contact. Fixed-angle mode keeps it; tilt mode
uses it to derive wrist rotation around the mapped pen-body direction.
The offset is saved in `paper.json` as `tip_offset_m` for provenance. The program does
not change Studio's TCP configuration. Do not apply a flange-to-tip offset again if
Studio has already applied it to the reported TCP.

### Paper height and surface consistency

The measured offset does not replace touching the paper. Contact samples still
establish its actual height, tilt, and position; three corners define the drawing
area. Raw TCP Z is not
directly comparable to a local TCP-to-tip Z offset; the offset must first be rotated
into the robot base frame. Corners on a tilted sheet may legitimately have different Z.

With a known offset, the default workflow requires **no exact-point repeats**:

1. Touch the origin/top-left, top-right, and bottom-left of the drawing area once.
2. Touch three other distinct points anywhere inside that area, keeping the tip in
   light contact with the sheet. You do not need to enter or reproduce their XYZ.
3. At the last contact, choose the wrist orientation to keep during drawing.

The code computes tip positions using the measured offset and fits a plane through
all six contacts. It reports each contact's perpendicular distance from that plane,
plus the maximum and RMS residual. The first three corners are projected onto the
fitted plane to map the tablet's drawing area. Different XY locations and wrist
angles are permitted; paper contact supplies the surface constraint.

Edit these settings near the bottom of `examples/cfg/lite6_tablet.py`:

```python
calibration_mode: str = "plane"
calibration_surface_samples: int = 3
calibration_plane_tolerance_mm: float = 2.0
```

`calibration_surface_samples` is the number of additional contacts after the three
corners and must be at least one. If any point exceeds the plane tolerance, calibration
reports the residuals and refuses to overwrite `paper.json`. Check contact, sheet
flatness, mounting, and the measured offset. Raw poses, tip positions, and plane-fit
statistics are saved in `surface_fit`. The program does not auto-adjust the offset.

Three noncollinear points always define a plane, which is why extra contacts are used
to check surface consistency. Even a good fit cannot prove absolute tip-offset accuracy:
a common offset error can look like a different paper height or tilt. Keep the offset
based on a physical measurement and use the air-gap motion test before drawing.

The older exact-point check remains optional with `calibration_mode = "repeat"`.
Only that mode uses `calibration_repeats` and `calibration_max_discrepancy_mm` near the
top of the configuration. Its raw contacts and statistics are saved in `contact_checks`.

An optional `--tip-offset-mm X Y Z` overrides the file for one invocation. If both the
file setting and the override are absent, calibration retains the legacy assumption of a constant
unknown offset and requires orientation to stay within three degrees of the first
point. With an explicit offset, including zero for a tip TCP, that teaching restriction
does not apply. With `tip_offset_m` saved in the calibration, startup automatically
aligns the wrist. The physical tip must start over the drawing area, on or above
the paper; its initial wrist angle may differ. Legacy files with no tip offset
still require manual alignment because rotation compensation cannot be calculated.

After the final contact, calibration keeps hand guiding enabled and prompts you to
lift the pen clear of the sheet and reposition it above the drawing area. Keep the
last contact's wrist orientation if convenient; startup can align it later. Press Enter only when ready to hold
the new position. This repositioning is not another calibration contact and does not
change the taught plane, corner mapping, or drawing orientation. Existing
`paper.json` files remain usable; recalibration is not needed for this change.

On completion, Ctrl+C, or an error, calibration stops teaching and returns to mode 0
to hold the current position before disconnecting. It never restores an earlier servo
mode or sends the arm home. Controller errors/warnings and SDK failures are reported;
they are not automatically cleared. A controller fault encountered while teaching
also prevents recording the contact. Degenerate rectangles are rejected. Calibration
stores drawing TCP targets in meters and orientation as a rotation vector in radians.

The clearance normal defaults toward robot base +Z, appropriate for desktop paper.
Inspect the printed normal; `--flip-normal` reverses it for a differently mounted
surface. The arm follows the taught plane even when the paper is tilted.

By default the full tablet maps to the taught rectangle. Match their aspect ratios to
preserve proportions, or edit `tablet_bounds` in `paper.json` to crop the tablet's active
region: `[u_min, v_min, u_max, v_max]`, with normalized coordinates in `[0, 1]`.
Leaving this region lifts the pen instead of drawing along a clamped boundary.

## 3. Preview without connecting to the robot

```bash
uv run -m examples.teleop_tablet preview --calibration paper.json --device /dev/input/eventN
```

Draw on the tablet and press Ctrl+C. The command finishes the buffered strokes and
writes `tablet_preview.svg`. Open it to check orientation, crop, and stroke separation.
`--output PATH` changes the output file. To exercise the entire mapping and stroke
controller without a tablet, add `--demo`; this draws a square and a separate diagonal.
Neither preview mode imports or starts the robot driver.

## 4. First motion test above the paper

To check the current physical tip position without enabling motors, changing robot
modes, or sending motion commands, run:

```bash
uv run -m examples.teleop_tablet inspect-start \
  --robot-ip 192.168.2.176 --calibration paper.json --air-gap-mm 10
```

This prints the reported TCP, saved tip offset, computed tip XYZ, normalized paper
coordinates U/V, and perpendicular height above the paper. U and V must be from 0
to 1 (with a 1% edge tolerance). A negative height beyond the calibration contact
tolerance is rejected. There is no upper starting-height limit; the old travel-height
plus 50 mm ceiling rejected otherwise valid lifted poses. Startup motion remains
speed-limited. An outside-area tip must be positioned over the taught drawing area
before running. If calculated height disagrees with the real sheet, check the saved
offset, unchanged TCP settings, and paper placement before proceeding.

Keep the pen mount and paper fixed after teaching the three contact points. For a
first trial, an 80 x 50 mm drawing rectangle is a convenient small area with roughly
the Inspiroy 2 M's aspect ratio. Stop other robot-control scripts and leave the arm
with the physical tip over that rectangle. Its wrist orientation may differ when
the calibration contains the known tip offset.

Use an air gap for the first motion test, without recording:

```bash
uv run -m examples.teleop_tablet run \
  --robot-ip 192.168.2.176 \
  --calibration paper.json \
  --device /dev/input/event29 \
  --air-gap-mm 10 \
  --speed-mm-s 5 \
  --lift-speed-mm-s 2 \
  --no-record
```

This example uses the follower IP in `Lite6LeaderFollowerStation` and the user's
verified pen node; adjust the IP if your robot configuration differs. The calibrated
paper plane stays unchanged. Stylus contact now traces 10 mm above it; pen-up travel
is another 5 mm higher. If the wrist needs alignment, the arm first lifts at its current
angle to at least 20 mm above the paper, then rotates while compensating the TCP
to hold the elevated tip fixed. The clearance increases if needed to keep the
straight TCP-to-tip segment above the plane. Each transition waits for measured
position/orientation arrival. This is not obstacle or whole-arm collision planning.
Tablet events during alignment are discarded; lift the stylus before starting a stroke.
The aligned arm then moves to the selected travel height. Move the stylus slowly
and test two separate strokes, checking their direction and the lift between them.
Ctrl+C discards queued strokes and attempts a vertical retract to the travel height.

After Enter, `run` captures the selected pen input node exclusively until exit;
desktop applications no longer receive events from that node. Keyboard and separate
mouse devices remain usable. Close `evtest --grab` or another exclusive reader first.

Once per second, the terminal prints the motion phase, measured TCP displacement,
tip height, wrist angle remaining, tracking error, tablet report count/contact, and
raw tilt angles. `phase=ready` means the arm is waiting for stylus input.
Watch the remaining angle
decrease during `align`; tablet strokes are ignored until alignment finishes.
If a pending move makes no measured position or orientation progress for three
seconds, the run stops and attempts its normal cleanup. A dead RIO arm command
thread is rejected even if the independent pose-report thread is still publishing.
These diagnostics distinguish waiting for alignment, missing tablet events, and a
command/feedback problem; a moving desktop cursor alone does not verify RIO input.

An air gap raises position targets; it does not add force sensing. To make contact
after a successful air test, rerun with `--air-gap-mm 0` (the default). Positive gaps
are also included as `meta/drawing_air_gap_m` in recordings.

## 5. Run and record

Position the physical tip above the calibrated rectangle. Startup aligns its wrist
using the saved tip offset. Start:

```bash
uv run -m examples.teleop_tablet run \
  --robot-ip 192.168.1.111 \
  --calibration paper.json \
  --device /dev/input/eventN \
  --record-path data/drawing/
```

After Ctrl+C, the arm retracts and the recording is saved and closed. The terminal
then prompts for optional notes, such as `Operator: Jihyun; drawing: flower`.
Press Enter to skip or keep existing notes. Ctrl+C or end-of-input at this prompt
also keeps the saved recording. The prompt is disabled by `--no-record`; hardware
errors save the recording through the existing cleanup and propagate without a prompt.

Every recording gets a unique UTC date/time filename with a short random suffix,
such as `traj_20261122T190000_123456Z_a1b2c3d4.vla`. Existing recordings are preserved.
If `--record-path` ends in `.vla`, its filename is used as a prefix; for example,
`data/flower.vla` produces `data/flower_<timestamp>_<suffix>.vla`.
A matching `.json` file stores the note, creation time, and trajectory filename;
notes entered at the closing prompt update this JSON without rewriting the `.vla`.
You can still supply `--note "Operator: Jihyun; drawing: flower"` before recording;
that initial note is also included as `meta/recording/note` in the `.vla` steps.

Lift the stylus before pressing Enter. Pre-session events are discarded. The arm first
retracts along the paper normal, then follows the tablet:

- Tip down: acquire the stylus position smoothly above the paper; touching alone never enables live tracking.
- READY: press and release stylus button 1, hold still through LOWERING, then draw after LIVE TRACKING appears.
- Tip up: immediately disable live tracking, discard pending drawing, and lift vertically from the measured tip position.
- Out of proximity/cropped region: end the current stroke and lift.
- Ctrl+C, device disconnect, event loss, or queue overflow: discard unfinished strokes,
  attempt a vertical retract from measured position, then save the recording.

Clearance defaults to 5 mm. The station enables live targets and button clutching;
tilt following is disabled by default. Edit these values in `examples/cfg/lite6_tablet.py`:

```python
realtime: bool = True
follow_pen_tilt: bool = False
down_speed_mm_s: float = 2.0
tilt_limit_deg: float = 60.0
pen_axis_tcp: tuple[float, float, float] = (0.0, 0.0, -1.0)
tilt_signs: tuple[float, float] = (1.0, 1.0)
button_clutch: bool = True
clutch_button: int = 1
clutch_position_speed_mm_s: float = 20.0
clutch_rotation_speed_deg_s: float = 15.0
```

With the default button clutch, each stroke follows this sequence:

1. Place the stylus at its start point, touching the tablet. The robot positions
   the physical tip above that point, including tilt, using the acquisition speeds.
2. Wait for `READY`. This requires measured position within 0.5 mm and angle within
   0.5 degrees of the latest target. Press the configured button only after READY.
3. Hold the stylus still during `LOWERING`. The approach retains the configured
   downward speed limit; live tracking begins only after measured arrival.
4. Draw when `LIVE TRACKING` appears. The button latches this stroke; holding it
   down is unnecessary. Lifting the stylus, leaving proximity, or leaving the mapped
   area disables tracking and requires positioning plus a fresh button press again.

An early press is ignored and must be released and pressed again after READY.
Holding a button across a lift never enables the next stroke. The other stylus
button has no clutch effect. Readiness is rechecked against the latest target and
feedback on every press. During acquisition and lowering, changing targets stay
speed-limited instead of accumulating old points or jumping into live motion.
A stationary pen normally emits no evdev events; silence alone does not reset a
stroke. Explicit tip-up/proximity-loss events reset it; device disconnect or dropped
input events stop the run and retract through normal cleanup. `--no-button-clutch`
restores the previous touch-to-draw behavior when explicitly requested.

After clutch engagement, live mode has no configured lateral/upward/rotational speed cap. The driver
interpolates live targets and the robot's physical limits still apply; actual motion
cannot be guaranteed to match arbitrary hand motion. Only the tip's motion toward
the sheet is limited by `down_speed_mm_s`, or `--lift-speed-mm-s` for one run.
`--speed-mm-s` applies only with `--no-realtime`. Downward approach can still delay
the start of a stroke. Orientation changes are subdivided into at most 5 degrees
per control update for continuous TCP compensation; at 50 Hz this permits up to
250 degrees/s before the robot's limits. Transitions still wait for measured arrival.

Linux `ABS_TILT_X/Y` are read as signed degrees and included in recordings. Positive
X/Y lean maps toward paper +U/+V. The two signs can invert either direction. The
body axis points from tip toward pen body in the reported TCP frame; `-Z` matches
the measured +Z TCP-to-tip offset. Zero tilt points that body axis along the paper
normal. Tilting rotates the wrist while recomputing TCP XYZ to hold the physical
tip at its mapped point and selected air gap. Total lean is limited to the configured
angle. Barrel twist is derived from the taught orientation; it is not measured.
No paper recalibration is required when the saved tip offset and mounting are unchanged.

Use `--no-follow-tilt` for fixed-angle drawing, or `--no-realtime` to restore capped
queued playback. In capped mode `--speed-mm-s` limits travel/drawing (default 20 mm/s),
`--lift-speed-mm-s` limits lift/lower, and `tilt_speed_deg_s` limits rotation.
The rigid mount uses the exact taught contact plane; the software does not intentionally
push below it and cannot compensate for paper movement or measure contact force.

Live mode replaces superseded continuous drawing targets with the newest sample.
With the clutch enabled, tip-up cancels unfinished targets and retracts at the actual
measured position, preventing a delayed segment from joining strokes. Capped
mode queues every report. `--max-queue` defaults to 10000; exhausting the transition
queue stops the session. Arrival gates require position within 0.5 mm and orientation
within 0.5 degrees. The 10 mm/5 degree lag guards apply only to capped playback;
live mode expects tracking lag and retains fresh-feedback, worker-health and
no-progress checks. No default gripper or startup homing motion is configured.

The recorder saves normal RIO robot states/actions and camera observations. Each step
also contains `meta/tablet/*`, drawing phase, queue length, and target time. A companion
`<recording-stem>_input.jsonl` preserves every raw report with its kernel timestamp and
relative host receipt time; `<recording-stem>_calibration.json` preserves the mapping.
`--no-record` disables both recording and these companion files.

To include cameras, add them to `Lite6TabletStation` in `examples/cfg/lite6_tablet.py`,
or register a copy of that station with your camera configuration and pass
`--station YourDrawingStation`. The entry point checks that the station uses Lite 6
Cartesian position control without homing, gripper, hand, or other teleop nodes.
The default station uses the existing robot TCP settings. If your station sets a TCP
offset, teach the calibration with that same offset active in Studio.

Only the explicit `run` command connects through the motion driver. Validate the
tablet mapping in preview and the physical motion with paper clear of the pen before
collecting contact demonstrations. Physical contact behavior requires validation on
your actual arm and pen.
