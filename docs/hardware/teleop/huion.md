# Huion tablet drawing on UFACTORY Lite 6

`examples.teleop_tablet` uses absolute Linux stylus input to control a fixed-orientation
pen in a calibrated paper plane. It runs through RIO's `XarmArm`/`task_pos` driver and
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
command-line arguments or interactive offset entry are needed. The default `None`
means the offset is unknown. All three values are in **millimeters in the reported
TCP's local coordinate frame**, not world coordinates. A scalar pen length is enough
only when its direction along that frame's axes is also known. If Studio's TCP is
already at the physical pen tip, set `tip_offset_mm = (0.0, 0.0, 0.0)`.

After editing the file, calibrate:

```bash
uv run -m examples.teleop_tablet calibrate \
  --robot-ip 192.168.2.176 --output paper.json
```

In this mode the wrist may rotate freely between the three contact points. The code
computes each tip position as `p_tip = p_tcp + R_tcp @ offset`, then converts the
three contacts into drawing targets at the **last taught orientation**. Choose the
desired drawing angle at the final corner; that angle stays fixed during drawing.
The offset is saved in `paper.json` as `tip_offset_m` for provenance. The program does
not change Studio's TCP configuration. Do not apply a flange-to-tip offset again if
Studio has already applied it to the reported TCP.

### Paper height and per-corner consistency

The measured offset does not replace touching the paper. The three marked corners
still establish its actual height, tilt, position, and drawing area. Raw TCP Z is not
directly comparable to a local TCP-to-tip Z offset; the offset must first be rotated
into the robot base frame. Corners on a tilted sheet may legitimately have different Z.

With a known offset, calibration defaults to **two contacts at each marked corner**.
Between contacts, change the wrist tilt and return the tip to the exact same mark.
Edit these settings in `examples/cfg/lite6_tablet.py`:

```python
calibration_repeats: int = 2
calibration_max_discrepancy_mm: float = 2.0
```

For each corner, calibration reports the maximum pairwise XYZ distance and the Z
spread between its computed tip positions. If any corner exceeds the configured
XYZ tolerance, it reports all three corners and refuses to overwrite `paper.json`.
Check the contact locations, rigid mounting, and measured offset; edit `tip_offset_mm`
and rerun when the offset is wrong. The program does not automatically alter that
physical measurement. Raw contact poses, tip positions, and discrepancy statistics
are saved in `contact_checks` when validation succeeds.

Three distinct points always define a plane, so their plane-fit residual alone cannot
validate the tip offset. Repeated contacts test consistency, not absolute accuracy:
the same wrist angle at every repeat can hide a common offset error. Pure spin about
the pen axis also cannot expose an axial length error; use different tilts. The final
accepted contact at each corner defines the plane, and the final sample's orientation
is used for drawing. Setting `calibration_repeats = 1` disables the repeated-contact
check; it is also unavailable in the legacy unknown-offset mode.

An optional `--tip-offset-mm X Y Z` overrides the file for one invocation. If both the
file setting and the override are absent, calibration retains the legacy assumption of a constant
unknown offset and requires orientation to stay within three degrees of the first
point. With an explicit offset, including zero for a tip TCP, that teaching restriction
does not apply. Startup still checks the chosen drawing orientation; leave the arm
at its final teaching pose or align to that same orientation before starting a run.

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

Keep the pen mount and paper fixed after teaching the three contact points. For a
first trial, an 80 x 50 mm drawing rectangle is a convenient small area with roughly
the Inspiroy 2 M's aspect ratio. Stop other robot-control scripts and leave the arm
over that rectangle with the taught pen orientation.

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
is another 5 mm higher. The arm first raises along the paper normal before any lateral
movement, including when starting at a taught contact point. Move the stylus slowly
and test two separate strokes, checking their direction and the lift between them.
Ctrl+C discards queued strokes and attempts a vertical retract to the travel height.

An air gap raises position targets; it does not add force sensing. To make contact
after a successful air test, rerun with `--air-gap-mm 0` (the default). Positive gaps
are also included as `meta/drawing_air_gap_m` in recordings.

## 5. Run and record

Position the arm above the calibrated rectangle with its taught orientation. Start:

```bash
uv run -m examples.teleop_tablet run \
  --robot-ip 192.168.1.111 \
  --calibration paper.json \
  --device /dev/input/eventN \
  --record-path data/drawing/
```

Lift the stylus before pressing Enter. Pre-session events are discarded. The arm first
retracts along the paper normal, then follows the tablet:

- Tip down: travel above the stroke start, lower to the taught contact plane, draw.
- Tip up: lift at the last contact point before traveling to the next stroke.
- Out of proximity/cropped region: end the current stroke and lift.
- Ctrl+C, device disconnect, event loss, or queue overflow: discard unfinished strokes,
  attempt a vertical retract from measured position, then save the recording.

Clearance defaults to 5 mm, drawing/travel speed to 20 mm/s, and lift/lower speed to
5 mm/s. Override with `--clearance-mm`, `--speed-mm-s`, and `--lift-speed-mm-s`.
These are position-path speed limits, not contact-force or acceleration control.
The rigid mount uses the exact taught contact plane; the software does not intentionally
push below it and cannot compensate for paper movement or measure contact force.

Every stylus report is queued, including fast tip-up/down transitions. If your hand
moves faster than the arm, the arm finishes those strokes in order. `--max-queue`
defaults to 10000; reaching it stops rather than silently dropping strokes. Transitions
wait for measured TCP arrival within 0.5 mm, and a tracking error over 10 mm stops the
session. No default gripper or startup homing motion is configured.

The recorder saves normal RIO robot states/actions and camera observations. Each step
also contains `meta/tablet/*`, drawing phase, queue length, and target time. A companion
`tablet_<session>_input.jsonl` preserves every raw report with its kernel timestamp and
relative host receipt time; `tablet_<session>_calibration.json` preserves the mapping.
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
