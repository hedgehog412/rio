# Huion tablet drawing on UFACTORY Lite 6

`examples.teleop_tablet` uses absolute Linux stylus input to control a fixed-orientation
pen in a calibrated paper plane. It runs through RIO's `XarmArm`/`task_pos` driver and
recorder. It does not require a leader arm, desktop mouse mapping, or an additional
Python dependency. Stylus pressure is recorded but does not command downward motion.

**Experimental: not tested on physical Huion or Lite 6 hardware.** Automated tests
use synthetic input and simulated robot feedback; physical drawing and hand guiding
remain unverified.

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

Keep the paper, pen mount, TCP configuration, and pen orientation fixed throughout
calibration and recording. Choose a reachable rectangle entirely on the paper.

```bash
uv run -m examples.teleop_tablet calibrate --robot-ip 192.168.1.111 --output paper.json
```

Stop any running RIO teleoperation scripts first. Calibration prompts you to support
the arm with the pen clear of the paper, then enables motors and **joint teaching
mode 2** (`motion_enable(True)`, `set_mode(2)`, `set_state(0)`). This is gravity-compensated
hand guiding, so you can move the follower directly by hand to each contact point:
tablet top-left, top-right, then bottom-left. It sends no pose/joint trajectories and
does not release the motors/brakes. Keep the pen's orientation the same at each point;
teaching mode allows rotation as well as translation. Configure the pen's actual payload
in Studio so gravity compensation is appropriate.

On completion, Ctrl+C, or an error, calibration stops teaching and returns to mode 0
to hold the current position before disconnecting. It never restores an earlier servo
mode or sends the arm home. Controller errors/warnings and SDK failures are reported;
they are not automatically cleared. The calibration rejects orientation
changes exceeding three degrees and degenerate rectangles. It stores TCP positions in
meters and orientation as a rotation vector in radians. A fixed pen offset is absorbed
by teaching the contact poses; no flange-to-tip conversion is needed.

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

## 4. Run and record

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
