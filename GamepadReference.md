# 🎮 Gamepad Commands Cheat Sheet

Get to know your gamepad controls and their functionalities with this complete guide.

## 🔵 Basic Button Commands
- **`x`**: *Returns `true` if the X button is pressed, `false` otherwise.*
- **`y`**: *Returns `true` if the Y button is pressed, `false` otherwise.*
- **`b`**: *Returns `true` if the B button is pressed, `false` otherwise.*
- **`a`**: *Returns `true` if the A button is pressed, `false` otherwise.*

## 🎯 Directional Pad (D-Pad) Commands
- **`dpad_down`**: *Returns `true` if D-pad Down is pressed, `false` otherwise.*
- **`dpad_left`**: *Returns `true` if D-pad Left is pressed, `false` otherwise.*
- **`dpad_right`**: *Returns `true` if D-pad Right is pressed, `false` otherwise.*
- **`dpad_up`**: *Returns `true` if D-pad Up is pressed, `false` otherwise.*

## 🎮 Trigger and Bumper Controls
- **`left_trigger`**: *Returns a value from `0.0` to `1.0` as the left trigger is pressed.*
- **`right_trigger`**: *Returns a value from `0.0` to `1.0` as the right trigger is pressed.*
- **`left_bumper`**: *Returns `true` if the left bumper is pressed, `false` otherwise.*
- **`right_bumper`**: *Returns `true` if the right bumper is pressed, `false` otherwise.*

## 🛑 System and Auxiliary Buttons
- **`back`**: *Returns `true` if the Back button is pressed, `false` otherwise.*
- **`start`**: *Returns `true` if the Start button is pressed, `false` otherwise.*
- **`guide`**: *Returns `true` if the Guide/Mode button is pressed, `false` otherwise.*

## 🕹️ Joystick Controls
### Left Stick
- **`left_stick_x`**: *Returns the left-right deflection of the left stick.*
  - Range: `-1.0` (full left) to `+1.0` (full right)
- **`left_stick_y`**: *Returns the up-down deflection of the left stick.*
  - Range: `-1.0` (up) to `+1.0` (down)

### Right Stick
- **`right_stick_x`**: *Returns the left-right deflection of the right stick.*
  - Range: `-1.0` (full left) to `+1.0` (full right)
- **`right_stick_y`**: *Returns the up-down deflection of the right stick.*
  - Range: `-1.0` (up) to `+1.0` (down)

## 🔘 Joystick Button Presses
- **`left_stick_button`**: *Returns `true` if the left stick button is pressed, `false` otherwise.*
- **`right_stick_button`**: *Returns `true` if the right stick button is pressed, `false` otherwise.*

## 🔄 Utility Methods
- **`atRest()`**: *Returns `true` if all analog inputs (joysticks and triggers) are in their neutral positions, `false` otherwise.*
  - *Note*: This is a property and not an actual button.

## ⚙️ Configuration Options
- **`joystickDeadzone`**: *Represents the deadzone for joystick inputs.*
  - *Movements within this range are considered neutral to avoid accidental inputs.*
  - Default value: `0.2` (can be adjusted for sensitivity)

---

### 🔗 Additional Resources
For more details and best practices, refer to:
- [FTC Gamepad Documentation](https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/hardware/Gamepad.html)

