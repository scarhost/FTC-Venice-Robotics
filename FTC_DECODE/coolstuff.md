# FTC Robot Cheat Sheet — Encoders & TeleOp

This guide explains the key parts of your **Autonomous (Encoder_Auto_Tuner)** and **TeleOp (MAIN_EXE)** programs. It’s designed as a quick reference for tuning and testing your robot.

---

## 🔧 1. Constants You Can Tune

In **Encoder_Auto_Tuner.java**:

- `TICKS_PER_REV` → encoder ticks per motor rev (REV HD Hex 20:1 = 560).  
- `WHEEL_DIAMETER_IN` → wheel diameter (2.95 in for 75 mm).  
- `EXTERNAL_GEAR_RATIO` → if gears/sprockets are used (1.0 = direct).  
- `STRAFE_CORRECTION` → multiplier for sideways motion (start 1.10).  
- `WHEELBASE_DIAMETER_IN` → distance between left/right wheels (start ~14.0).  
- `MAX_POWER` → power cap for autonomous moves (0.6).  
- `TIMEOUT_S_PER_FOOT` → timeout budget (2.5 s per foot).  

In **MAIN_EXE.java**:

- `GAIN_FL, GAIN_FR, GAIN_BL, GAIN_BR` → scale each motor’s power (fixes drift).  

---

## 📐 2. Quick Math

- **Ticks per inch (TPI):**  
  ```
  TPI = (TICKS_PER_REV × EXTERNAL_GEAR_RATIO) / (π × WHEEL_DIAMETER_IN)
  ```

- **Ticks for distance:**  
  ```
  ticks = inches × TPI
  ```

- **Turn arc length:**  
  ```
  arc_in = (degrees / 360) × π × WHEELBASE_DIAMETER_IN
  ticks  = arc_in × TPI
  ```

Example (90° turn, 14 in wheelbase, 2.95 in wheels, 560 ticks/rev):  
- arc ≈ 11 in → ticks ≈ 664 per side.

---

## 🤖 3. Autonomous Functions

- `driveInches(inches, power, timeoutS)` → move forward/backward.  
- `strafeInches(inches, power, timeoutS)` → move sideways (with correction).  
- `turnByEncoders(degrees, power, timeoutS)` → rotate using encoders only.  
- `ticksFromInches(inches)` → convert inches to ticks.  
- `getTimeout(inches)` → sets safety timeout based on distance.  

---

## 🕹️ 4. TeleOp Features

- **Gains:** set per motor (e.g., `FL=0.94, FR=0.94, BL=1.00, BR=0.94`).  
- **Slow Mode:** left stick click → toggle between 1.0× and 0.6×.  
- **Deadzone:** ignores stick inputs smaller than 0.1.  
- **ZeroPowerBehavior:**  
  - `BRAKE` → stops immediately.  
  - `FLOAT` → coasts when released.  

---

## 🔄 5. Tuning Steps

### Forward / Backward
1. Command 24 in, measure actual.  
2. Adjust `WHEEL_DIAMETER_IN` or `TICKS_PER_REV`:  
   ```
   new = old × (commanded / actual)
   ```

### Strafing
- If commanded 24 in but moved 21 in →  
  ```
  STRAFE_CORRECTION *= (24/21) ≈ 1.14
  ```

### Turning
- If commanded 90° but got 80° →  
  ```
  WHEELBASE_DIAMETER_IN *= (90/80) ≈ 1.125
  ```
- If overshoot → reduce wheelbase value.

### TeleOp Drift
- Curves left → left motors too strong → lower FL/BL gains.  
- Curves right → right motors too strong → lower FR/BR gains.  

---

## 🛠️ 6. Troubleshooting

- **Drifts in forward drive:** check motor gains.  
- **Rotates while strafing:** increase `STRAFE_CORRECTION`.  
- **Doesn’t finish RUN_TO_POSITION:** check encoders, raise timeout.  
- **Robot slides when stopping:** use `BRAKE` mode.  

---

## 📋 7. Test Log Template

| Move        | Commanded | Actual | Fix (new constant) |
|-------------|-----------|--------|--------------------|
| Forward     |           |        |                    |
| Strafe      |           |        |                    |
| Turn        |           |        |                    |

---
