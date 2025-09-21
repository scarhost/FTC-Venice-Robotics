package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "MAIN_EXE", group = "TeleOp")
public class MAIN_EXE extends OpMode {
    // Drive train motors
    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor back_left;
    private DcMotor back_right;

    // Live trim for all motors except back_left (BL = reference)
    private float otherGain = 0.94f;       // scales FL/FR/BR; BL stays 1.00
    private boolean trimDebounce = false;  // prevents repeat on held D-pad

    // Gamepad control variables
    private float leftStickX, leftStickY, rightStickX, leftTrigger, rightTrigger;
    private boolean leftBump, rightBump, leftStickDown, buttonX, buttonY, buttonA, buttonB;
    private boolean dpadUp, dpadDown, dpadLeft, dpadRight;

    // Per-wheel power
    private final float[] power = new float[4];

    // Speed toggle
    private float  speedMultiplier = 1.0f;
    private boolean slowMode = false, antiRetrig = false, singleSetComplete = false;

    // Deadzone & normalize
    private final float deadzone = 0.1f;
    private float max;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialised");

        front_left  = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_left   = hardwareMap.get(DcMotor.class, "back_left");
        back_right  = hardwareMap.get(DcMotor.class, "back_right");

        // directions (so +power = forward)
        front_left.setDirection(DcMotor.Direction.FORWARD);
        back_left.setDirection(DcMotor.Direction.FORWARD);
        front_right.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.REVERSE);

        // coast on zero power (your original choice)
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop() {
        // sticks/buttons
        leftStickY   = gamepad1.left_stick_y;
        leftStickX   = gamepad1.left_stick_x;
        rightStickX  = gamepad1.right_stick_x;
        leftStickDown= gamepad1.left_stick_button;

        dpadUp   = gamepad1.dpad_up;
        dpadDown = gamepad1.dpad_down;

        // deadzones
        if (Math.abs(leftStickY)  < deadzone) leftStickY  = 0;
        if (Math.abs(leftStickX)  < deadzone) leftStickX  = 0;
        if (Math.abs(rightStickX) < deadzone) rightStickX = 0;
        if (Math.abs(leftTrigger) < deadzone) leftTrigger = 0;
        if (Math.abs(rightTrigger)< deadzone) rightTrigger= 0;

        // slow mode toggle (left stick click)
        if (leftStickDown && !antiRetrig) { slowMode = !slowMode; antiRetrig = true; }
        if (slowMode && antiRetrig && !singleSetComplete) { speedMultiplier = 0.6f; singleSetComplete = true; }
        else if (antiRetrig && !singleSetComplete)        { speedMultiplier = 1.0f; singleSetComplete = true; }
        if (!leftStickDown && antiRetrig) { antiRetrig = false; singleSetComplete = false; }

        // live trim: D-pad up/down -> ±1% for FL/FR/BR, BL unchanged
        if ((dpadUp || dpadDown) && !trimDebounce) {
            otherGain += dpadUp ? 0.01f : -0.01f;
            if (otherGain < 0.80f) otherGain = 0.80f;   // sane bounds
            if (otherGain > 1.10f) otherGain = 1.10f;
            trimDebounce = true;
        }
        if (!dpadUp && !dpadDown) trimDebounce = false;

        // mecanum mix: fwd/back, strafe, rotate
        power[0] = leftStickY - leftStickX - rightStickX; // FL
        power[1] = leftStickY + leftStickX + rightStickX; // FR
        power[2] = leftStickY + leftStickX - rightStickX; // BL
        power[3] = leftStickY - leftStickX + rightStickX; // BR

        // normalize
        max = Math.abs(power[0]);
        for (int i = 1; i < power.length; i++) if (Math.abs(power[i]) > max) max = Math.abs(power[i]);
        if (max > 1) for (int i = 0; i < power.length; i++) power[i] /= max;

        // apply (trim only FL/FR/BR)
        front_left.setPower (power[0] * otherGain * speedMultiplier);
        front_right.setPower(power[1] * otherGain * speedMultiplier);
        back_left.setPower  (power[2] * 1.00f     * speedMultiplier); // BL reference
        back_right.setPower (power[3] * otherGain * speedMultiplier);

        // HUD
        telemetry.addData("Trim otherGain (FL/FR/BR)", "%.2f", otherGain);
        telemetry.addData("Slow", slowMode ? "0.6x" : "1.0x");
        telemetry.update();   // <— make sure telemetry shows
    }
}
