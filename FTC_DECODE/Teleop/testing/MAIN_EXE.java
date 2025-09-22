package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "MAIN_EXE_TRIM", group = "Testing")
public class MAIN_EXE extends OpMode {
    // Drive motors
    private DcMotor front_left, front_right, back_left, back_right;

    // Live trim for all motors except back_left (BL = reference)
    private float otherGain = 1.00f;       // start at 1.00; adjust with D-pad
    private boolean trimDebounce = false;  // prevents repeat on held D-pad

    // Sticks
    private float leftStickX, leftStickY, rightStickX;

    // Slow mode (edge-detected)
    private boolean slowMode = false;
    private boolean leftStickDownPrev = false;
    private float speedMultiplier = 1.0f;

    // Deadzone
    private static final float DEADZONE = 0.1f;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized (Trim Test)");

        front_left  = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_left   = hardwareMap.get(DcMotor.class, "back_left");
        back_right  = hardwareMap.get(DcMotor.class, "back_right");

        // directions (so +power = forward)
        front_left.setDirection(DcMotor.Direction.FORWARD);
        back_left.setDirection(DcMotor.Direction.FORWARD);
        front_right.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.REVERSE);

        // stop quickly when power=0 (BRAKE). Change to FLOAT if you prefer coasting.
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // (Optional) open-loop:
        // front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        // read sticks
        leftStickY  = gamepad1.left_stick_y;
        leftStickX  = gamepad1.left_stick_x;
        rightStickX = gamepad1.right_stick_x;

        // deadzones
        if (Math.abs(leftStickY)  < DEADZONE) leftStickY  = 0;
        if (Math.abs(leftStickX)  < DEADZONE) leftStickX  = 0;
        if (Math.abs(rightStickX) < DEADZONE) rightStickX = 0;

        // slow-mode toggle (left stick click, edge-detected)
        boolean leftStickDown = gamepad1.left_stick_button;
        if (leftStickDown && !leftStickDownPrev) slowMode = !slowMode;
        leftStickDownPrev = leftStickDown;
        speedMultiplier = slowMode ? 0.6f : 1.0f;

        // live trim: D-pad up/down -> ±1% for FL/FR/BR, BL unchanged
        boolean dpadUp   = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;
        if ((dpadUp || dpadDown) && !trimDebounce) {
            otherGain += dpadUp ? 0.01f : -0.01f;
            if (otherGain < 0.80f) otherGain = 0.80f; // sane bounds
            if (otherGain > 1.10f) otherGain = 1.10f;
            trimDebounce = true;
        }
        if (!dpadUp && !dpadDown) trimDebounce = false;

        // quick reset: X sets trim back to 1.00
        if (gamepad1.x) otherGain = 1.00f;

        // mecanum mix
        float fl = leftStickY - leftStickX - rightStickX;
        float fr = leftStickY + leftStickX + rightStickX;
        float bl = leftStickY + leftStickX - rightStickX;
        float br = leftStickY - leftStickX + rightStickX;

        // normalize to keep max |power| ≤ 1
        float max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                             Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0f) { fl /= max; fr /= max; bl /= max; br /= max; }

        // apply trim (FL/FR/BR scaled, BL is reference), then clip
        double pfl = clip(fl * otherGain * speedMultiplier, -1, 1);
        double pfr = clip(fr * otherGain * speedMultiplier, -1, 1);
        double pbl = clip(bl * 1.00f     * speedMultiplier, -1, 1); // BL unchanged
        double pbr = clip(br * otherGain * speedMultiplier, -1, 1);

        front_left.setPower(pfl);
        front_right.setPower(pfr);
        back_left.setPower(pbl);
        back_right.setPower(pbr);

        telemetry.addData("Trim otherGain (FL/FR/BR)", "%.2f", otherGain);
        telemetry.addData("Mode", slowMode ? "SLOW (0.6x)"
