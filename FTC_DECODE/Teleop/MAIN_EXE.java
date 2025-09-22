package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.util.Range; // if you prefer Range.clip

@TeleOp(name = "MAIN_EXE_FINAL", group = "TeleOp")
public class MAIN_EXE extends OpMode {
    // Motors
    private DcMotor front_left, front_right, back_left, back_right;

    // <<< Paste your discovered live-trim here >>>
    private static final float OTHER_GAIN = 0.94f; // affects FL/FR/BR; BL stays 1.00

    // Sticks
    private float leftStickX, leftStickY, rightStickX;

    // Speed toggle
    private boolean slowMode = false;
    private boolean leftStickDownPrev = false; // edge-detect
    private float speedMultiplier = 1.0f;

    // Deadzone
    private static final float DEADZONE = 0.1f;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized (Final)");

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

        // mecanum mix
        float fl = leftStickY - leftStickX - rightStickX;
        float fr = leftStickY + leftStickX + rightStickX;
        float bl = leftStickY + leftStickX - rightStickX;
        float br = leftStickY - leftStickX + rightStickX;

        // normalize to keep max |power| ≤ 1
        float max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                             Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0f) { fl /= max; fr /= max; bl /= max; br /= max; }

        // apply fixed gains: FL/FR/BR scaled, BL is reference; then clip
        double pfl = clip(fl * OTHER_GAIN * speedMultiplier, -1, 1);
        double pfr = clip(fr * OTHER_GAIN * speedMultiplier, -1, 1);
        double pbl = clip(bl * 1.00f      * speedMultiplier, -1, 1); // BL unchanged
        double pbr = clip(br * OTHER_GAIN * speedMultiplier, -1, 1);

        front_left.setPower(pfl);
        front_right.setPower(pfr);
        back_left.setPower(pbl);
        back_right.setPower(pbr);

        telemetry.addData("Fixed otherGain (FL/FR/BR)", "%.2f", OTHER_GAIN);
        telemetry.addData("Mode", slowMode ? "SLOW (0.6x)" : "NORMAL (1.0x)");
        telemetry.update();
    }

    private static double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
        // return Range.clip(v, lo, hi); // if you prefer FTC's helper
    }
}
