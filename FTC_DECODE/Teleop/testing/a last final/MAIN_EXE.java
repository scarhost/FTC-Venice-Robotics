package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.util.Range; // optional if you prefer Range.clip

@TeleOp(name = "MAIN_EXE", group = "TeleOp")
public class MAIN_EXE extends OpMode {
    // Motors
    private DcMotor front_left, front_right, back_left, back_right;

    // Per-motor gains (set from your tuning)
    private static final float GAIN_FL = 0.94f;
    private static final float GAIN_FR = 0.94f;
    private static final float GAIN_BL = 1.00f;  // baseline
    private static final float GAIN_BR = 0.94f;

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

        // coast on zero power
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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
        if (max > 1.0f) {
            fl /= max; fr /= max; bl /= max; br /= max;
        }

        // apply gains & speed, then clip to [-1,1]
        double pfl = clip(fl * GAIN_FL * speedMultiplier, -1, 1);
        double pfr = clip(fr * GAIN_FR * speedMultiplier, -1, 1);
        double pbl = clip(bl * GAIN_BL * speedMultiplier, -1, 1);
        double pbr = clip(br * GAIN_BR * speedMultiplier, -1, 1);

        front_left.setPower(pfl);
        front_right.setPower(pfr);
        back_left.setPower(pbl);
        back_right.setPower(pbr);

        telemetry.addData("Gains", "FL %.2f | FR %.2f | BL %.2f | BR %.2f",
                GAIN_FL, GAIN_FR, GAIN_BL, GAIN_BR);
        telemetry.addData("Mode", slowMode ? "SLOW (0.6x)" : "NORMAL (1.0x)");
        telemetry.update();
    }

    private static double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
        // return Range.clip(v, lo, hi); // if you prefer FTC's helper
    }
}
