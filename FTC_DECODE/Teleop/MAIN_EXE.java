package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "MAIN_EXE", group = "TeleOp")
public class MAIN_EXE extends OpMode {
    // Drive motors
    private DcMotor front_left, front_right, back_left, back_right;

    // === Fixed per-motor gains (from your tuning) ===
    private static final float GAIN_FL = 0.94f;
    private static final float GAIN_FR = 0.94f;
    private static final float GAIN_BL = 1.00f;  // baseline reference
    private static final float GAIN_BR = 0.94f;

    // Sticks / buttons
    private float leftStickX, leftStickY, rightStickX, leftTrigger, rightTrigger;
    private boolean leftStickDown;

    // Mecanum mix
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

        // your original choice: coast
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void loop() {
        // sticks
        leftStickY    = gamepad1.left_stick_y;
        leftStickX    = gamepad1.left_stick_x;
        rightStickX   = gamepad1.right_stick_x;
        leftStickDown = gamepad1.left_stick_button;

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

        // mecanum mix: fwd/back, strafe, rotate
        power[0] = leftStickY - leftStickX - rightStickX; // FL
        power[1] = leftStickY + leftStickX + rightStickX; // FR
        power[2] = leftStickY + leftStickX - rightStickX; // BL
        power[3] = leftStickY - leftStickX + rightStickX; // BR

        // normalize base mix to [-1,1]
        max = Math.abs(power[0]);
        for (int i = 1; i < power.length; i++) if (Math.abs(power[i]) > max) max = Math.abs(power[i]);
        if (max > 1) for (int i = 0; i < power.length; i++) power[i] /= max;

        // apply gains + speed, then clip to [-1,1]
        double fl = clip(power[0] * GAIN_FL * speedMultiplier, -1,  1);
        double fr = clip(power[1] * GAIN_FR * speedMultiplier, -1,  1);
        double bl = clip(power[2] * GAIN_BL * speedMultiplier, -1,  1);
        double br = clip(power[3] * GAIN_BR * speedMultiplier, -1,  1);

        front_left.setPower(fl);
        front_right.setPower(fr);
        back_left.setPower(bl);
        back_right.setPower(br);

        telemetry.addData("Gains", "FL %.2f | FR %.2f | BL %.2f | BR %.2f", GAIN_FL, GAIN_FR, GAIN_BL, GAIN_BR);
        telemetry.addData("Mode", slowMode ? "SLOW (0.6x)" : "NORMAL (1.0x)");
        telemetry.update();
    }

    private static double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
