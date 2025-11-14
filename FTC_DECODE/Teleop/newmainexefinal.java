package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "MAIN_EXE", group = "TeleOp")
public class MAIN_EXE extends OpMode {

    // ===== Drive motors =====
    private DcMotor front_left, front_right, back_left, back_right;

    // ===== Subsystem motors =====
    private DcMotor intakeHarvester; // A = forward (hold), Y = reverse (hold, also drives conveyor)
    private DcMotor conveyor;        // X/B toggle when Y is not held

    // ===== Tunables =====
    private static final float OTHER_GAIN    = 0.94f;  // tiny trim for FR/BR
    private static final float BL_GAIN       = 1.00f;  // keep BL at 1.0
    private static final float DEADZONE      = 0.10f;
    private static final float SLOW_MULT     = 0.60f;
    private static final float NORM_MULT     = 1.00f;

    // Conveyor speed cap you requested
    private static final double CONVEYOR_SPEED = 0.60; // 60% max in either direction

    // ===== State =====
    private float leftStickX, leftStickY, rightStickX;

    private boolean slowMode = false;
    private boolean leftStickDownPrev = false;
    private float speedMultiplier = NORM_MULT;

    // Toggle state for conveyor (used when Y is NOT held)
    private boolean xPrev = false;
    private boolean bPrev = false;
    private double  conveyorPowerToggled = 0.0; // +0.6, -0.6, or 0.0 when using X/B

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized (Intake+Conveyor linked on Y)");

        // Map drive
        front_left  = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_left   = hardwareMap.get(DcMotor.class, "back_left");
        back_right  = hardwareMap.get(DcMotor.class, "back_right");

        // Map subsystems
        intakeHarvester = hardwareMap.get(DcMotor.class, "intakeHarvester");
        conveyor        = hardwareMap.get(DcMotor.class, "conveyor");

        // Standard mecanum directions (POS power moves robot forward)
        front_left.setDirection(DcMotor.Direction.FORWARD);
        back_left.setDirection(DcMotor.Direction.FORWARD);
        front_right.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.REVERSE);

        // Subsystems (flip if needed)
        intakeHarvester.setDirection(DcMotor.Direction.FORWARD);
        conveyor.setDirection(DcMotor.Direction.FORWARD);

        // Brake at zero
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeHarvester.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        conveyor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Start off
        intakeHarvester.setPower(0);
        conveyor.setPower(0);
    }

    @Override
    public void loop() {
        // ===== Read sticks =====
        leftStickY  = gamepad1.left_stick_y;
        leftStickX  = gamepad1.left_stick_x;
        rightStickX = gamepad1.right_stick_x;

        if (Math.abs(leftStickY)  < DEADZONE) leftStickY  = 0;
        if (Math.abs(leftStickX)  < DEADZONE) leftStickX  = 0;
        if (Math.abs(rightStickX) < DEADZONE) rightStickX = 0;

        // ===== Speed toggle (Left stick button) =====
        boolean leftStickDown = gamepad1.left_stick_button;
        if (leftStickDown && !leftStickDownPrev) slowMode = !slowMode;
        leftStickDownPrev = leftStickDown;
        speedMultiplier = slowMode ? SLOW_MULT : NORM_MULT;

        // ===== Mecanum drive math =====
        float fl = leftStickY - leftStickX - rightStickX;
        float fr = leftStickY + leftStickX + rightStickX;
        float bl = leftStickY + leftStickX - rightStickX;
        float br = leftStickY - leftStickX + rightStickX;

        float max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                             Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0f) { fl /= max; fr /= max; bl /= max; br /= max; }

        double pfl = clip(fl * OTHER_GAIN * speedMultiplier, -1, 1);
        double pfr = clip(fr * OTHER_GAIN * speedMultiplier, -1, 1);
        double pbl = clip(bl * BL_GAIN    * speedMultiplier, -1, 1);
        double pbr = clip(br * OTHER_GAIN * speedMultiplier, -1, 1);

        front_left.setPower(pfl);
        front_right.setPower(pfr);
        back_left.setPower(pbl);
        back_right.setPower(pbr);

        // ===== Intake & Conveyor control =====
        // A = intake forward (hold)
        // Y = reverse (hold) AND also run the conveyor at 0.6 in the same (reverse) direction
        // X/B = conveyor toggle +0.6 / -0.6 (only when Y is NOT held)

        boolean holdA = gamepad1.a;
        boolean holdY = gamepad1.y;

        // Intake power (hold-to-run)
        double intakePower = 0.0;
        if (holdA) intakePower = -1.0;
        else if (holdY) intakePower = 1.0;
        intakeHarvester.setPower(intakePower);

        // Conveyor logic
        if (holdY) {
            // While Y is held, conveyor follows Y direction at capped speed
            conveyor.setPower(-CONVEYOR_SPEED); // reverse to match Y behavior (flip sign if needed)
        } else {
            // Use X/B toggles when Y is NOT held
            boolean xNow = gamepad1.x;
            if (xNow && !xPrev) {
                conveyorPowerToggled = (conveyorPowerToggled == CONVEYOR_SPEED) ? 0.0 : CONVEYOR_SPEED;
            }
            xPrev = xNow;

            boolean bNow = gamepad1.b;
            if (bNow && !bPrev) {
                conveyorPowerToggled = (conveyorPowerToggled == -CONVEYOR_SPEED) ? 0.0 : -CONVEYOR_SPEED;
            }
            bPrev = bNow;

            conveyor.setPower(conveyorPowerToggled);
        }

        // ===== Telemetry =====
        telemetry.addData("Mode", slowMode ? "SLOW (0.6x)" : "NORMAL (1.0x)");
        telemetry.addData("Drive", "FL %.2f  FR %.2f  BL %.2f  BR %.2f", pfl, pfr, pbl, pbr);
        telemetry.addData("Intake", "%.1f  (A=+1 hold, Y=-1 hold)", intakePower);
        telemetry.addData("Conveyor", "%.2f  (Y overrides @±0.60; X:+0.60 toggle, B:-0.60 toggle)", 
                          holdY ? -CONVEYOR_SPEED : conveyorPowerToggled);
        telemetry.update();
    }

    private static double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
