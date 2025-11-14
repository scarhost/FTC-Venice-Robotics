package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "MAIN_EXE", group = "TeleOp")
public class MAIN_EXE extends OpMode {

    // ====== DRIVETRAIN ======
    private DcMotor front_left, front_right, back_left, back_right;

    // ====== SUBSYSTEMS ======
    private DcMotor intakeHarvester;
    private DcMotor conveyor;
    private DcMotor launcher;   // NEW

    // ====== DRIVE SETTINGS ======
    private static final float DEADZONE   = 0.10f;
    private static final float SLOW_MULT  = 0.60f;
    private static final float NORM_MULT  = 1.00f;

    private boolean slowMode = false;
    private boolean leftStickPrev = false;
    private float speedMult = NORM_MULT;

    // ====== INTAKE / CONVEYOR / LAUNCHER SPEEDS ======
    private static final double INTAKE_POWER    = 1.0;   // full power
    private static final double CONVEYOR_POWER  = 1.0;   // full power
    private static final double LAUNCHER_POWER  = 1.0;   // launcher power

    // ====== LAUNCHER TOGGLE STATE ======
    private boolean xPrev = false;
    private boolean launcherOn = false;

    @Override
    public void init() {

        // ---- Map hardware (names must match configuration) ----
        front_left  = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_left   = hardwareMap.get(DcMotor.class, "back_left");
        back_right  = hardwareMap.get(DcMotor.class, "back_right");

        intakeHarvester = hardwareMap.get(DcMotor.class, "intakeHarvester");
        conveyor        = hardwareMap.get(DcMotor.class, "conveyor");
        launcher        = hardwareMap.get(DcMotor.class, "launcher"); // NEW: make sure this name matches RC config

        // ---- Drivetrain directions ----
        front_left.setDirection(DcMotor.Direction.FORWARD);
        back_left.setDirection(DcMotor.Direction.FORWARD);
        front_right.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.REVERSE);

        // Intake reversed (from earlier fix), others normal
        intakeHarvester.setDirection(DcMotorSimple.Direction.REVERSE);
        conveyor.setDirection(DcMotorSimple.Direction.FORWARD);
        launcher.setDirection(DcMotorSimple.Direction.FORWARD); // flip if it spins the wrong way

        // Brake when zero power
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeHarvester.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        conveyor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Start subsystems off
        intakeHarvester.setPower(0);
        conveyor.setPower(0);
        launcher.setPower(0);

        telemetry.addLine("TeleOp READY: drive + Y(intake+conveyor) + X(launcher toggle)");
        telemetry.update();
    }

    @Override
    public void loop() {

        // ========= DRIVE =========
        // NOTE: minus sign so pushing stick forward = robot forward
        float y  = -dead(gamepad1.left_stick_y);
        float x  =  dead(gamepad1.left_stick_x);
        float rx =  dead(gamepad1.right_stick_x);

        // Slow mode toggle (left stick button)
        boolean stickBtn = gamepad1.left_stick_button;
        if (stickBtn && !leftStickPrev) slowMode = !slowMode;
        leftStickPrev = stickBtn;
        speedMult = slowMode ? SLOW_MULT : NORM_MULT;

        // Mecanum math
        float fl = y - x - rx;
        float fr = y + x + rx;
        float bl = y + x - rx;
        float br = y - x + rx;

        float max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                             Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1f) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        front_left.setPower(clip(fl * speedMult, -1, 1));
        front_right.setPower(clip(fr * speedMult, -1, 1));
        back_left.setPower(clip(bl * speedMult, -1, 1));
        back_right.setPower(clip(br * speedMult, -1, 1));

        // ========= INTAKE + CONVEYOR (HOLD Y) =========
        if (gamepad1.y) {
            intakeHarvester.setPower(INTAKE_POWER);
            conveyor.setPower(CONVEYOR_POWER);
        } else {
            intakeHarvester.setPower(0);
            conveyor.setPower(0);
        }

        // ========= LAUNCHER TOGGLE (PRESS X) =========
        boolean xNow = gamepad1.x;
        if (xNow && !xPrev) {
            launcherOn = !launcherOn;  // flip state each press
        }
        xPrev = xNow;

        if (launcherOn) {
            launcher.setPower(LAUNCHER_POWER);
        } else {
            launcher.setPower(0);
        }

        // ========= TELEMETRY =========
        telemetry.addData("Mode", slowMode ? "SLOW" : "NORMAL");
        telemetry.addData("Launcher", launcherOn ? "ON" : "OFF");
        telemetry.update();
    }

    // ========= UTILS =========
    private float dead(float v) {
        return (Math.abs(v) < DEADZONE) ? 0f : v;
    }

    private double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
