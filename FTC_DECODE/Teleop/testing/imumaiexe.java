package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "MAIN_EXE", group = "TeleOp")
public class MAIN_EXE extends OpMode {

    // ====== DRIVETRAIN ======
    private DcMotor front_left, front_right, back_left, back_right;

    // ====== SUBSYSTEMS ======
    private DcMotor intakeHarvester;
    private DcMotor conveyor;
    private DcMotor launcher;

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

    // ====== IMU HEADING HOLD ======
    private IMU imu;
    private double targetHeadingDeg = 0.0;

    // when |rx| > this, we assume driver is intentionally turning
    private static final double ROT_DEADZONE     = 0.05;
    // how strong to correct heading (deg → power)
    private static final double HEADING_KP       = 0.02;   // tune 0.015–0.03
    private static final double MAX_HEADING_CORR = 0.4;    // clamp correction

    @Override
    public void init() {

        // ---- Map hardware (names must match configuration) ----
        front_left  = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_left   = hardwareMap.get(DcMotor.class, "back_left");
        back_right  = hardwareMap.get(DcMotor.class, "back_right");

        intakeHarvester = hardwareMap.get(DcMotor.class, "intakeHarvester");
        conveyor        = hardwareMap.get(DcMotor.class, "conveyor");
        launcher        = hardwareMap.get(DcMotor.class, "launcher");

        // ---- Drivetrain directions ----
        front_left.setDirection(DcMotor.Direction.FORWARD);
        back_left.setDirection(DcMotor.Direction.FORWARD);
        front_right.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.REVERSE);

        // Intake reversed (you wanted this), others normal
        intakeHarvester.setDirection(DcMotorSimple.Direction.REVERSE);
        conveyor.setDirection(DcMotorSimple.Direction.FORWARD);
        launcher.setDirection(DcMotorSimple.Direction.FORWARD); // flip if needed

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

        // ---- IMU ----
        imu = hardwareMap.get(IMU.class, "imu"); // make sure name matches config
        imu.initialize(new IMU.Parameters());
        targetHeadingDeg = getHeadingDeg();      // lock current heading as target

        telemetry.addLine("TeleOp READY");
        telemetry.addLine("Y = intake+conveyor | X = launcher toggle");
        telemetry.addLine("IMU heading hold active when not turning");
        telemetry.update();
    }

    @Override
    public void loop() {

        // ========= READ STICKS =========
        // minus sign so pushing stick forward = robot forward
        float yRaw  = gamepad1.left_stick_y;
        float xRaw  = gamepad1.left_stick_x;
        float rxRaw = gamepad1.right_stick_x;

        float y  = -dead(yRaw);          // forward/back
        float x  =  dead(xRaw);          // strafe
        float rx =  dead(rxRaw);         // rotation command

        // ========= IMU HEADING HOLD (modify rx) =========
        double currentHeading = getHeadingDeg();

        if (Math.abs(rx) > ROT_DEADZONE) {
            // driver is intentionally turning -> update target to new heading
            targetHeadingDeg = currentHeading;
        } else {
            // no manual turn -> auto-correct heading toward target
            double errorDeg = angleErrorDeg(targetHeadingDeg, currentHeading);
            double corr = HEADING_KP * errorDeg;

            // clamp correction
            if (Math.abs(corr) > MAX_HEADING_CORR) {
                corr = Math.copySign(MAX_HEADING_CORR, corr);
            }

            rx += (float)corr;  // inject correction into rotation
        }

        // ========= SPEED MODE (normal / slow) =========
        boolean stickBtn = gamepad1.left_stick_button;
        if (stickBtn && !leftStickPrev) slowMode = !slowMode;
        leftStickPrev = stickBtn;
        speedMult = slowMode ? SLOW_MULT : NORM_MULT;

        // ========= MECANUM DRIVE =========
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
            launcherOn = !launcherOn;  // flip on press
        }
        xPrev = xNow;

        if (launcherOn) {
            launcher.setPower(LAUNCHER_POWER);
        } else {
            launcher.setPower(0);
        }

        // ========= TELEMETRY =========
        telemetry.addData("Mode", slowMode ? "SLOW" : "NORMAL");
        telemetry.addData("Heading", "%.1f°  (target=%.1f°)", currentHeading, targetHeadingDeg);
        telemetry.addData("Launcher", launcherOn ? "ON" : "OFF");
        telemetry.update();
    }

    // ========= IMU HELPERS =========

    private double getHeadingDeg() {
        if (imu == null) return 0.0;
        YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
        return ypr.getYaw(AngleUnit.DEGREES);
    }

    // Smallest signed difference between two angles (deg) in range (-180, 180]
    private double angleErrorDeg(double target, double current) {
        double error = target - current;
        while (error > 180)  error -= 360;
        while (error <= -180) error += 360;
        return error;
    }

    // ========= UTILS =========
    private float dead(float v) {
        return (Math.abs(v) < DEADZONE) ? 0f : v;
    }

    private double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
