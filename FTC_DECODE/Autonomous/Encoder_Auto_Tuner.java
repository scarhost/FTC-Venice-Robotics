package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="Encoder_Auto_Tuner", group="Test")
public class Encoder_Auto_Tuner extends LinearOpMode {

    private DcMotor front_left, front_right, back_left, back_right;

    // === Robot constants (tune these) ===
    private static final double TICKS_PER_REV         = 560.0;  // e.g., NeveRest 20 or REV HD Hex 20:1 (~560)
    private static final double WHEEL_DIAMETER_IN     = 2.95;   // 75 mm mecanum wheels
    private static final double EXTERNAL_GEAR_RATIO   = 1.0;
    private static final double STRAFE_CORRECTION     = 1.10;   // tune after strafe test
    private static final double WHEELBASE_DIAMETER_IN = 14.0;   // effective track width; tune for turns

    private static final double MAX_POWER           = 0.6;
    private static final double TIMEOUT_S_PER_FOOT  = 2.5;

    // stop when we’re “close enough” to target (ticks)
    private static final int POS_TOL_TICKS = 10;

    @Override
    public void runOpMode() {
        // map hardware
        front_left  = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_left   = hardwareMap.get(DcMotor.class, "back_left");
        back_right  = hardwareMap.get(DcMotor.class, "back_right");

        // directions (so positive = forward)
        front_left.setDirection(DcMotorSimple.Direction.FORWARD);
        back_left.setDirection(DcMotorSimple.Direction.FORWARD);
        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{front_left, front_right, back_left, back_right}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Optional: set built-in target tolerance if DcMotorEx is available
        setBuiltInToleranceIfSupported(POS_TOL_TICKS);

        telemetry.addLine("Encoder_Auto_Tuner READY");
        telemetry.addData("Ticks per inch", "%.1f", ticksPerInch());
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // === Test sequence ===
        double p = MAX_POWER;

        driveInches(24,  p, getTimeout(24));    // forward 24"
        sleep(250);

        strafeInches(24, p, getTimeout(24));    // strafe right 24"
        sleep(250);

        turnByEncoders(90, p, getTurnTimeout(90)); // turn ~90° right
        sleep(250);

        driveInches(-24, p, getTimeout(24));    // back 24"

        telemetry.addLine("Sequence complete ✅");
        telemetry.update();
        sleep(600);
    }

    // === Movement functions ===

    private void driveInches(double inches, double power, double timeoutS) {
        int dt = ticksFromInches(inches);
        int flT = front_left.getCurrentPosition()  + dt;
        int frT = front_right.getCurrentPosition() + dt;
        int blT = back_left.getCurrentPosition()   + dt;
        int brT = back_right.getCurrentPosition()  + dt;

        setRunToPosition(flT, frT, blT, brT);
        // tiny settle (optional)
        // sleep(10);

        long start = now();
        final double requested = clipAbs(power, 1.0);

        while (opModeIsActive()
                && !allWithin(flT, frT, blT, brT, POS_TOL_TICKS)
                && elapsedS(start) < timeoutS) {
            setAllPower(requested);
            showTelemetry("Drive", flT, frT, blT, brT, requested);
        }
        stopAll();
        setRunUsingEncoder();
    }

    private void strafeInches(double inches, double power, double timeoutS) {
        int dt  = (int)Math.round(ticksFromInches(inches) * STRAFE_CORRECTION);
        int flT = front_left.getCurrentPosition()  + dt;
        int frT = front_right.getCurrentPosition() - dt;
        int blT = back_left.getCurrentPosition()   - dt;
        int brT = back_right.getCurrentPosition()  + dt;

        setRunToPosition(flT, frT, blT, brT);
        // sleep(10);

        long start = now();
        final double requested = clipAbs(power, 1.0);

        while (opModeIsActive()
                && !allWithin(flT, frT, blT, brT, POS_TOL_TICKS)
                && elapsedS(start) < timeoutS) {
            setAllPower(requested);
            showTelemetry("Strafe", flT, frT, blT, brT, requested);
        }
        stopAll();
        setRunUsingEncoder();
    }

    private void turnByEncoders(double deltaDeg, double power, double timeoutS) {
        double turnCirc = Math.PI * WHEELBASE_DIAMETER_IN;
        double arcLen   = (deltaDeg / 360.0) * turnCirc;
        int dt = ticksFromInches(arcLen);

        // Right turn: left wheels backward, right wheels forward
        int flT = front_left.getCurrentPosition()  - dt;
        int frT = front_right.getCurrentPosition() + dt;
        int blT = back_left.getCurrentPosition()   - dt;
        int brT = back_right.getCurrentPosition()  + dt;

        setRunToPosition(flT, frT, blT, brT);
        // sleep(10);

        long start = now();
        final double requested = clipAbs(power, 1.0);

        while (opModeIsActive()
                && !allWithin(flT, frT, blT, brT, POS_TOL_TICKS)
                && elapsedS(start) < timeoutS) {
            setAllPower(requested);
            showTelemetry("Turn", flT, frT, blT, brT, requested);
        }
        stopAll();
        setRunUsingEncoder();
    }

    // === Utilities ===

    private void setBuiltInToleranceIfSupported(int tolTicks) {
        try {
            if (front_left  instanceof DcMotorEx) ((DcMotorEx) front_left ).setTargetPositionTolerance(tolTicks);
            if (front_right instanceof DcMotorEx) ((DcMotorEx) front_right).setTargetPositionTolerance(tolTicks);
            if (back_left   instanceof DcMotorEx) ((DcMotorEx) back_left  ).setTargetPositionTolerance(tolTicks);
            if (back_right  instanceof DcMotorEx) ((DcMotorEx) back_right ).setTargetPositionTolerance(tolTicks);
        } catch (Exception ignored) {
            // Safe to ignore if the hardware class doesn’t support it
        }
    }

    private double ticksPerInch() {
        return (TICKS_PER_REV * EXTERNAL_GEAR_RATIO) / (Math.PI * WHEEL_DIAMETER_IN);
    }

    private int ticksFromInches(double inches) {
        return (int)Math.round(inches * ticksPerInch());
    }

    private void setRunToPosition(int flT, int frT, int blT, int brT) {
        front_left.setTargetPosition(flT);
        front_right.setTargetPosition(frT);
        back_left.setTargetPosition(blT);
        back_right.setTargetPosition(brT);

        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void setRunUsingEncoder() {
        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setAllPower(double requested) {
        double p = Math.min(Math.abs(requested), MAX_POWER);
        front_left.setPower(p);
        front_right.setPower(p);
        back_left.setPower(p);
        back_right.setPower(p);
    }

    private void stopAll() { setAllPower(0); }

    private boolean allWithin(int flT, int frT, int blT, int brT, int tol) {
        return withinTolerance(flT, front_left.getCurrentPosition(), tol)
            && withinTolerance(frT, front_right.getCurrentPosition(), tol)
            && withinTolerance(blT, back_left.getCurrentPosition(), tol)
            && withinTolerance(brT, back_right.getCurrentPosition(), tol);
    }

    private boolean withinTolerance(int target, int current, int tolTicks) {
        return Math.abs(target - current) <= tolTicks;
    }

    // distance-based timeout helper
    private double getTimeout(double inches) {
        return Math.max(1.0, (Math.abs(inches) / 12.0) * TIMEOUT_S_PER_FOOT);
    }

    // turn timeout scaled from arc length
    private double getTurnTimeout(double degrees) {
        double arcLen = (Math.abs(degrees) / 360.0) * (Math.PI * WHEELBASE_DIAMETER_IN);
        return Math.max(1.0, (arcLen / 12.0) * TIMEOUT_S_PER_FOOT);
    }

    private long now() { return System.currentTimeMillis(); }
    private double elapsedS(long startMs) { return (now() - startMs) / 1000.0; }

    private void showTelemetry(String label, int flT, int frT, int blT, int brT, double requestedPower) {
        telemetry.addData(label, "Requested=%.2f  (Capped at %.2f)", requestedPower, MAX_POWER);
        telemetry.addData("Targets", "FL:%d  FR:%d  BL:%d  BR:%d", flT, frT, blT, brT);
        telemetry.addData("Now    ", "FL:%d  FR:%d  BL:%d  BR:%d",
                front_left.getCurrentPosition(),
                front_right.getCurrentPosition(),
                back_left.getCurrentPosition(),
                back_right.getCurrentPosition());
        telemetry.update();
    }

    private static double clipAbs(double v, double max) {
        return Math.copySign(Math.min(Math.abs(v), max), v);
    }
}
