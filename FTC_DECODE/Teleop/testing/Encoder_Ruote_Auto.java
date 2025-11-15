package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="Encoder_Route_Auto", group="Test")
public class Encoder_Route_Auto extends LinearOpMode {

    private DcMotor front_left, front_right, back_left, back_right;

    // === Robot constants (calibrated so your inches/degrees feel correct) ===
    private static final double TICKS_PER_REV         = 560.0;   // NeveRest 20 / REV HD Hex 20:1
    private static final double WHEEL_DIAMETER_IN     = 3.54;    // effective dia (scaled from 2.95 by ~1.20)
    private static final double EXTERNAL_GEAR_RATIO   = 1.0;
    private static final double STRAFE_CORRECTION     = 1.10;    // strafing fudge (tune only if strafe is off)
    private static final double WHEELBASE_DIAMETER_IN = 16.8;    // effective track (scaled from 14.0 by ~1.20)

    private static final double MAX_POWER           = 0.6;
    private static final double TIMEOUT_S_PER_FOOT  = 2.5;

    // stop when we’re “close enough” to target (ticks)
    private static final int POS_TOL_TICKS = 10;

    // === Sign convention for THIS robot (your field truth) ===
    // driveInches(negative)   -> FORWARD
    // driveInches(positive)   -> BACKWARD
    // strafeInches(negative)  -> RIGHT
    // strafeInches(positive)  -> LEFT
    // turnByEncoders(negative)-> RIGHT turn

    // === Route constants (like your RR plan) ===
    // One common left / right turn magnitude:
    private static final double TURN_LEFT_DEG   = -85.0;   // CCW
    private static final double TURN_RIGHT_DEG  = 85.0;  // CW

    // Leg distances (inches). Positive/negative per convention above.
    private static final double BACK_PRELOAD_IN    = 36.0;

    private static final double STRAFE_LEFT_1_IN   = 15.0;
    private static final double FWD_1_IN           = -36.0;     // forward is NEGATIVE; we’ll pass -14 below
    private static final double BACK_1_IN          = -36.0;    // back is POSITIVE; we’ll pass +12 below (see calls)
    private static final double STRAFE_RIGHT_1_IN  = -15.0;

    private static final double STRAFE_LEFT_2_IN   = 35.0;
    private static final double FWD_2_IN           = -36.0;
    private static final double BACK_2_IN          = -36.0;
    private static final double STRAFE_RIGHT_2_IN  = -30.0;

    private static final double STRAFE_LEFT_3_IN   = 55.0;
    private static final double FWD_3_IN           = -36.0;
    private static final double BACK_3_IN          = -36.0;
    private static final double STRAFE_RIGHT_3_IN  = -55.0;

    @Override
    public void runOpMode() {
        // map hardware
        front_left  = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_left   = hardwareMap.get(DcMotor.class, "back_left");
        back_right  = hardwareMap.get(DcMotor.class, "back_right");

        // directions (so POSITIVE POWER drives robot forward in teleop sense)
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

        telemetry.addLine("Encoder_Route_Auto READY");
        telemetry.addData("Ticks/in", "%.2f", ticksPerInch());
        telemetry.addData("Sign", "FWD=neg, RIGHT strafe=neg, RIGHT turn=neg");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // === Route sequence (no Road Runner; pure encoders) ===
        final double p = MAX_POWER;

        // -------- PRELOAD BACKUP --------
        driveInches(BACK_PRELOAD_IN, p, getTimeout(BACK_PRELOAD_IN));
        sleep(200);
        // Preload end
        // GPP Intake start
        turnByEncoders(TURN_LEFT_DEG, p, getTurnTimeout(TURN_LEFT_DEG));
        sleep(200);
        strafeInches(STRAFE_LEFT_1_IN, p, getTimeout(STRAFE_LEFT_1_IN));
        sleep(200);
        driveInches(FWD_1_IN, p, getTimeout(FWD_1_IN));
        sleep(200);
        driveInches(-BACK_1_IN, p, getTimeout(BACK_1_IN)); // back is positive -> pass negative to go backward
        sleep(200);
        // GPP Intake done

        // Go to Shoting position
        strafeInches(STRAFE_RIGHT_1_IN, p, getTimeout(STRAFE_RIGHT_1_IN));
        sleep(200);
        turnByEncoders(TURN_RIGHT_DEG, p, getTurnTimeout(TURN_RIGHT_DEG));
        sleep(200);
        // GPP Shoot out
        
        // PGP Intake start
        turnByEncoders(TURN_LEFT_DEG, p, getTurnTimeout(TURN_LEFT_DEG));
        sleep(200);
        strafeInches(STRAFE_LEFT_2_IN, p, getTimeout(STRAFE_LEFT_2_IN));
        sleep(200);
        driveInches(FWD_2_IN, p, getTimeout(FWD_2_IN));
        sleep(200);
        driveInches(-BACK_2_IN, p, getTimeout(BACK_2_IN)); // back is positive -> pass negative to go backward
        sleep(200);
        // PGP Intake done

        // Go to Shoting position
        strafeInches(STRAFE_RIGHT_2_IN, p, getTimeout(STRAFE_RIGHT_2_IN));
        sleep(200);
        turnByEncoders(TURN_RIGHT_DEG, p, getTurnTimeout(TURN_RIGHT_DEG));
        sleep(200);
        // PGP Shoot out

        // PPG Intake start
        turnByEncoders(TURN_LEFT_DEG, p, getTurnTimeout(TURN_LEFT_DEG));
        sleep(200);
        strafeInches(STRAFE_LEFT_3_IN, p, getTimeout(STRAFE_LEFT_3_IN));
        sleep(200);
        driveInches(FWD_3_IN, p, getTimeout(FWD_3_IN));
        sleep(200);
        driveInches(-BACK_3_IN, p, getTimeout(BACK_3_IN)); // back is positive -> pass negative to go backward
        sleep(200);
        // PPG Intake done

        // Go to Shoting position
        strafeInches(STRAFE_RIGHT_3_IN, p, getTimeout(STRAFE_RIGHT_3_IN));
        sleep(200);
        turnByEncoders(TURN_RIGHT_DEG, p, getTurnTimeout(TURN_RIGHT_DEG));
        sleep(200);

        driveInches(FWD_1_IN, p, getTimeout(FWD_1_IN));
        // TODO: preload action
        telemetry.addLine("Route complete ✅");
        telemetry.update();
        sleep(500);
    }

    // === Movement functions ===

    private void driveInches(double inches, double power, double timeoutS) {
        // NEGATIVE inches = forward on this robot (kept intentionally)
        final int dt = ticksFromInches(inches);
        final int flT = front_left.getCurrentPosition()  + dt;
        final int frT = front_right.getCurrentPosition() + dt;
        final int blT = back_left.getCurrentPosition()   + dt;
        final int brT = back_right.getCurrentPosition()  + dt;

        setRunToPosition(flT, frT, blT, brT);

        long start = now();
        final double requested = clipAbs(power, 1.0);
        while (opModeIsActive()
                && !allWithin(flT, frT, blT, brT, POS_TOL_TICKS)
                && elapsedS(start) < timeoutS) {
            setAllPower(requested);
            showTelemetry("Drive(FWD=neg)", flT, frT, blT, brT, requested);
        }
        stopAll();
        setRunUsingEncoder();
    }

    private void strafeInches(double inches, double power, double timeoutS) {
        // NEGATIVE inches = strafe right on this robot (kept intentionally)
        final int dt  = (int)Math.round(ticksFromInches(inches) * STRAFE_CORRECTION);
        final int flT = front_left.getCurrentPosition()  + dt;
        final int frT = front_right.getCurrentPosition() - dt;
        final int blT = back_left.getCurrentPosition()   - dt;
        final int brT = back_right.getCurrentPosition()  + dt;

        setRunToPosition(flT, frT, blT, brT);

        long start = now();
        final double requested = clipAbs(power, 1.0);
        while (opModeIsActive()
                && !allWithin(flT, frT, blT, brT, POS_TOL_TICKS)
                && elapsedS(start) < timeoutS) {
            setAllPower(requested);
            showTelemetry("Strafe(RIGHT=neg)", flT, frT, blT, brT, requested);
        }
        stopAll();
        setRunUsingEncoder();
    }

    private void turnByEncoders(double deltaDeg, double power, double timeoutS) {
        // NEGATIVE = turn right; POSITIVE = turn left (your convention)
        final double turnCirc = Math.PI * WHEELBASE_DIAMETER_IN;
        final double arcLen   = (deltaDeg / 360.0) * turnCirc;
        final int dt = ticksFromInches(arcLen);

        // Right turn: left wheels backward, right wheels forward (signs handled by -dt/+dt)
        final int flT = front_left.getCurrentPosition()  - dt;
        final int frT = front_right.getCurrentPosition() + dt;
        final int blT = back_left.getCurrentPosition()   - dt;
        final int brT = back_right.getCurrentPosition()  + dt;

        setRunToPosition(flT, frT, blT, brT);

        long start = now();
        final double requested = clipAbs(power, 1.0);
        while (opModeIsActive()
                && !allWithin(flT, frT, blT, brT, POS_TOL_TICKS)
                && elapsedS(start) < timeoutS) {
            setAllPower(requested);
            showTelemetry("Turn(RIGHT=neg)", flT, frT, blT, brT, requested);
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
        } catch (Exception ignored) { }
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
        telemetry.addData(label, "Req=%.2f  (cap=%.2f)", requestedPower, MAX_POWER);
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
