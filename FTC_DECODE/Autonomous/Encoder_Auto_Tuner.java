package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="Encoder_Auto_Tuner", group="Test")
public class Encoder_Auto_Tuner extends LinearOpMode {

    private DcMotor front_left, front_right, back_left, back_right;

    // === Robot constants (tune these) ===
    private static final double TICKS_PER_REV         = 560.0;  // REV HD Hex 20:1
    private static final double WHEEL_DIAMETER_IN     = 2.95;   // 75 mm mecanum wheels
    private static final double EXTERNAL_GEAR_RATIO   = 1.0;
    private static final double STRAFE_CORRECTION     = 1.10;   // tune after strafe test
    private static final double WHEELBASE_DIAMETER_IN = 14.0;   // tune for accurate turns

    private static final double MAX_POWER           = 0.6;
    private static final double TIMEOUT_S_PER_FOOT  = 2.5;

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

        telemetry.addLine("Encoder_Auto_Tuner READY");
        telemetry.addData("Ticks per inch", "%.1f", ticksPerInch());
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // === Test sequence ===
        driveInches(24, MAX_POWER, getTimeout(24));   // forward 24"
        sleep(250);

        strafeInches(24, MAX_POWER, getTimeout(24));  // strafe right 24"
        sleep(250);

        turnByEncoders(90, MAX_POWER, 2.5);           // turn ~90° right
        sleep(250);

        driveInches(-24, MAX_POWER, getTimeout(24));  // back 24"

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
        long start = now();
        power = Math.min(Math.abs(power), 1.0);

        while (opModeIsActive() && anyBusy() && elapsedS(start) < timeoutS) {
            setAllPower(power);
            showTelemetry("Drive", flT, frT, blT, brT);
        }
        stopAll();
        setRunUsingEncoder();
    }

    private void strafeInches(double inches, double power, double timeoutS) {
        int dt = (int)Math.round(ticksFromInches(inches) * STRAFE_CORRECTION);
        int flT = front_left.getCurrentPosition()  + dt;
        int frT = front_right.getCurrentPosition() - dt;
        int blT = back_left.getCurrentPosition()   - dt;
        int brT = back_right.getCurrentPosition()  + dt;

        setRunToPosition(flT, frT, blT, brT);
        long start = now();
        power = Math.min(Math.abs(power), 1.0);

        while (opModeIsActive() && anyBusy() && elapsedS(start) < timeoutS) {
            setAllPower(power);
            showTelemetry("Strafe", flT, frT, blT, brT);
        }
        stopAll();
        setRunUsingEncoder();
    }

    private void turnByEncoders(double deltaDeg, double power, double timeoutS) {
        double turnCirc = Math.PI * WHEELBASE_DIAMETER_IN;
        double arcLen   = (deltaDeg / 360.0) * turnCirc;
        int dt = ticksFromInches(arcLen);

        int flT = front_left.getCurrentPosition()  - dt;
        int frT = front_right.getCurrentPosition() + dt;
        int blT = back_left.getCurrentPosition()   - dt;
        int brT = back_right.getCurrentPosition()  + dt;

        setRunToPosition(flT, frT, blT, brT);
        long start = now();
        power = Math.min(Math.abs(power), 1.0);

        while (opModeIsActive() && anyBusy() && elapsedS(start) < timeoutS) {
            setAllPower(power);
            showTelemetry("Turn", flT, frT, blT, brT);
        }
        stopAll();
        setRunUsingEncoder();
    }

    // === Utilities ===

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

    private void setAllPower(double p) {
        p = Math.min(Math.abs(p), MAX_POWER);
        front_left.setPower(p);
        front_right.setPower(p);
        back_left.setPower(p);
        back_right.setPower(p);
    }

    private void stopAll() { setAllPower(0); }

    private boolean anyBusy() {
        return front_left.isBusy() || front_right.isBusy() || back_left.isBusy() || back_right.isBusy();
    }

    private long now() { return System.currentTimeMillis(); }
    private double elapsedS(long startMs) { return (now() - startMs) / 1000.0; }

    private void showTelemetry(String label, int flT, int frT, int blT, int brT) {
        telemetry.addData(label, "Power=%.2f", MAX_POWER);
        telemetry.addData("Targets", "FL:%d  FR:%d  BL:%d  BR:%d", flT, frT, blT, brT);
        telemetry.addData("Now    ", "FL:%d  FR:%d  BL:%d  BR:%d",
                front_left.getCurrentPosition(),
                front_right.getCurrentPosition(),
                back_left.getCurrentPosition(),
                back_right.getCurrentPosition());
        telemetry.update();
    }
}
