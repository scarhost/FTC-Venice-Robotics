package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor.TagFamily;

import java.util.List;

@TeleOp(name = "MAIN_EXE", group = "TeleOp")
public class MAIN_EXE extends OpMode {

    // ===== Drive motors =====
    private DcMotor front_left, front_right, back_left, back_right;

    // ===== Subsystem motors =====
    private DcMotor intakeHarvester; // Y = reverse (hold, also runs conveyor)
    private DcMotor conveyor; // X/B toggle ±0.6 when Y not held

    // ===== Tunables =====
    private static final float OTHER_GAIN = 0.94f; // tiny trim for FR/BR
    private static final float BL_GAIN = 1.00f; // keep BL at 1.0
    private static final float DEADZONE = 0.10f;
    private static final float SLOW_MULT = 0.60f;
    private static final float NORM_MULT = 1.00f;

    private static final double CONVEYOR_SPEED = 0.40; // cap

    // ===== State =====
    private float leftStickX, leftStickY, rightStickX;

    private boolean slowMode = false;
    private boolean leftStickDownPrev = false;
    private float speedMultiplier = NORM_MULT;

    // Conveyor toggle state (active when Y is NOT held)
    private boolean xPrev = false;
    private boolean bPrev = false;
    private double conveyorPowerToggled = 0.0;

    // ===== AprilTag auto-align (hold A) =====
    private static final int TARGET_TAG_ID = 24;
    private static final String WEBCAM_NAME = "Webcam 1"; // change if your config name differs
    private static final double BEARING_TOL_DEG = 2.0; // stop turning when |bearing| <= tol
    private static final double KP_TURN = 0.02; // proportional turn gain (power/deg)
    private static final double MIN_TURN = 0.12; // minimum turn power to overcome stiction
    private static final double MAX_TURN = 0.50; // cap so we don’t whip

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    @Override
    public void init() {
        telemetry.addLine("Initializing… (AprilTag + TeleOp)");
        telemetry.update();

        // Map drive
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");

        // Map subsystems
        intakeHarvester = hardwareMap.get(DcMotor.class, "intakeHarvester");
        conveyor = hardwareMap.get(DcMotor.class, "conveyor");

        // Standard mecanum directions (POS power moves robot forward)
        front_left.setDirection(DcMotor.Direction.FORWARD);
        back_left.setDirection(DcMotor.Direction.FORWARD);
        front_right.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.REVERSE);

        // Subsystems (flip if needed)
        intakeHarvester.setDirection(DcMotor.Direction.FORWARD);
        conveyor.setDirection(DcMotor.Direction.FORWARD);

        // Brake at zero
        for (DcMotor m : new DcMotor[]{front_left, front_right, back_left, back_right}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        intakeHarvester.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        conveyor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Start off
        intakeHarvester.setPower(0);
        conveyor.setPower(0);

        // ===== Vision (AprilTag via webcam) =====
        aprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(TagFamily.TAG_36h11)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setOutputUnits(AprilTagProcessor.OutputUnits.INCH) // units for ftcPose range
                .build();

        WebcamName webcam = hardwareMap.get(WebcamName.class, WEBCAM_NAME);

        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTag)
                .setCamera(webcam)
                .build();

        telemetry.addLine("Initialized ✅ (Hold A to auto-align to Tag 24)");
        telemetry.update();
    }

    @Override
    public void loop() {
        // If A is held -> ALIGN; else -> normal drive + intake/conveyor
        if (gamepad1.a) {
            alignToAprilTag24();
            return; // skip the rest while aligning
        }

        // ===== Normal driving =====
        leftStickY = gamepad1.left_stick_y;
        leftStickX = gamepad1.left_stick_x;
        rightStickX = gamepad1.right_stick_x;

        if (Math.abs(leftStickY) < DEADZONE) leftStickY = 0;
        if (Math.abs(leftStickX) < DEADZONE) leftStickX = 0;
        if (Math.abs(rightStickX) < DEADZONE) rightStickX = 0;

        boolean leftStickDown = gamepad1.left_stick_button;
        if (leftStickDown && !leftStickDownPrev) slowMode = !slowMode;
        leftStickDownPrev = leftStickDown;
        speedMultiplier = slowMode ? SLOW_MULT : NORM_MULT;

        float fl = leftStickY - leftStickX - rightStickX;
        float fr = leftStickY + leftStickX + rightStickX;
        float bl = leftStickY + leftStickX - rightStickX;
        float br = leftStickY - leftStickX + rightStickX;

        float max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                             Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0f) { fl /= max; fr /= max; bl /= max; br /= max; }

        double pfl = clip(fl * OTHER_GAIN * speedMultiplier, -1, 1);
        double pfr = clip(fr * OTHER_GAIN * speedMultiplier, -1, 1);
        double pbl = clip(bl * BL_GAIN * speedMultiplier, -1, 1);
        double pbr = clip(br * OTHER_GAIN * speedMultiplier, -1, 1);

        front_left.setPower(pfl);
        front_right.setPower(pfr);
        back_left.setPower(pbl);
        back_right.setPower(pbr);

        // ===== Intake & Conveyor (A is reserved for ALIGN, so no intake-forward on A) =====
        boolean holdY = gamepad1.y;

        // Intake power (only Y = reverse hold)
        double intakePower = 0.0;
        if (holdY) intakePower = -1.0;
        intakeHarvester.setPower(intakePower);

        // Conveyor logic
        if (holdY) {
            // While Y is held, conveyor follows Y direction at capped speed
            conveyor.setPower(-CONVEYOR_SPEED); // flip sign if needed
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

        telemetry.addData("Mode", slowMode ? "SLOW (0.6x)" : "NORMAL (1.0x)");
        telemetry.addData("Drive", "FL %.2f FR %.2f BL %.2f BR %.2f", pfl, pfr, pbl, pbr);
        telemetry.addData("Intake", "%.1f (Y=-1 hold; A=ALIGN)", intakePower);
        telemetry.addData("Conveyor", "%.2f (Y overrides @±0.60; X:+0.60, B:-0.60)", 
                          holdY ? -CONVEYOR_SPEED : conveyorPowerToggled);
        addTagTelemetry();
        telemetry.update();
    }

    // ===== AprilTag ALIGN =====

    private void alignToAprilTag24() {
        // Stop subsystems while aligning
        intakeHarvester.setPower(0);
        conveyor.setPower(0);

        AprilTagDetection best = getTagById(TARGET_TAG_ID);
        if (best == null || best.ftcPose == null) {
            // no tag -> stop turning
            setTurnPower(0);
            telemetry.addLine("ALIGN: Tag 24 NOT VISIBLE");
            addTagTelemetry();
            telemetry.update();
            return;
        }

        // Use FTC pose bearing (deg). Convention: +bearing means tag is to the LEFT of center.
        double bearingDeg = best.ftcPose.bearing;

        double turnCmd = 0.0;
        if (Math.abs(bearingDeg) > BEARING_TOL_DEG) {
            turnCmd = KP_TURN * bearingDeg; // proportional
            turnCmd = Math.copySign(Math.max(Math.abs(turnCmd), MIN_TURN), turnCmd); // enforce min
            turnCmd = clip(turnCmd, -MAX_TURN, MAX_TURN); // cap
        }

        // Positive turnCmd = rotate CCW (left). Negative = CW (right).
        setTurnPower(turnCmd);

        telemetry.addData("ALIGN", "bearing=%.1f° cmd=%.2f", bearingDeg, turnCmd);
        addTagTelemetry();
        telemetry.update();
    }

    private void setTurnPower(double p) {
        // Tank-style in-place turn
        p = clip(p, -1, 1);
        front_left.setPower(+p);
        back_left.setPower(+p);
        front_right.setPower(-p);
        back_right.setPower(-p);
    }

    private AprilTagDetection getTagById(int id) {
        if (aprilTag == null) return null;
        List<AprilTagDetection> dets = aprilTag.getDetections();
        if (dets == null || dets.isEmpty()) return null;
        AprilTagDetection best = null;
        double bestAbsBearing = Double.MAX_VALUE;
        for (AprilTagDetection d : dets) {
            if (d == null || d.id != id || d.ftcPose == null) continue;
            double ab = Math.abs(d.ftcPose.bearing);
            if (ab < bestAbsBearing) {
                best = d;
                bestAbsBearing = ab;
            }
        }
        return best;
    }

    private void addTagTelemetry() {
        try {
            List<AprilTagDetection> dets = (aprilTag != null) ? aprilTag.getDetections() : null;
            int count = (dets == null) ? 0 : dets.size();
            telemetry.addData("Tags", "#%d seen", count);
            if (dets != null) {
                for (AprilTagDetection d : dets) {
                    if (d == null || d.ftcPose == null) continue;
                    telemetry.addData("Tag", "id=%d bearing=%.1f° range=%.1f in",
                            d.id, d.ftcPose.bearing, d.ftcPose.range);
                }
            }
        } catch (Exception ignored) { }
    }

    private static double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    @Override
    public void stop() {
        // Clean up camera to free resources between runs
        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }
    }
}
