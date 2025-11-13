package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor.TagFamily;

import java.util.List;

@TeleOp(name = "MAIN_EXE", group = "TeleOp")
public class MAIN_EXE extends OpMode {

    // ====== DRIVETRAIN ======
    private DcMotor front_left, front_right, back_left, back_right;

    // ====== SUBSYSTEMS ======
    private DcMotor intakeHarvester; 
    private DcMotor conveyor;

    // ====== DRIVE SETTINGS ======
    private static final float DEADZONE = 0.10f;
    private static final float SLOW_MULT = 0.60f;
    private static final float NORM_MULT = 1.00f;
    private boolean slowMode = false;
    private boolean leftStickPrev = false;
    private float speedMult = NORM_MULT;

    // ====== INTAKE SPEED ======
    private static final double INTAKE_POWER = 1.0;
    private static final double CONVEYOR_POWER = 0.60;

    // ====== APRILTAG SETTINGS ======
    private static final int TARGET_TAG_ID = 24;
    private static final String WEBCAM_NAME = "Webcam 1";
    private static final double BEARING_TOL_DEG = 2.0;
    private static final double KP_TURN = 0.02;
    private static final double MIN_TURN = 0.12;
    private static final double MAX_TURN = 0.50;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    @Override
    public void init() {

        // ---- Map hardware ----
        front_left  = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_left   = hardwareMap.get(DcMotor.class, "back_left");
        back_right  = hardwareMap.get(DcMotor.class, "back_right");

        intakeHarvester = hardwareMap.get(DcMotor.class, "intakeHarvester");
        conveyor        = hardwareMap.get(DcMotor.class, "conveyor");

        // ---- Drivetrain Directions ----
        front_left.setDirection(DcMotor.Direction.FORWARD);
        back_left.setDirection(DcMotor.Direction.FORWARD);
        front_right.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.REVERSE);

        // Reverse intake as you requested
        intakeHarvester.setDirection(DcMotorSimple.Direction.REVERSE);
        conveyor.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{front_left, front_right, back_left, back_right}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        intakeHarvester.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        conveyor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ==== AprilTag Vision ====
        aprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(TagFamily.TAG_36h11)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        WebcamName cam = hardwareMap.get(WebcamName.class, WEBCAM_NAME);

        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTag)
                .setCamera(cam)
                .build();

        telemetry.addLine("Ready: Hold A to align with Tag 24");
        telemetry.update();
    }

    @Override
    public void loop() {

        // ========== AUTO-ALIGN (A HOLD) ==========
        if (gamepad1.a) {
            alignToTag24();
            return;
        }

        // ========== NORMAL DRIVE ==========
        float y = dead(gamepad1.left_stick_y);
        float x = dead(gamepad1.left_stick_x);
        float rx = dead(gamepad1.right_stick_x);

        boolean stickBtn = gamepad1.left_stick_button;
        if (stickBtn && !leftStickPrev) slowMode = !slowMode;
        leftStickPrev = stickBtn;

        speedMult = slowMode ? SLOW_MULT : NORM_MULT;

        float fl = y - x - rx;
        float fr = y + x + rx;
        float bl = y + x - rx;
        float br = y - x + rx;

        float max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1f) { fl /= max; fr /= max; bl /= max; br /= max; }

        front_left.setPower(clip(fl * speedMult, -1, 1));
        front_right.setPower(clip(fr * speedMult, -1, 1));
        back_left.setPower(clip(bl * speedMult, -1, 1));
        back_right.setPower(clip(br * speedMult, -1, 1));

        // ========== INTAKE + CONVEYOR (Y HOLD) ==========
        if (gamepad1.y) {
            intakeHarvester.setPower(INTAKE_POWER);
            conveyor.setPower(CONVEYOR_POWER);
        } else {
            intakeHarvester.setPower(0);
            conveyor.setPower(0);
        }

        telemetry.addData("Mode", slowMode ? "SLOW" : "NORMAL");
        telemetry.update();
    }

    // ========== ALIGNMENT FUNCTIONS ==========
    private void alignToTag24() {
        intakeHarvester.setPower(0);
        conveyor.setPower(0);

        AprilTagDetection tag = findTag(TARGET_TAG_ID);
        if (tag == null || tag.ftcPose == null) {
            setTurn(0);
            telemetry.addLine("ALIGN: Tag 24 not visible");
            telemetry.update();
            return;
        }

        double bearing = tag.ftcPose.bearing;
        double cmd = 0;

        if (Math.abs(bearing) > BEARING_TOL_DEG) {
            cmd = KP_TURN * bearing;
            if (Math.abs(cmd) < MIN_TURN) cmd = Math.copySign(MIN_TURN, cmd);
            cmd = clip(cmd, -MAX_TURN, MAX_TURN);
        }

        setTurn(cmd);

        telemetry.addData("ALIGN", "bearing=%.1f cmd=%.2f", bearing, cmd);
        telemetry.update();
    }

    private AprilTagDetection findTag(int id) {
        List<AprilTagDetection> list = aprilTag.getDetections();
        if (list == null) return null;
        AprilTagDetection best = null;
        double bestAbs = 9999;
        for (AprilTagDetection d : list) {
            if (d.id == id && d.ftcPose != null) {
                double ab = Math.abs(d.ftcPose.bearing);
                if (ab < bestAbs) { bestAbs = ab; best = d; }
            }
        }
        return best;
    }

    private void setTurn(double p) {
        p = clip(p, -1, 1);
        front_left.setPower(+p);
        back_left.setPower(+p);
        front_right.setPower(-p);
        back_right.setPower(-p);
    }

    // ========== UTILS ==========
    private float dead(float v) { return (Math.abs(v) < DEADZONE) ? 0f : v; }
    private double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    @Override
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }
    }
}
