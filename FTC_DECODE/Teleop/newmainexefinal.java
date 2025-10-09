package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.util.Range; // if you prefer Range.clip

@TeleOp(name = "MAIN_EXE_FINAL", group = "TeleOp")
public class MAIN_EXE extends OpMode {
    // Motors
    private DcMotor front_left, front_right, back_left, back_right;
    private DcMotor intakeHarvester; // <<< ADDED intake motor

    // <<< Paste your discovered live-trim here >>>
    private static final float OTHER_GAIN = 0.94f; // affects FL/FR/BR; BL stays 1.00

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
        telemetry.addData("Status", "Initialized (Final)");

        front_left = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_left   = hardwareMap.get(DcMotor.class, "back_left");
        back_right  = hardwareMap.get(DcMotor.class, "back_right");

        intakeHarvester = hardwareMap.get(DcMotor.class, "intakeHarvester"); // <<< INIT

        // directions (so +power = forward)
        front_left.setDirection(DcMotor.Direction.FORWARD);
        back_left.setDirection(DcMotor.Direction.FORWARD);
        front_right.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.REVERSE);
        intakeHarvester.setDirection(DcMotor.Direction.FORWARD); // adjust if needed

        // stop quickly when power=0 (BRAKE). Change to FLOAT if you prefer coasting.
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeHarvester.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // (Optional) open-loop:
        // front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // intakeHarvester.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // >>> Intake Harvester Controls <<<
        if (gamepad1.x) {
            intakeHarvester.setPower(1.0);
        } else if (gamepad1.a) {
            intakeHarvester.setPower(-1.0);
        } else {
            intakeHarvester.setPower(0.0);
        }

        // Telemetry
        telemetry.addData("Fixed otherGain (FL/FR/BR)", "%.2f", OTHER_GAIN);
        telemetry.addData("Mode", slowMode ? "SLOW (0.6x)" : "NORMAL (1.0x)");
        telemetry.addData("Intake", gamepad1.x ? "IN" : (gamepad1.a ? "OUT" : "OFF"));
