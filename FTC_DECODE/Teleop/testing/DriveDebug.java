package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "DriveDebug", group = "Debug")
public class DriveDebug extends OpMode {

    private DcMotor front_left, front_right, back_left, back_right;

    @Override
    public void init() {
        front_left  = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_left   = hardwareMap.get(DcMotor.class, "back_left");
        back_right  = hardwareMap.get(DcMotor.class, "back_right");

        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Press:");
        telemetry.addLine("  dpad_up    -> front_left");
        telemetry.addLine("  dpad_right -> front_right");
        telemetry.addLine("  dpad_down  -> back_right");
        telemetry.addLine("  dpad_left  -> back_left");
        telemetry.update();
    }

    @Override
    public void loop() {
        double p = 0.3;

        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);

        if (gamepad1.dpad_up) {
            front_left.setPower(p);
            telemetry.addLine("Spinning: front_left");
        } else if (gamepad1.dpad_right) {
            front_right.setPower(p);
            telemetry.addLine("Spinning: front_right");
        } else if (gamepad1.dpad_down) {
            back_right.setPower(p);
            telemetry.addLine("Spinning: back_right");
        } else if (gamepad1.dpad_left) {
            back_left.setPower(p);
            telemetry.addLine("Spinning: back_left");
        }

        telemetry.update();
    }
}
