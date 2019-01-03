package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

@TeleOp
@Disabled
public class DiscoveryDay extends LinearOpMode {
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;

    public void runOpMode() throws InterruptedException{
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight.setDirection(Direction.REVERSE);
        motorBackRight.setDirection(Direction.REVERSE);

        telemetry.addData("System: ", "Waiting for initialization");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("System: ", "Initialized");
            telemetry.update();
            motorFrontRight.setPower(gamepad1.right_stick_y);
            motorFrontLeft.setPower(gamepad1.left_stick_y);
            motorBackRight.setPower(gamepad1.right_stick_y);
            motorBackLeft.setPower(gamepad1.left_stick_y);
        }

    }
}
