package org.firstinspires.ftc.teamcode.OutreachStuff;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

@TeleOp

public class DiscoveryDay extends LinearOpMode {
    private DcMotor motorRight;
    private DcMotor motorLeft;


    public void runOpMode() throws InterruptedException{
        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorLeft = hardwareMap.dcMotor.get("motorLeft");

        motorRight.setDirection(Direction.REVERSE);

        telemetry.addData("System: ", "Waiting for initialization");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("System: ", "Initialized");
            telemetry.update();
            motorRight.setPower(gamepad1.right_stick_y);
            motorLeft.setPower(gamepad1.left_stick_y);
        }

    }
}
