package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

/**
 * Created by sharma on 10/14/18.
 */
@TeleOp
public class servo_test extends LinearOpMode {
    //boom-ting

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    DcMotor leftLift;
    DcMotor rightLift;

    //hello boom yo ting goes skrrra
    public void runOpMode() throws InterruptedException {
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        leftLift = hardwareMap.dcMotor.get("leftLift");
        rightLift = hardwareMap.dcMotor.get("rightLift");

        motorBackLeft.setDirection(Direction.REVERSE);
        motorFrontLeft.setDirection(Direction.REVERSE);


        telemetry.addData("System: ", "waiting for initalization");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("System: ", "initialized");
            telemetry.update();

            if(gamepad1.dpad_up){
                rightLift.setPower(0.5);
                leftLift.setPower(-0.5);
            }else if(gamepad1.dpad_down){
                rightLift.setPower(-0.5);
                leftLift.setPower(0.5);
            }
            else{
                rightLift.setPower(0);
                leftLift.setPower(0);
            }
//            if(gamepad1.dpad_right){
//
//                leftLift.setPower(-0.5);
//            }else if(gamepad1.dpad_left){
//
//                leftLift.setPower(0.5);
//            }
//            else{
//
//                leftLift.setPower(0);
//            }

            motorFrontRight.setPower(gamepad1.right_stick_y);
            motorFrontLeft.setPower(gamepad1.left_stick_y);
            motorBackRight.setPower(gamepad1.right_stick_y);
            motorBackLeft.setPower(gamepad1.left_stick_y);

        }
    }
}
