package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.Servo;

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
    DcMotor intakeDump;
    DcMotor intakeSlides;


    CRServo spinnerR;
    CRServo spinnerL;
    Servo dumper;

    //hello boom yo ting goes skrrra
    public void runOpMode() throws InterruptedException {
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        leftLift = hardwareMap.dcMotor.get("leftLift");
        rightLift = hardwareMap.dcMotor.get("rightLift");

        intakeDump = hardwareMap.dcMotor.get("intakeDump");
        intakeSlides = hardwareMap.dcMotor.get("intakeSlides");

        motorBackLeft.setDirection(Direction.REVERSE);
        motorFrontLeft.setDirection(Direction.REVERSE);

        dumper = hardwareMap.servo.get("dumper");

        spinnerR = hardwareMap.crservo.get("spinnerR");
        spinnerL = hardwareMap.crservo.get("spinnerL");


        telemetry.addData("System: ", "waiting for initalization");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("System: ", "initialized");
            telemetry.update();

            if(gamepad1.right_bumper){
                spinnerL.setPower(1);
                spinnerR.setPower(-1);
            }
            else if(gamepad1.left_bumper){
                spinnerL.setPower(-1);
                spinnerR.setPower(1);
            }
            else{
                spinnerL.setPower(0);
                spinnerR.setPower(0);
            }

            if(gamepad1.dpad_up){
                rightLift.setPower(1);
                leftLift.setPower(-0.7);
            }else if(gamepad1.dpad_down){
                rightLift.setPower(-1);
                leftLift.setPower(0.3);
            }
            else{
                rightLift.setPower(0);
                leftLift.setPower(0);
            }
            if(gamepad1.a){
                dumper.setPosition(-1);
            }else{
                dumper.setPosition(0.85);
            }
            intakeSlides.setPower(gamepad2.left_stick_y);

            if(gamepad2.right_stick_y<0){
                intakeDump.setPower(0.8*gamepad2.right_stick_y);
            }
            else{
                intakeDump.setPower(0.2*gamepad2.right_stick_y);
            }



            motorFrontRight.setPower(gamepad1.right_stick_y);
            motorFrontLeft.setPower(gamepad1.left_stick_y);
            motorBackRight.setPower(gamepad1.right_stick_y);
            motorBackLeft.setPower(gamepad1.left_stick_y);

        }
    }
}
