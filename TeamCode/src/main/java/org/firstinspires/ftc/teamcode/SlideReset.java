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
public class SlideReset extends LinearOpMode {
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
    Servo hangLock;

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

        hangLock = hardwareMap.servo.get("hangLock");

        spinnerR = hardwareMap.crservo.get("spinnerR");
        spinnerL = hardwareMap.crservo.get("spinnerL");


        telemetry.addData("System: ", "waiting for initalization");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("System: ", "initialized");
            telemetry.update();



            if(gamepad1.dpad_up){
                rightLift.setPower(1);

            }else if(gamepad1.dpad_down){
                rightLift.setPower(-1);

            }
            else{
                rightLift.setPower(0);

            }

            if(gamepad1.dpad_right){
                leftLift.setPower(1);

            }else if(gamepad1.dpad_left){
                leftLift.setPower(-1);

            }
            else{
                leftLift.setPower(0);

            }



        }
    }
}

