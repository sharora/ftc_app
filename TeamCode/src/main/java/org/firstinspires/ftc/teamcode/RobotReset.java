package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by sharma on 10/14/18.
 */
@TeleOp
public class RobotReset extends LinearOpMode {
    //boom-ting

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    DcMotor hangingMotor;
    DcMotor verticalLift;
    DcMotor intakeDump;
    DcMotor intakeSlides;

    DigitalChannel mgLimVert;


    CRServo spinner;

    Servo dumper1;
    Servo dumper2;

    //hello boom yo ting goes skrrra
    public void runOpMode() throws InterruptedException {
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        hangingMotor = hardwareMap.dcMotor.get("hangingMotor");
        verticalLift = hardwareMap.dcMotor.get("verticalLift");
        verticalLift.setDirection(Direction.REVERSE);


        intakeDump = hardwareMap.dcMotor.get("intakeDump");

        mgLimVert = hardwareMap.digitalChannel.get("mgLimVert");


        intakeSlides = hardwareMap.dcMotor.get("intakeSlides");
        intakeSlides.setDirection(Direction.REVERSE);

        motorBackLeft.setDirection(Direction.REVERSE);
        motorFrontLeft.setDirection(Direction.REVERSE);

        dumper1 = hardwareMap.servo.get("dumper1");
        dumper2 = hardwareMap.servo.get("dumper2");

        spinner = hardwareMap.crservo.get("spinner");

        telemetry.addData("System: ", "waiting for initalization");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("System: ", "initialized");
            telemetry.update();



            if(gamepad1.dpad_up){
                hangingMotor.setPower(1);

            }else if(gamepad1.dpad_down){
                hangingMotor.setPower(-1);

            }
            else{
                hangingMotor.setPower(0);

            }

            verticalLift.setPower(gamepad1.left_stick_y);



        }
    }
}

