package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.RobotConstants;

/**
 * Created by sharma on 10/14/18.
 *
 * This class is for the teleop period in the robot game and has a continuous loop that maps
 * controller input to robot actions. Read the names of variables and refer to the design
 * to know which buttons control what actions.
 */
@TeleOp
@Disabled
public class Teleop extends LinearOpMode {
    //boom-ting

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    DcMotor hangingMotor;
    DcMotor verticalLift;
    ServoImplEx intakeDump;
    DcMotor intakeSlides;

    DigitalChannel mgLimVert;
    private Servo teamMarkerServo;


    DcMotor spinner;

    Servo dumper1;
    Servo dumper2;
    public ElapsedTime time = new ElapsedTime();

    public void runOpMode() throws InterruptedException {
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        hangingMotor = hardwareMap.dcMotor.get("hangingMotor");
        verticalLift = hardwareMap.dcMotor.get("verticalLift");
        verticalLift.setDirection(Direction.REVERSE);
        verticalLift.setMode(RunMode.RUN_TO_POSITION);
        hangingMotor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);



        mgLimVert = hardwareMap.digitalChannel.get("mgLimVert");

        intakeDump = hardwareMap.get(ServoImplEx.class,"intakeDump");

        intakeSlides = hardwareMap.dcMotor.get("intakeSlides");

        teamMarkerServo = hardwareMap.servo.get("teamMarker");

        teamMarkerServo.setPosition(0);


        intakeSlides.setMode(RunMode.RUN_TO_POSITION);


        motorBackLeft.setDirection(Direction.REVERSE);
        motorFrontLeft.setDirection(Direction.REVERSE);

        dumper1 = hardwareMap.servo.get("dumper1");
        dumper2 = hardwareMap.servo.get("dumper2");

        spinner = hardwareMap.dcMotor.get("spinner");


        double prevtime = 0;


        telemetry.addData("S=ystem: ", "waiting to be run");
        telemetry.update();
        waitForStart();
        intakeDump.setPwmEnable();

        while (opModeIsActive()) {
            teamMarkerServo.setPosition(0);
            if(gamepad1.right_trigger>0){
                spinner.setPower(1);
            }
            else if(gamepad1.left_trigger>0){
                spinner.setPower(-1);
            }
            else{
                spinner.setPower(0);
            }

            if(gamepad2.dpad_up){
                hangingMotor.setPower(1);
            }
            else if(gamepad2.dpad_down){
                hangingMotor.setPower(-1);
            }
            else{
                hangingMotor.setPower(0);
            }
//
//


            //sends the mechanism to bottom position when right bumper is pressed
            if(gamepad2.right_trigger>0){
                intakeDump.setPosition(0.75);
            }
            //sends the mechanism to the top position when left bumper is pressed
            else if(gamepad2.left_trigger>0){
                intakeDump.setPosition(0.21);
            }
            //if no buttons are pressed thanx the mechanism is sent to neutral position
            else{
                intakeDump.setPosition(0.5);
            }




            if(gamepad1.b){
                verticalLift.setTargetPosition(-1930);
                verticalLift.setPower(1);
                if(verticalLift.getCurrentPosition()<-50){
                    dumper2.setPosition(dumper2.getPosition() + 0.045*(0.78-dumper2.getPosition()));
                    dumper1.setPosition(-(dumper1.getPosition() + 0.045*(0.78-dumper1.getPosition())));
                }


            }else if(gamepad1.x){
                dumper2.setPosition(0.4);
                dumper1.setPosition(-0.4);
            }
            else if(gamepad2.y){
                verticalLift.setTargetPosition(-1620);
                verticalLift.setPower(0.6);


            }
            else if(!mgLimVert.getState() && verticalLift.isBusy()){
                verticalLift.setTargetPosition(0);
                verticalLift.setPower(0.2);

            }

            else{
                verticalLift.setPower(0);
                dumper2.setPosition(-1);
                dumper1.setPosition(1);

            }


            if(gamepad2.right_stick_y!=0){
                intakeSlides.setMode(RunMode.RUN_USING_ENCODER);
                intakeSlides.setPower(gamepad2.right_stick_y);
            }
            else if(gamepad2.right_bumper){
                intakeSlides.setMode(RunMode.RUN_TO_POSITION);
                intakeSlides.setPower(1);
                intakeSlides.setTargetPosition(RobotConstants.horizontalslidemax);
            }
            else if(gamepad2.left_bumper){
                intakeSlides.setMode(RunMode.RUN_TO_POSITION);
                intakeSlides.setPower(-1);
                intakeSlides.setTargetPosition(-125);
            }
            else if(!intakeSlides.isBusy()){
                intakeSlides.setPower(0);
            }



            motorFrontRight.setPower(-0.65*gamepad1.right_stick_y);
            motorFrontLeft.setPower(-0.65*gamepad1.left_stick_y);
            motorBackRight.setPower(-0.65*gamepad1.right_stick_y);
            motorBackLeft.setPower(-0.65*gamepad1.left_stick_y);

            telemetry.addData("time: ",time.milliseconds()-prevtime);
            prevtime = time.milliseconds();
            telemetry.update();



        }
    }

}
