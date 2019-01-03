package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by sharma on 10/14/18.
 *
 * This class is for the teleop period in the robot game and has a continuous loop that maps
 * controller input to robot actions. Read the names of variables and refer to the design
 * to know which buttons control what actions.
 */
@TeleOp
public class Teleop extends LinearOpMode {
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
    public ElapsedTime time = new ElapsedTime();




    //hello boom yo ting goes skrrra
    public void runOpMode() throws InterruptedException {
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        hangingMotor = hardwareMap.dcMotor.get("hangingMotor");
        verticalLift = hardwareMap.dcMotor.get("verticalLift");
        verticalLift.setDirection(Direction.REVERSE);
        verticalLift.setMode(RunMode.RUN_TO_POSITION);

        intakeDump = hardwareMap.dcMotor.get("intakeDump");

        mgLimVert = hardwareMap.digitalChannel.get("mgLimVert");

        intakeDump.setMode(RunMode.RUN_TO_POSITION);
        intakeSlides = hardwareMap.dcMotor.get("intakeSlides");
        intakeSlides.setDirection(Direction.REVERSE);

        motorBackLeft.setDirection(Direction.REVERSE);
        motorFrontLeft.setDirection(Direction.REVERSE);

        dumper1 = hardwareMap.servo.get("dumper1");
        dumper2 = hardwareMap.servo.get("dumper2");

        spinner = hardwareMap.crservo.get("spinner");

        int dumpPos = 0;
        int vertPos = 0;

        boolean intakeContinue = false;
        boolean vertSlidetime = false;



        telemetry.addData("System: ", "waiting for initalization");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.right_bumper){
                spinner.setPower(1);
            }
            else if(gamepad1.left_bumper){
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


            intakeSlides.setPower(gamepad2.right_stick_y);
            if(dumpPos == 0 && intakeContinue){
                if(Math.abs(intakeDump.getCurrentPosition() + 600) < 3){
                    dumpPos = 1;
                    intakeDump.setPower(0);
                    intakeContinue = false;

                }
                else if(Math.abs(intakeDump.getCurrentPosition() + 1600) < 20){
                    dumpPos = 2;
                    intakeDump.setPower(0);
                    intakeContinue = false;

                }
                else if(gamepad2.right_bumper){
                    intakeDump.setPower(0.4);
                    intakeDump.setTargetPosition(-600);
                }


            }else if(dumpPos == 1 && intakeContinue){
                if(Math.abs(intakeDump.getCurrentPosition() + 1600) < 20){
                    dumpPos = 2;
                    intakeDump.setPower(0);
                    intakeContinue = false;

                }
                else if(Math.abs(intakeDump.getCurrentPosition()) < 6){
                    dumpPos = 0;
                    intakeDump.setPower(0);
                    intakeContinue = false;

                }
                else if(gamepad2.right_bumper){
                    intakeDump.setPower(0.4);
                    intakeDump.setTargetPosition(-1600);
                }
                else if(gamepad2.left_bumper){
                    intakeDump.setPower(0.65);
                    intakeDump.setTargetPosition(0);
                }

            }
            else if(dumpPos == 2){
                if(Math.abs(intakeDump.getCurrentPosition() + 600) < 3){
                    dumpPos = 1;
                    intakeDump.setPower(0);
                    intakeContinue = false;

                }
                else if(gamepad2.left_bumper){
                    intakeDump.setPower(0.65);
                    intakeDump.setTargetPosition(-600);
                }


            }
            else{
                intakeContinue = true;
            }

            if(gamepad1.b){

                dumper1.setPosition(-0.76);
                dumper2.setPosition(0.76);
            }
            else{
                dumper1.setPosition(1);
                dumper2.setPosition(-1);
            }

//            if(vertPos == 1 && !mgLimVert.getState()){
//                if(gamepad2.left_stick_y < 0){
//                    vertPos = 2;
//                    verticalLift.setPower(0);
//
//                }
//                else if(gamepad2.left_stick_y >= 0 && vertSlidetime){
//                    verticalLift.setPower(-0.1);
//                    telemetry.addData("time",time.milliseconds());
//                    if(Math.abs(time.milliseconds() - 400)< 50 ){
//                        vertPos = 0;
//                        vertSlidetime = false;
//                        verticalLift.setPower(0);
//                    }
//
//                }
//                else if(gamepad2.left_stick_y >= 0){
//                    vertSlidetime = true;
//                    time.reset();
//                }
//
//
//            }
//            else if(vertPos == 1 && mgLimVert.getState()){
//                verticalLift.setPower(gamepad2.left_stick_y);
//
//            }
//
//            else if((vertPos == 0 || vertPos == 2) && mgLimVert.getState()){
//                vertPos = 1;
//                verticalLift.setPower(gamepad2.left_stick_y);
//            }
//            else if(vertPos == 0){
//                if(gamepad2.left_stick_y < 0){
//                    verticalLift.setPower(gamepad2.left_stick_y);
//                }
//                else{
//                    verticalLift.setPower(0);
//                }
//
//            }else if(vertPos == 2){
//                if(gamepad2.left_stick_y > 0) {
//                    verticalLift.setPower(gamepad2.left_stick_y);
//                }
//                else{
//                    verticalLift.setPower(-0.1);
//                }
//
//
//            }
            if(gamepad2.x){
                verticalLift.setTargetPosition(-1950);
                verticalLift.setPower(1);

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
            }





            telemetry.addData("vertPos", vertPos);
            telemetry.addData("MotorPower", gamepad2.left_stick_y);
            telemetry.addData("Intake Dump Position: ",dumpPos);
            telemetry.addData("Intake Dump Ticks: " , intakeDump.getCurrentPosition());
            telemetry.update();


            motorFrontRight.setPower(0.9*gamepad1.right_stick_y);
            motorFrontLeft.setPower(0.9*gamepad1.left_stick_y);
            motorBackRight.setPower(0.9*gamepad1.right_stick_y);
            motorBackLeft.setPower(0.9*gamepad1.left_stick_y);


        }
    }
}
