package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
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

        intakeDump = hardwareMap.dcMotor.get("intakeDump");
        intakeDump.setMode(RunMode.RUN_TO_POSITION);
        intakeSlides = hardwareMap.dcMotor.get("intakeSlides");

        intakeSlides.setMode(RunMode.RUN_TO_POSITION);


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
//            if(gamepad1.right_bumper){
//                spinner.setPower(1);
//            }
//            else if(gamepad1.left_bumper){
//                spinner.setPower(-1);
//            }

            if(gamepad2.dpad_up){
                hangingMotor.setPower(1);
            }
            else if(gamepad2.dpad_down){
                hangingMotor.setPower(-1);
            }
            else{
                hangingMotor.setPower(0);
            }




            //sends the mechanism to bottom position when right bumper is pressed
            if(gamepad2.right_trigger>0){
                intakeDump.setPower(0.4);
                intakeDump.setTargetPosition(-1500);
                spinner.setPower(-1);
            }
            //sends the mechanism to the top position when left bumper is pressed
            else if(gamepad2.left_trigger>0){
                intakeDump.setPower(0.6);
                intakeDump.setTargetPosition(0);
                spinner.setPower(-1);
            }
            else if(gamepad2.a){
                spinner.setPower(1);
            }
            //if no buttons are pressed than the mechanism is sent to neutral position
            else{
                intakeDump.setPower(0.4);
                intakeDump.setTargetPosition(-650);
                spinner.setPower(0);
            }



//            if(dumpPos == 0 && intakeContinue){
//                if(Math.abs(intakeDump.getCurrentPosition() + 600) < 3){
//                    dumpPos = 1;
//                    intakeDump.setPower(0);
//                    intakeContinue = false;
//
//                }
//                else if(Math.abs(intakeDump.getCurrentPosition() + 1600) < 20){
//                    dumpPos = 2;
//                    intakeDump.setPower(0);
//                    intakeContinue = false;
//
//                }
//                else if(gamepad2.right_bumper){
//                    intakeDump.setPower(0.4);
//                    intakeDump.setTargetPosition(-600);
//                }
//
//
//            }else if(dumpPos == 1 && intakeContinue){
//                if(Math.abs(intakeDump.getCurrentPosition() + 1600) < 20){
//                    dumpPos = 2;
//                    intakeDump.setPower(0);
//                    intakeContinue = false;
//
//                }
//                else if(Math.abs(intakeDump.getCurrentPosition()) < 6){
//                    dumpPos = 0;
//                    intakeDump.setPower(0);
//                    intakeContinue = false;
//
//                }
//                else if(gamepad2.right_bumper){
//                    intakeDump.setPower(0.4);
//                    intakeDump.setTargetPosition(-1600);
//                }
//                else if(gamepad2.left_bumper){
//                    intakeDump.setPower(0.65);
//                    intakeDump.setTargetPosition(0);
//                }
//
//            }
//            else if(dumpPos == 2){
//                if(Math.abs(intakeDump.getCurrentPosition() + 600) < 3){
//                    dumpPos = 1;
//                    intakeDump.setPower(0);
//                    intakeContinue = false;
//
//                }
//                else if(gamepad2.left_bumper){
//                    intakeDump.setPower(0.65);
//                    intakeDump.setTargetPosition(-600);
//                }
//
//
//            }
//            else{
//                intakeContinue = true;
//            }
            //Deposit minerals if the button B is pressed
//            if(gamepad1.b){
//                //if the dummper is in the second stage of rotation lower the speed
//                if(dumper2.getPosition()>0.4 && dumper2.getPosition()<0.84){
//                    dumper2.setPosition(dumper2.getPosition()+0.013);
//                    dumper1.setPosition(dumper1.getPosition()-0.013);
//                }
//                //if the dumper is in the first stage of rotation set a higher speed
//                else if(dumper2.getPosition()<0.84){
//                    dumper2.setPosition(dumper2.getPosition()+0.04);
//                    dumper1.setPosition(dumper1.getPosition()-0.040);
//                }
//            }
//            //otherwise when the button is not being pressed, set the button back to home position
//            else{
//                dumper1.setPosition(1);
//                dumper2.setPosition(-1);
//            }

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
            if(gamepad1.b){
                verticalLift.setTargetPosition(-1930);
                verticalLift.setPower(1);
                if(verticalLift.getCurrentPosition()<-50){
//                    if(dumper2.getPosition()>0.4 && dumper2.getPosition()<0.84){
//                        dumper2.setPosition(dumper2.getPosition()+0.013);
//                        dumper1.setPosition(dumper1.getPosition()-0.013);
//                    }
//                    //if the dumper is in the first stage of rotation set a higher speed
//                    else if(dumper2.getPosition()<0.84){
//                        dumper2.setPosition(dumper2.getPosition() + 0.1);
//                        dumper1.setPosition(dumper1.getPosition() + 0.1);
//                    }
                    dumper2.setPosition(dumper2.getPosition() + 0.08*(0.80-dumper2.getPosition()));
                    dumper1.setPosition(-(dumper1.getPosition() + 0.08*(0.80-dumper1.getPosition())));
                }
//                else{
//                    dumper2.setPosition(dumper2.getPosition() + 0.002*(-1-dumper2.getPosition()));
//                    dumper1.setPosition(-(dumper1.getPosition() + 0.002*(-1-dumper1.getPosition())));
//
//                }


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
//            else if(mgLimVert.getState()){
//                verticalLift.setPower(0);
//                dumper2.setPosition(dumper2.getPosition() + 0.6*(-1-dumper2.getPosition()));
//                dumper1.setPosition(-(dumper1.getPosition() + 0.6*(-1-dumper1.getPosition())));
//
//            }
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
                intakeSlides.setTargetPosition(0);
            }
            else if(!intakeSlides.isBusy()){
                intakeSlides.setPower(0);
            }





            telemetry.addData("vertPos", vertPos);
            telemetry.addData("MotorPower", gamepad2.left_stick_y);
            telemetry.addData("Intake Dump Position: ",dumpPos);
            telemetry.addData("Intake Dump Ticks: " , intakeDump.getCurrentPosition());
            telemetry.update();


            motorFrontRight.setPower(-0.65*gamepad1.right_stick_y);
            motorFrontLeft.setPower(-0.65*gamepad1.left_stick_y);
            motorBackRight.setPower(-0.65*gamepad1.right_stick_y);
            motorBackLeft.setPower(-0.65*gamepad1.left_stick_y);
//            double x = -gamepad1.right_stick_x;
//            double forward = gamepad1.left_trigger*0.75;
//            double backward = gamepad1.right_trigger*0.75;
//            if(gamepad1.right_trigger== 0 && gamepad1.left_trigger ==0){
//
//                motorFrontRight.setPower(0.6*x);
//                motorBackRight.setPower(0.6*x);
//                motorBackLeft.setPower(0.6*-x);
//                motorFrontLeft.setPower(0.6*-x);
//
//            }
//            else if(forward != 0){
//                if (x>=0){
//                    motorFrontRight.setPower(forward);
//                    motorBackRight.setPower(forward);
//                    motorFrontLeft.setPower(forward*(1-x));
//                    motorBackLeft.setPower(forward*(1-x));
//
//                }
//                else{
//                    motorFrontLeft.setPower(forward);
//                    motorBackLeft.setPower(forward);
//                    motorFrontRight.setPower(forward*(1+x));
//                    motorBackRight.setPower(forward*(1+x));
//                }
//
//            }
//            else{
//                if (x>=0){
//                    motorFrontRight.setPower(-backward);
//                    motorBackRight.setPower(-backward);
//                    motorBackLeft.setPower(-backward*(1-x));
//                    motorFrontLeft.setPower(-backward*(1-x));
//
//                }
//                else{
//                    motorFrontLeft.setPower(-backward);
//                    motorBackLeft.setPower(-backward);
//                    motorFrontRight.setPower(-backward*(1+x));
//                    motorBackRight.setPower(-backward*(1+x));
//                }
//
//            }


        }
    }
}
