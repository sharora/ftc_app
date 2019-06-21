package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.RobotConstants;

@TeleOp
public class CraterTeleop extends LinearOpMode {

    private Robot robot;
    private ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode(){
        Robot robot = new Robot(hardwareMap,this);
        double prevtime = 0;
        telemetry.setAutoClear(true);
        waitForStart();

        int x = 0;
        int z = 1;
        boolean randomvar = true;

        double slideOutCoeff = 1;

        while(opModeIsActive()){

            slideOutCoeff = 0.7 + (robot.intake.getIntakeSlidePosition()/-1800.0)*0.3;
            //setting drivetrain powers
            robot.dt.setLeftPower(slideOutCoeff*gamepad1.left_stick_y);
            robot.dt.setRightPower(slideOutCoeff*gamepad1.right_stick_y);

            //setting intake powers
            if(gamepad1.right_trigger>0) {
                robot.intake.setSpinnerPower(1);
            }else if(gamepad1.left_trigger>0){
                robot.intake.setSpinnerPower(-1);
            }
            else{
                robot.intake.setSpinnerPower(0);
            }

            //setting powers for hanging mechanism
            if(gamepad2.dpad_up){
                robot.outake.setHangPower(1);
            }
            else if(gamepad2.dpad_down){
                robot.outake.setHangPower(-1);
            }
            else{
                robot.outake.setHangPower(0);
            }






            //setting intake positions
            //sends the mechanism to bottom position when right bumper is pressed
            if(gamepad2.right_trigger>0){
                robot.intake.setIntakeDumpPosition(0.92);
                robot.intake.setStopperServoPosition(-0.5);
                x=0;
                randomvar = true;

            }

            //sends the mechanism to the top position when left bumper is pressed
            else if(gamepad2.left_trigger>0 && robot.intake.getIntakeSlidePosition()>-600){
                robot.intake.setIntakeDumpPosition(0.15);
                robot.intake.setStopperServoPosition(0.5);
                if(randomvar){
                    robot.intake.setSpinnerPower(-0.7);
                }
                else{
                    robot.intake.setSpinnerPower(-0.3*z);
                }
                x++;
                if(x==10){
                    z =  -z;
                    x = 0;
                    randomvar = false;
                }


            }
            else if(gamepad2.left_trigger>0 && robot.intake.getIntakeSlidePosition()>-1200){
                robot.intake.setIntakeDumpPosition(0.5);
                robot.intake.setStopperServoPosition(0.5);
            }
            //if no buttons are pressed the mechanism is sent to neutral position
            else{
                robot.intake.setIntakeDumpPosition(0.5);
                robot.intake.setStopperServoPosition(0);
                x=0;
                randomvar = true;
            }

            //setting the intake slide powers
            if(gamepad2.right_stick_y!=0){
                robot.intake.setIntakeSlidePower(gamepad2.right_stick_y);
            }
            else if(gamepad2.right_bumper){
                robot.intake.setIntakeSlidePosition(RobotConstants.horizontalslidemax,0.4);
            }
            else if(gamepad2.left_bumper){
                robot.intake.setIntakeSlidePosition(-0,-1);
            }
            else if(!robot.intake.isIntakeSlidesBusy()){
                robot.intake.setIntakeSlidePower(0);
            }


            //setting vertical slide powers
            if(gamepad1.left_bumper){
                robot.outake.setUpSlidePosition(-1440,1);

                if(robot.outake.upSlidePosition()<-300){

                    robot.outake.setDumpPos(robot.outake.getDumperPos() + 0.056*(Math.max(0.95-robot.outake.getDumperPos(),0.1)));
                }
//                robot.outake.setDumpPos(robot.outake.getDumperPos() - 0.009* (robot.outake.getDumperPos()+0.5));


            }
            else if(gamepad2.y){
                robot.outake.setUpSlidePosition(-980,0.6);
            }
            else if(gamepad1.b){
                robot.outake.setDumpPos(0.5);
            }
            else if(robot.outake.upSlidePosition()>0 && robot.outake.upSlideisBusy()){
                robot.outake.setUpSlidePosition(0,0.3);
            }
            else{
                robot.outake.setUpSlidePosition(0,0);
                robot.outake.setDumpPos(0.1);
            }






            //updating telemetry
            prevtime = time.milliseconds();
            telemetry.addData("time: ",time.milliseconds()-prevtime);
            telemetry.update();
        }
    }

}

