package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;

@Autonomous
public class downStringTest extends LinearOpMode {
    DcMotor rightLift;
    DcMotor leftLift;


    @Override

    public void runOpMode() throws InterruptedException {
        rightLift = hardwareMap.dcMotor.get("rightLift");
        leftLift = hardwareMap.dcMotor.get("leftLift");

        waitForStart();

        moveLift(-7100, -0.7);




    }
    public void moveLift(int ticks,double power){
        leftLift.setMode(RunMode.RUN_USING_ENCODER);
        rightLift.setMode(RunMode.STOP_AND_RESET_ENCODER);

        rightLift.setTargetPosition(-ticks);

        rightLift.setMode(RunMode.RUN_TO_POSITION);

        rightLift.setPower(power);



        while (rightLift.isBusy() && opModeIsActive()) {
            leftLift.setPower(-0.15);

        }

        rightLift.setPower(0);

        rightLift.setMode(RunMode.RUN_USING_ENCODER);

    }
}
