package org.firstinspires.ftc.teamcode.FunctionTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.Drivetrain;

@Autonomous
public class DrivetrainTest extends LinearOpMode {

    public Drivetrain dt;



    @Override
    public void runOpMode() throws InterruptedException{
        dt = new Drivetrain(hardwareMap , this);

        int ayganggang   = hardwareMap.appContext.getResources()
                .getIdentifier("ayganggang",   "raw", hardwareMap.appContext.getPackageName());

        waitForStart();

//        dt.RuntoPositionTrapezoidal(20,40, 20);
        //negative angle is clockwise
//        SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, ayganggang);
//        sleep(30000);
        dt.TurntoAngleTrapezoidalGyroCorrection(Math.PI/2, 40, 30);
//        dt.RuntoPositionTrapezoidal(20,40,20);






    }

}
