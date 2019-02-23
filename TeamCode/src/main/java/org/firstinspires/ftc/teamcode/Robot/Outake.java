package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Outake {
    //motors
    private DcMotor hangingMotor;
    private DcMotor verticalLift;

    private LinearOpMode opMode;

    //servo depositers
    Servo dumper1;
    Servo dumper2;

    //limit switch that indicates base position
    DigitalChannel mgLimVert;

    public Outake(HardwareMap hardwareMap, LinearOpMode opMode){

        this.opMode = opMode;

        hangingMotor = hardwareMap.dcMotor.get("hangingMotor");

        verticalLift = hardwareMap.dcMotor.get("verticalLift");
        verticalLift.setDirection(Direction.REVERSE);
        verticalLift.setMode(RunMode.RUN_TO_POSITION);

        dumper1 = hardwareMap.servo.get("dumper1");
        dumper2 = hardwareMap.servo.get("dumper2");

        mgLimVert = hardwareMap.digitalChannel.get("mgLimVert");


    }
    public void setHangPower(double power){
        if(power == 0){
            hangingMotor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        }
        else{
            hangingMotor.setPower(power);
        }

    }
    public void unHangInAuto(){
        verticalLift.setMode(RunMode.STOP_AND_RESET_ENCODER);
        hangingMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);

        verticalLift.setTargetPosition(-15200/8);
        hangingMotor.setTargetPosition(15200 + 1200);


        verticalLift.setMode(RunMode.RUN_TO_POSITION);
        hangingMotor.setMode(RunMode.RUN_TO_POSITION);

        hangingMotor.setPower(1.0);
        verticalLift.setPower(-0.125);



        while (hangingMotor.isBusy() || verticalLift.isBusy() && opMode.opModeIsActive()) {


        }

        hangingMotor.setPower(0);

        hangingMotor.setMode(RunMode.RUN_USING_ENCODER);

    }
    public void lowerSlidesAuto(){
        while(mgLimVert.getState()){
            verticalLift.setPower(0);
        }
        verticalLift.setTargetPosition(0);
        verticalLift.setPower(0.2);
    }
    public boolean getMgLimState(){
        return mgLimVert.getState();
    }
    public void setDumpPos(double p){
        dumper1.setPosition(-p);
        dumper2.setPosition(p);

    }
    



}
