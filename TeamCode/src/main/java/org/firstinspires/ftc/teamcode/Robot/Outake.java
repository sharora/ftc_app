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
    private Servo hangLock;

    //servo depositer
    private Servo dumper1;


    //limit switch that indicates base position
    private DigitalChannel mgLimVert;

    public Outake(HardwareMap hardwareMap, LinearOpMode opMode){

        this.opMode = opMode;

        hangingMotor = hardwareMap.dcMotor.get("hangingMotor");
        hangingMotor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        verticalLift = hardwareMap.dcMotor.get("verticalLift");
        verticalLift.setDirection(Direction.REVERSE);
        verticalLift.setMode(RunMode.RUN_TO_POSITION);

        hangLock = hardwareMap.servo.get("hangLock");

        dumper1 = hardwareMap.servo.get("dumper1");


        mgLimVert = hardwareMap.digitalChannel.get("mgLimVert");


    }
    public void setHangPower(double power){
        if(power == 0){
            hangingMotor.setPower(0);
            hangingMotor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        }
        else{
            hangingMotor.setPower(power);
        }

    }
    public void setHangLock(double position){
        hangLock.setPosition(position);
    }
    public void setUpSlidePower(double power){
        verticalLift.setMode(RunMode.RUN_USING_ENCODER);
        verticalLift.setPower(power);
    }
    public void setUpSlidePosition(int position, double power){
        verticalLift.setMode(RunMode.RUN_TO_POSITION);
        verticalLift.setTargetPosition(position);
        verticalLift.setPower(power);
    }
    public int upSlidePosition(){
        return verticalLift.getCurrentPosition();
    }
    public boolean upSlideisBusy(){
        return verticalLift.isBusy();
    }

    public void unHanginAutoV2(){
        hangLock.setPosition(0);
        hangingMotor.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        dumper1.setPosition(0.1);

        verticalLift.setTargetPosition(-600);

        verticalLift.setMode(RunMode.RUN_TO_POSITION);

        verticalLift.setPower(-0.5);



        while (verticalLift.isBusy() && opMode.opModeIsActive()) {
            opMode.telemetry.addData("hang pos1", hangingMotor.getCurrentPosition());
            opMode.telemetry.addData("vert pos1", verticalLift.getCurrentPosition());
            opMode.telemetry.update();
        }
        hangingMotor.setMode(RunMode.RUN_USING_ENCODER);
        verticalLift.setTargetPosition(-1150);

        verticalLift.setPower(-0.125);
        hangingMotor.setPower(1.0);

        while (hangingMotor.getCurrentPosition()<9400 && verticalLift.isBusy() && opMode.opModeIsActive()) {
            opMode.telemetry.addData("hang pos2", hangingMotor.getCurrentPosition());
            opMode.telemetry.addData("vert pos2", verticalLift.getCurrentPosition());
            opMode.telemetry.update();
        }

        hangingMotor.setMode(RunMode.RUN_USING_ENCODER);
        hangingMotor.setPower(0);

    }
    public void unHangInAuto(){
        hangingMotor.setMode(RunMode.RUN_TO_POSITION);
        hangingMotor.setTargetPosition(50);
        hangingMotor.setPower(1);
        while(hangingMotor.isBusy()){

        }
        hangingMotor.setPower(0);
        hangingMotor.setMode(RunMode.RUN_WITHOUT_ENCODER);

        hangLock.setPosition(0);
        hangingMotor.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);


        verticalLift.setTargetPosition(-600);

        verticalLift.setMode(RunMode.RUN_TO_POSITION);

        verticalLift.setPower(-0.5);



        while (verticalLift.isBusy() && opMode.opModeIsActive()) {
            opMode.telemetry.addData("hang pos1", hangingMotor.getCurrentPosition());
            opMode.telemetry.addData("vert pos1", verticalLift.getCurrentPosition());
            opMode.telemetry.update();
        }
        dumper1.setPosition(0.1);
        hangingMotor.setMode(RunMode.RUN_TO_POSITION);


        hangingMotor.setTargetPosition(9400);
        verticalLift.setTargetPosition(-1150);

        verticalLift.setPower(-0.125);
        hangingMotor.setPower(1.0);

        while (hangingMotor.isBusy() && verticalLift.isBusy() && opMode.opModeIsActive()) {
            opMode.telemetry.addData("hang pos", hangingMotor.getCurrentPosition());
            opMode.telemetry.addData("vert pos", verticalLift.getCurrentPosition());
            opMode.telemetry.update();
        }

        hangingMotor.setMode(RunMode.RUN_USING_ENCODER);
        hangingMotor.setPower(0);

    }
    public void lowerSlidesAuto(){
        while(verticalLift.getCurrentPosition()<-50){
            verticalLift.setPower(0);
        }
        verticalLift.setTargetPosition(0);
        verticalLift.setPower(0.3);
    }
    public double getDumperPos(){
        return dumper1.getPosition();
    }
    public boolean getMgLimState(){
        return mgLimVert.getState();
    }
    public void setDumpPos(double p){
        dumper1.setPosition(p);
    }
    public void setHangMotorPosition(int n, double p){
        hangingMotor.setMode(RunMode.RUN_TO_POSITION);
        hangingMotor.setTargetPosition(n);
        hangingMotor.setPower(p);

    }
    



}
