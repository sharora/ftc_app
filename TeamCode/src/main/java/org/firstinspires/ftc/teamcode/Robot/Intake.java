package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Intake {

    private ServoImplEx intakeDump;
    private Servo stopperServo;
    private DcMotorEx intakeSlides;

    private DcMotor spinner;

    LinearOpMode opMode;

    public Intake(HardwareMap hardwareMap, LinearOpMode opMode){

        this.opMode = opMode;
        intakeDump = hardwareMap.get(ServoImplEx.class,"intakeDump");
        intakeSlides = hardwareMap.get(DcMotorEx.class,"intakeSlides");
        stopperServo = hardwareMap.servo.get("stopperServo");

        intakeSlides.setMode(RunMode.RUN_TO_POSITION);
        intakeSlides.setPower(0);

        spinner = hardwareMap.dcMotor.get("spinner");

    }
    public void setIntakeSlidePosition(int n, double power){
        intakeSlides.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        intakeSlides.setMode(RunMode.RUN_TO_POSITION);
        intakeSlides.setTargetPosition(n);
        intakeSlides.setPower(power);
    }
    public void setStopperServoPosition(double p){
        stopperServo.setPosition(p);
    }
    public void setIntakeDumpPosition(double p){
        intakeDump.setPosition(p);
    }
    public void setSpinnerPower(double p){
        spinner.setPower(p);
    }
    public void setIntakeSlidePower(double p){
        intakeSlides.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        intakeSlides.setMode(RunMode.RUN_USING_ENCODER);
        intakeSlides.setPower(p);
    }
    public boolean isIntakeSlidesBusy(){
        return intakeSlides.isBusy();
    }
    public int getIntakeSlidePosition(){
        return intakeSlides.getCurrentPosition();
    }



}
