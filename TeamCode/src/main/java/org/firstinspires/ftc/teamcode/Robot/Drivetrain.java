package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {
    private DcMotorEx motorFrontRight;
    private DcMotorEx motorFrontLeft;
    private DcMotorEx motorBackRight;
    private DcMotorEx motorBackLeft;

    private double k = RobotConstants.driveSpeedCoefficient;


    public Drivetrain(HardwareMap hardwareMap){

        motorBackLeft.setDirection(Direction.REVERSE);
        motorFrontLeft.setDirection(Direction.REVERSE);

        motorFrontRight = hardwareMap.get(DcMotorEx.class,"motorFrontRight");
        motorFrontLeft = hardwareMap.get(DcMotorEx.class,"motorFrontLeft");
        motorBackRight = hardwareMap.get(DcMotorEx.class,"motorBackRight");
        motorBackLeft = hardwareMap.get(DcMotorEx.class,"motorBackLeft");

    }
    public void setRightPower(double power){
        motorFrontRight.setPower(k*power);
        motorBackRight.setPower(k*power);
    }

    public void setLeftPower(double power){
        motorFrontLeft.setPower(k*power);
        motorBackLeft.setPower(k*power);
    }




}
