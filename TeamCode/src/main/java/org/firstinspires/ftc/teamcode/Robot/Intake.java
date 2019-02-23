package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private DcMotor intakeDump;
    private DcMotor intakeSlides;

    private CRServo spinner;

    LinearOpMode opMode;

    public Intake(HardwareMap hardwareMap, LinearOpMode opMode){

        this.opMode = opMode;
        intakeDump = hardwareMap.dcMotor.get("intakeDump");
        intakeDump.setMode(RunMode.RUN_TO_POSITION);
        intakeSlides = hardwareMap.dcMotor.get("intakeSlides");

        intakeSlides.setMode(RunMode.RUN_TO_POSITION);

        spinner = hardwareMap.crservo.get("spinner");

    }
    public void setIntakeSlidePosition(int n, double power){
        intakeSlides.setMode(RunMode.RUN_TO_POSITION);
        intakeSlides.setTargetPosition(n);
        intakeSlides.setPower(power);
    }

}
