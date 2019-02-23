package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Robot {
    public Drivetrain dt;
    public Intake intake;
    public Outake outake;

    private Servo teamMarkerServo;


    public Robot(HardwareMap hardwareMap, LinearOpMode opMode){
        dt = new Drivetrain(hardwareMap,opMode);
        intake = new Intake(hardwareMap,opMode);
        outake = new Outake(hardwareMap,opMode);

        teamMarkerServo = hardwareMap.servo.get("teamMarker");


    }
    public void setTeamMarkerServo(double p){
        teamMarkerServo.setPosition(p);
    }


}
