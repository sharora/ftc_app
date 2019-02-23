package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {
    public static double kp = 0.3;
    public static double ki = 0;
    public static double kd = 0;

    public static double kpaxial = 1.2;
    public static double kptrollturn = 2500;
    public static double kdaxial = 0;
    public static double wheelDistance = 7.9;
    public static int horizontalslidemax = -1350;

    public static double driveSpeedCoefficient = 0.83;
    public static double driveWheelDiameter = 4.00;
    public static double driveEncoderTicksPerRev = 536.7;


}
