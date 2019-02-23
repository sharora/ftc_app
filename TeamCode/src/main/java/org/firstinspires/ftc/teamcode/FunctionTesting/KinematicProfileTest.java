package org.firstinspires.ftc.teamcode.FunctionTesting;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Robot.RobotConstants;

@Autonomous
@Disabled
public class KinematicProfileTest extends LinearOpMode {
    DcMotorEx motorFrontRight;
    DcMotorEx motorFrontLeft;
    DcMotorEx motorBackRight;
    DcMotorEx motorBackLeft;



    public ElapsedTime time = new ElapsedTime();
    private BNO055IMU imu;



    @Override
    public void runOpMode() throws InterruptedException {


        motorFrontRight = hardwareMap.get(DcMotorEx.class,"motorFrontRight");
        motorFrontLeft = hardwareMap.get(DcMotorEx.class,"motorFrontLeft");
        motorBackRight = hardwareMap.get(DcMotorEx.class,"motorBackRight");
        motorBackLeft = hardwareMap.get(DcMotorEx.class,"motorBackLeft");


        motorBackLeft.setDirection(Direction.REVERSE);
        motorFrontLeft.setDirection(Direction.REVERSE);


        BNO055IMU.Parameters gyroParam = new BNO055IMU.Parameters();
        gyroParam.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        gyroParam.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyroParam.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        gyroParam.loggingEnabled      = true;
        gyroParam.loggingTag          = "IMU";
        gyroParam.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize, the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class,"imu");
        imu.initialize(gyroParam);



        waitForStart();
        if (isStopRequested()) return;
        //velocity and accleration in units of inches/second
//        RuntoPositionTrapezoidal(20, 40, 20);
//        TurntoAngleRuntoPosition
        TurntoAngleTrapezoidal(Math.PI/2,40 , 15);









    }


    public void RuntoPositionTrapezoidal(double x, double maxVelocity, double maxAcceleration){
        //convert inches to ticks

        FtcDashboard dashboard = FtcDashboard.getInstance();


        TelemetryPacket packet = new TelemetryPacket();
        double kp = RobotConstants.kp;
        ElapsedTime time = new ElapsedTime();

        int initial = motorFrontRight.getCurrentPosition();

        double v = 0;
        while(time.seconds()<Math.sqrt(x/maxAcceleration) && opModeIsActive()) {

            double expected = maxAcceleration*Math.pow(time.seconds(),2)/12.56*537.6;
            double current = motorFrontRight.getCurrentPosition()-initial;
            double feedback = kp*(expected-current);

            v = maxAcceleration*time.seconds()/12.56*537.6;
            motorFrontRight.setVelocity(v+feedback);
            motorFrontLeft.setVelocity(v+feedback);
            motorBackLeft.setVelocity(v+feedback);
            motorBackRight.setVelocity(v+feedback);
            packet.put("proportional", feedback);
            packet.put("pos",current/537.6*12.56);
            packet.put("expected", expected);
            dashboard.sendTelemetryPacket(packet);


        }
        time.reset();

        while(time.seconds()<Math.sqrt(x/maxAcceleration) && opModeIsActive()){

            double expected = maxAcceleration*Math.pow(time.seconds(),2)/12.56*537.6;
            double current = motorFrontRight.getCurrentPosition()-initial;
            double feedback = kp*(expected-current);

            double v2 = -maxAcceleration*time.seconds()/12.56*537.6 + v;
            motorFrontRight.setVelocity(v2+feedback);
            motorFrontLeft.setVelocity(v2+feedback);
            motorBackLeft.setVelocity(v2+feedback);
            motorBackRight.setVelocity(v2+feedback);

            packet.put("proportional", feedback);
            packet.put("expected", expected);
            packet.put("pos",current/537.6*12.56);
            dashboard.sendTelemetryPacket(packet);

        }

    }

    public void TurntoAngleTrapezoidal(double theta, double maxVelocity, double maxAcceleration){
        //convert inches to ticks
        //theta in radians


        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        FtcDashboard dashboard = FtcDashboard.getInstance();


        TelemetryPacket packet = new TelemetryPacket();


        ElapsedTime time = new ElapsedTime();
        double kp = RobotConstants.kpaxial;

        double x = theta*RobotConstants.wheelDistance;

        double v = 0;
        while(time.seconds()<Math.sqrt(x/maxAcceleration) && opModeIsActive()) {

            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

            double expected = maxAcceleration*Math.pow(time.seconds(),2)/RobotConstants.wheelDistance;
            double feedback = kp*(expected-angles.firstAngle)*537.5/12.56;

            v = maxAcceleration*time.seconds()/12.56*537.6;
            motorFrontRight.setVelocity(v+feedback);
            motorBackRight.setVelocity(v+feedback);
            motorFrontLeft.setVelocity(-v-feedback);
            motorBackLeft.setVelocity(-v-feedback);
            packet.put("proportional", feedback);
            packet.put("angle",angles.firstAngle);
            dashboard.sendTelemetryPacket(packet);


        }
        time.reset();
        while(time.seconds()<Math.sqrt(x/maxAcceleration) && opModeIsActive()){

            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

            double expected = maxAcceleration*Math.pow(time.seconds(),2)/RobotConstants.wheelDistance;
            double feedback = kp*(expected-angles.firstAngle)*537.5/12.56;

            double v2 = -maxAcceleration*time.seconds()/12.56*537.6 + v;
            motorFrontRight.setVelocity(v2+feedback);
            motorBackRight.setVelocity(v2+feedback);
            motorFrontLeft.setVelocity(-v2-feedback);
            motorBackLeft.setVelocity(-v2-feedback);
            packet.put("expected", expected);
            packet.put("angle",-angles.firstAngle);
            dashboard.sendTelemetryPacket(packet);

        }
        while(Math.abs(angles.firstAngle + theta)>0.06 && opModeIsActive()){

            angles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double power  = (angles.firstAngle-theta)*RobotConstants.kp*2500;

            motorFrontRight.setVelocity(power);
            motorFrontLeft.setVelocity(-power);
            motorBackLeft.setVelocity(-power);
            motorBackRight.setVelocity(power);

        }
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);


    }
}
