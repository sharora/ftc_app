package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Drivetrain {
    private DcMotorEx motorFrontRight;
    private DcMotorEx motorFrontLeft;
    private DcMotorEx motorBackRight;
    private DcMotorEx motorBackLeft;
    private BNO055IMU imu;

    private LinearOpMode opMode;



    private double k = RobotConstants.driveSpeedCoefficient;

    public Drivetrain(HardwareMap hardwareMap, LinearOpMode opMode){

        this.opMode = opMode;
        motorFrontRight = hardwareMap.get(DcMotorEx.class,"motorFrontRight");
        motorFrontLeft = hardwareMap.get(DcMotorEx.class,"motorFrontLeft");
        motorBackRight = hardwareMap.get(DcMotorEx.class,"motorBackRight");
        motorBackLeft = hardwareMap.get(DcMotorEx.class,"motorBackLeft");

        motorBackLeft.setDirection(Direction.REVERSE);
        motorFrontLeft.setDirection(Direction.REVERSE);

        motorFrontRight.setMode(RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(RunMode.RUN_USING_ENCODER);

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



    }
    public void setRightPower(double power){
        motorFrontRight.setPower(k*power);
        motorBackRight.setPower(k*power);
    }

    public void setLeftPower(double power){
        motorFrontLeft.setPower(k*power);
        motorBackLeft.setPower(k*power);
    }
    public void reverse(){
        k *= -1;
    }

    //proportional correction is working great now - woohoo
    // value of 1 works and is very tolerant to error but less smooth
    // value of 0.1 is optimal for most smooth path while being most accurate
    public void RuntoPositionTrapezoidal(double x, double maxVelocity, double maxAcceleration){

        FtcDashboard dashboard = FtcDashboard.getInstance();

        TelemetryPacket packet = new TelemetryPacket();

        double kp = RobotConstants.kp;
//        double kp = 0;

        int initialPos = motorFrontRight.getCurrentPosition();

        double v = 0;

        //creating time object so that kinematics approach can be used
        ElapsedTime time = new ElapsedTime();

        //while loop for acceleration
        while(time.seconds()<Math.sqrt(x/(maxAcceleration)) && opMode.opModeIsActive()){

            //x = 1/2*a*t^2 which is then converted from in to ticks
            //calculating the position which robot should be at
            double expected = 0.5*maxAcceleration*Math.pow(time.seconds(),2)/(RobotConstants.
                    driveWheelDiameter*Math.PI) *RobotConstants.driveEncoderTicksPerRev;

            //finding the current position
            double current = motorFrontRight.getCurrentPosition()-initialPos;

            //difference between the two is calculated
            double error = expected - current;

            //error is translated into correction(PID)
            double feedback = kp*(error);

            // v = a*t + vinitial
            // v is then converted from in/s to ticks/s
            v = maxAcceleration*time.seconds()/(RobotConstants.driveWheelDiameter*Math.PI)
                    *RobotConstants.driveEncoderTicksPerRev;

            //setting motor powers
            motorFrontRight.setVelocity(v+feedback);
            motorFrontLeft.setVelocity(v+feedback);
            motorBackLeft.setVelocity(v+feedback);
            motorBackRight.setVelocity(v+feedback);

            packet.put("proportional", feedback);
            packet.put("pos",current + initialPos);
            packet.put("expected", expected+initialPos);
            dashboard.sendTelemetryPacket(packet);


        }

        time.reset();
        initialPos = motorFrontRight.getCurrentPosition();

        //while loop for slowing down
        while(time.seconds()<Math.sqrt(x/(maxAcceleration))&& opMode.opModeIsActive()){

            //x = 1/2*a*t^2 + vinitial * time which is then converted from in to ticks
            //calculating the position which robot should be at
            double expected = (-0.5*maxAcceleration*Math.pow(time.seconds(),2))/(RobotConstants.
                    driveWheelDiameter*Math.PI) * RobotConstants.driveEncoderTicksPerRev + v*time.seconds();

            //finding the current position
            double current = motorFrontRight.getCurrentPosition()-initialPos;

            //difference between the two is calculated
            double error = expected - current;

            //error is translated into correction(PID)
            double feedback = kp*(error);

            // v = a*t + vinitial
            // v is then converted from in/s to ticks/s
            double v2 = -maxAcceleration*time.seconds()/(RobotConstants.driveWheelDiameter*Math.PI)
                    *RobotConstants.driveEncoderTicksPerRev + v;

            //setting motor powers
            motorFrontRight.setVelocity(v2+feedback);
            motorFrontLeft.setVelocity(v2+feedback);
            motorBackLeft.setVelocity(v2+feedback);
            motorBackRight.setVelocity(v2+feedback);

            packet.put("proportional", feedback);
            packet.put("pos",current + initialPos);
            packet.put("expected", expected+x/2.0);
            dashboard.sendTelemetryPacket(packet);


        }

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

    }

    //needs to be formatted and improved before it can be used
    public void TurntoAngleTrapezoidal(double theta, double maxVelocity, double maxAcceleration){
        //convert inches to ticks
        //theta in radians




        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);



        ElapsedTime time = new ElapsedTime();
        double kp = RobotConstants.kpaxial;
//        double kp = 0;

        double x = theta*RobotConstants.wheelDistance;

        int initialPos = motorFrontRight.getCurrentPosition();

        double v = 0;
        while(time.seconds()<Math.sqrt(x/maxAcceleration)&& opMode.opModeIsActive()) {

//            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

            //x = 1/2*a*t^2 + vinitial * time which is then converted from in to ticks
            //calculating the position which robot should be at
            double expected = (maxAcceleration*Math.pow(time.seconds(),2) )/(RobotConstants.
                    driveWheelDiameter*Math.PI) * RobotConstants.driveEncoderTicksPerRev;

            //finding the current position
            double current = motorFrontRight.getCurrentPosition()-initialPos;

            //difference between the two is calculated
            double error = expected - current;

            //error is translated into correction(PID)
            double feedback = kp*(error);
            v = maxAcceleration*time.seconds()/12.56*537.6;
            motorFrontRight.setVelocity(v+feedback);
            motorBackRight.setVelocity(v+feedback);
            motorFrontLeft.setVelocity(-v-feedback);
            motorBackLeft.setVelocity(-v-feedback);




        }
        time.reset();
        initialPos = motorFrontRight.getCurrentPosition();
        while(time.seconds()<Math.sqrt(x/maxAcceleration)&& opMode.opModeIsActive()){

//            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

            //x = 1/2*a*t^2 + vinitial * time which is then converted from in to ticks
            //calculating the position which robot should be at
            double expected = (-maxAcceleration*Math.pow(time.seconds(),2))/(RobotConstants.
                    driveWheelDiameter*Math.PI) * RobotConstants.driveEncoderTicksPerRev + v*time.seconds();

            //finding the current position
            double current = motorFrontRight.getCurrentPosition()-initialPos;

            //difference between the two is calculated
            double error = expected - current;

            //error is translated into correction(PID)
            double feedback = kp*(error);
            double v2 = -maxAcceleration*time.seconds()/12.56*537.6 + v;
            motorFrontRight.setVelocity(v2+feedback);
            motorBackRight.setVelocity(v2+feedback);
            motorFrontLeft.setVelocity(-v2-feedback);
            motorBackLeft.setVelocity(-v2-feedback);


        }
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);


    }
    public void TurntoAngleTrapezoidalGyroCorrection(double theta, double maxVelocity, double maxAcceleration){
        //convert inches to ticks
        //theta in radians

        //need to add large radian offset to counter -180 to 180 gyro switch



        FtcDashboard dashboard = FtcDashboard.getInstance();

        TelemetryPacket packet = new TelemetryPacket();


        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        double initangle = angles.firstAngle;

        ElapsedTime time = new ElapsedTime();
        double kp = RobotConstants.kpaxial;
        double kd  = RobotConstants.kdaxial;
//        double kp = 0;

        double x = theta*RobotConstants.wheelDistance;
        double err = 0;
        double prev = 0;
        double lastHeading = 0;

        int initialPos = motorFrontRight.getCurrentPosition();

        double v = 0;
        //removed while opmodeisactive
        while(time.seconds()<Math.sqrt(x/maxAcceleration) && opMode.opModeIsActive()) {

            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

            

            //x = r*theta
            double expected = (0.5*maxAcceleration*Math.pow(time.seconds(),2) )/RobotConstants.wheelDistance;

            //finding the current position
            double current = angles.firstAngle-initangle;

            //difference between the two is calculated
            double error = (expected - current)/12.56*537.6;

            //error is translated into correction(PID)
            double feedback = kp*(error) + kd*(error - err)/(time.seconds()-prev);
            prev = time.time();
            err = error;
            v = maxAcceleration*time.seconds()/12.56*537.6;
            motorFrontRight.setVelocity(v+feedback);
            motorBackRight.setVelocity(v+feedback);
            motorFrontLeft.setVelocity(-v-feedback);
            motorBackLeft.setVelocity(-v-feedback);

            packet.put("error", error);
            packet.put("pos(angle in radians)",current+initangle);
            packet.put("expected(angle in radians)", expected+initangle);
            dashboard.sendTelemetryPacket(packet);


        }
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double middleangle = angles.firstAngle;
        err = 0;
        prev = 0;
        time.reset();
        while(time.seconds()<Math.sqrt(x/maxAcceleration) && opMode.opModeIsActive()){

            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

            //x =r*theta
            double expected = (-0.5*maxAcceleration*Math.pow(time.seconds(),2)+v*time.seconds()/537.6*12.56)
                    /RobotConstants.wheelDistance;
            //finding the current position
            double current = angles.firstAngle - middleangle;

            //difference between the two is calculated
            double error = (expected - current)/12.56*537.6;

            //error is translated into correction(PID)
            double feedback = kp*(error) + kd*(error - err)/(time.seconds()-prev);
            prev = time.time();
            err = error;
            double v2 = -maxAcceleration*time.seconds()/12.56*537.6 + v;
            motorFrontRight.setVelocity(v2+feedback);
            motorBackRight.setVelocity(v2+feedback);
            motorFrontLeft.setVelocity(-v2-feedback);
            motorBackLeft.setVelocity(-v2-feedback);

            packet.put("error", error);
            packet.put("pos(angle in radians)",current+initangle);
            packet.put("expected(angle in radians)", expected+theta/2.0);
            dashboard.sendTelemetryPacket(packet);


        }
        time.reset();
        while(time.seconds()<Math.sqrt(x/maxAcceleration) && opMode.opModeIsActive()){
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double error = (theta-(angles.firstAngle-initangle));
            double feedback = RobotConstants.kptrollturn*(error);
            motorFrontRight.setVelocity(feedback);
            motorBackRight.setVelocity(feedback);
            motorFrontLeft.setVelocity(-feedback);
            motorBackLeft.setVelocity(-feedback);

            packet.put("error", feedback);
            packet.put("pos", angles.firstAngle-initangle);
            dashboard.sendTelemetryPacket(packet);

        }
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);


    }




}
