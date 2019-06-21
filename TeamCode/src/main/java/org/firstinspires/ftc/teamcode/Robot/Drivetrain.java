package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxEmbeddedIMU;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynchV2;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImplOnSimple;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
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


    private LinearOpMode opMode;
    private HardwareMap hardwareMap;
    private double lastheading = 0;
    private double deltaheading = 0;
    private double currentheading = 0;
    private Orientation angles;
    private LynxEmbeddedIMU imu;
    private double lastLeftVelo = 0;
    private double lastRightVelo = 0;




    private double k = RobotConstants.driveSpeedCoefficient;

    public Drivetrain(HardwareMap hardwareMap, LinearOpMode opMode){

        this.opMode = opMode;
        this.hardwareMap = hardwareMap;
        LynxModule module = hardwareMap.getAll(LynxModule.class).iterator().next();


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

//
//        BNO055IMU.Parameters gyroParam = new BNO055IMU.Parameters();
//        gyroParam.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
//        gyroParam.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        gyroParam.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//        gyroParam.loggingEnabled      = true;
//        gyroParam.loggingTag          = "IMU";
//        gyroParam.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

//        // Retrieve and initialize, the IMU. We expect the IMU to be attached to an I2C port
//        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
//        // and named "imu".
//        imu = hardwareMap.get(BNO055IMU.class,"imu");
//        imu.initialize(gyroParam);






        imu = new LynxEmbeddedIMU(new BetterI2cDeviceSynchImplOnSimple(
                new LynxI2cDeviceSynchV2(hardwareMap.appContext, module, 0), true
        ));

        imu.initialize(new BNO055IMU.Parameters());

        opMode.telemetry.addLine("ExH Version: " + getConciseLynxFirmwareVersion(module));
        opMode.telemetry.update();



    }


    private static class BetterI2cDeviceSynchImplOnSimple extends I2cDeviceSynchImplOnSimple {
        private BetterI2cDeviceSynchImplOnSimple(I2cDeviceSynchSimple simple, boolean isSimpleOwned) {
            super(simple, isSimpleOwned);
        }

        @Override
        public void setReadWindow(ReadWindow window) {
            // intentionally do nothing
        }
    }
    private static String getConciseLynxFirmwareVersion(LynxModule module) {
        String rawVersion = module.getFirmwareVersionString();
        String[] parts = rawVersion.split(" ");
        StringBuilder versionBuilder = new StringBuilder();
        for (int i = 0; i < 3; i++) {
            String part = parts[3 + 2*i];
            if (i == 2) {
                versionBuilder.append(part);
            } else {
                versionBuilder.append(part, 0, part.length() - 1);
                versionBuilder.append(".");
            }
        }
        return versionBuilder.toString();
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

    public void RunAtConstantVelocity(double x){
        FtcDashboard dashboard = FtcDashboard.getInstance();

        TelemetryPacket packet = new TelemetryPacket();

        double kp = RobotConstants.kp;

        int initialPos = motorFrontRight.getCurrentPosition();

        double v = 0;

        //creating time object so that kinematics approach can be used
        ElapsedTime time = new ElapsedTime();


        double targetVelocity = (lastLeftVelo+lastRightVelo)/2/537.6*12.56;

        double targetTime = x/targetVelocity;
        time.reset();
        while(time.seconds()<targetTime && opMode.opModeIsActive()){

            motorFrontRight.setVelocity(targetVelocity/12.56*537.6);
            motorBackRight.setVelocity(targetVelocity/12.56*537.6);
            motorFrontLeft.setVelocity(targetVelocity/12.56*537.6);
            motorBackLeft.setVelocity(targetVelocity/12.56*537.6);

        }
        lastRightVelo = motorFrontRight.getVelocity();
        lastLeftVelo = motorFrontLeft.getVelocity();


    }
    public void RuntoPositionTrapezoidalFirstHalf(double x, double maxVelocity, double maxAcceleration){
        FtcDashboard dashboard = FtcDashboard.getInstance();

        TelemetryPacket packet = new TelemetryPacket();

        double kp = RobotConstants.kp;

        int initialPos = motorFrontRight.getCurrentPosition();

        double v = 0;

        //creating time object so that kinematics approach can be used
        ElapsedTime time = new ElapsedTime();

        time.reset();

        //while loop for acceleration
        while(time.seconds()<Math.sqrt(2*x/(maxAcceleration)) && opMode.opModeIsActive()){

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

//            packet.put("proportional", feedback);
//            packet.put("pos",current + initialPos);
//            packet.put("expected", expected+initialPos);
//            dashboard.sendTelemetryPacket(packet);


        }
        lastRightVelo = motorFrontRight.getVelocity();
        lastLeftVelo = motorFrontLeft.getVelocity();
    }
    public void RuntoPositionTrapezoidalSecondHalf(double x){
        FtcDashboard dashboard = FtcDashboard.getInstance();

        TelemetryPacket packet = new TelemetryPacket();

        double kp = RobotConstants.kp;

        int initialPos = motorFrontRight.getCurrentPosition();

        double v = 0;

        //creating time object so that kinematics approach can be used
        ElapsedTime time = new ElapsedTime();


        double targetVelocity = (lastLeftVelo+lastRightVelo)/2/537.6*12.56;
        double initvelocity = targetVelocity;
        double a = Math.pow(targetVelocity,2)/2/x;
        opMode.telemetry.addData("initvel",targetVelocity);
        opMode.telemetry.addData("a",a);
        opMode.telemetry.update();

        time.reset();
        if(targetVelocity>0){
            while(targetVelocity> 0 && opMode.opModeIsActive()){
                targetVelocity = -a*time.seconds()+initvelocity;

                motorFrontRight.setVelocity(targetVelocity/12.56*537.6);
                motorBackRight.setVelocity(targetVelocity/12.56*537.6);
                motorFrontLeft.setVelocity(targetVelocity/12.56*537.6);
                motorBackLeft.setVelocity(targetVelocity/12.56*537.6);

            }
        }
        else{
            while(targetVelocity< 0 && opMode.opModeIsActive()){
                targetVelocity = -a*time.seconds()+initvelocity;

                motorFrontRight.setVelocity(targetVelocity/12.56*537.6);
                motorBackRight.setVelocity(targetVelocity/12.56*537.6);
                motorFrontLeft.setVelocity(targetVelocity/12.56*537.6);
                motorBackLeft.setVelocity(targetVelocity/12.56*537.6);

            }
        }

        lastRightVelo = motorFrontRight.getVelocity();
        lastLeftVelo = motorFrontLeft.getVelocity();

    }

    //proportional correction is working great now - woohoo
    // value of 1 works and is very tolerant to error but less smooth
    // value of 0.1 is optimal for most smooth path while being most accurate
    public void RuntoPositionTrapezoidal(double x, double maxVelocity, double maxAcceleration){

        FtcDashboard dashboard = FtcDashboard.getInstance();

        TelemetryPacket packet = new TelemetryPacket();

        double kp = RobotConstants.kp;

        int initialPos = motorFrontRight.getCurrentPosition();

        double v = 0;

        //creating time object so that kinematics approach can be used
        ElapsedTime time = new ElapsedTime();

        time.reset();

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

//            packet.put("proportional", feedback);
//            packet.put("pos",current + initialPos);
//            packet.put("expected", expected+initialPos);
//            dashboard.sendTelemetryPacket(packet);


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

//            packet.put("proportional", feedback);
//            packet.put("pos",current + initialPos);
//            packet.put("expected", expected+x/2.0);
//            dashboard.sendTelemetryPacket(packet);


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


    public void ArctoAngleFull(double theta, double maxVelocity,double alpha, double radius){
        //convert inches to ticks
        //theta in radianxs

        FtcDashboard dashboard = FtcDashboard.getInstance();

        TelemetryPacket packet = new TelemetryPacket();

        double kp = RobotConstants.kpaxial;
        double rightRad, leftRad;

        if(theta>0){
            rightRad = 2*RobotConstants.wheelDistance+radius;
            leftRad = radius;

        }
        else{
            rightRad = radius;
            leftRad = 2*RobotConstants.wheelDistance+radius;
        }

        double RightVelocity = 0;
        double LeftVelocity = 0;
        double prevtime = 0;

        double RightVelocity2 = 0;
        double LeftVelocity2 = 0;

        this.resetAngle();

        ElapsedTime time = new ElapsedTime();
        time.reset();
        //removed while opmodeisactive
        while(time.seconds()<Math.sqrt(theta/alpha) && opMode.opModeIsActive()) {

//            //x = r*theta
//            double expected = (0.5*maxAcceleration*Math.pow(time.seconds(),2) )/RobotConstants.wheelDistance;
//
////            //finding the current position
//            double current = this.getAngle();
//
//            //difference between the two is calculated
//            double error = (expected - currentheading)/12.56*537.6;
//
//            //error is translated into correction(PID)
//            double feedback = kp*(error);

            RightVelocity =(alpha*rightRad)*time.seconds()/12.56*537.6;
            LeftVelocity =(alpha*leftRad)*time.seconds()/12.56*537.6;

            motorFrontRight.setVelocity(RightVelocity);
            motorBackRight.setVelocity(RightVelocity);
            motorFrontLeft.setVelocity(LeftVelocity);
            motorBackLeft.setVelocity(LeftVelocity);

        }
        time.reset();
        while(time.seconds()<Math.sqrt(theta/alpha) && opMode.opModeIsActive()) {

//            //x = r*theta
//            double expected = (0.5*maxAcceleration*Math.pow(time.seconds(),2) )/RobotConstants.wheelDistance;
//
////            //finding the current position
//            double current = this.getAngle();
//
//            //difference between the two is calculated
//            double error = (expected - currentheading)/12.56*537.6;
//
//            //error is translated into correction(PID)
//            double feedback = kp*(error);

            RightVelocity2 = -(alpha*rightRad)*time.seconds()/12.56*537.6 + RightVelocity;
            LeftVelocity2 = -(alpha*leftRad)*time.seconds()/12.56*537.6 + LeftVelocity;

            motorFrontRight.setVelocity(RightVelocity2);
            motorBackRight.setVelocity(RightVelocity2);
            motorFrontLeft.setVelocity(LeftVelocity2);
            motorBackLeft.setVelocity(LeftVelocity2);

        }

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

    }
    public void ArctoAngleSecondHalf(double theta, double radius){
        double averageVelocity = (lastLeftVelo + lastRightVelo)/2/537.6*12.56;

        double targetRightVelocity, targetLeftVelocity;


        targetRightVelocity = 2*averageVelocity/(1+radius/(2*RobotConstants.wheelDistance+radius));
        targetLeftVelocity = targetRightVelocity*radius/(2*RobotConstants.wheelDistance+radius);

        ElapsedTime time = new ElapsedTime();
        time.reset();
        while(time.seconds()<theta*radius/targetLeftVelocity&& opMode.opModeIsActive()){
            opMode.telemetry.addData("targetLeftVelocity" ,targetLeftVelocity);
            opMode.telemetry.addData("targetRightVelocity" ,targetRightVelocity);
            opMode.telemetry.update();
            motorFrontRight.setVelocity(targetRightVelocity/12.56*537.6);
            motorBackRight.setVelocity(targetRightVelocity/12.56*537.6);
            motorFrontLeft.setVelocity(targetLeftVelocity/12.56*537.6);
            motorBackLeft.setVelocity(targetLeftVelocity/12.56*537.6);
        }
        lastRightVelo = motorFrontRight.getVelocity();
        lastLeftVelo = motorFrontLeft.getVelocity();




    }

    public void ArctoAngleFirstHalf(double theta, double maxVelocity,double alpha, double radius){


        //convert inches to ticks
        //theta in radianxs

        FtcDashboard dashboard = FtcDashboard.getInstance();

        TelemetryPacket packet = new TelemetryPacket();

        double kp = RobotConstants.kpaxial;
        double rightRad, leftRad;

        if(theta>0){
            rightRad = 2*RobotConstants.wheelDistance+radius;
            leftRad = radius;

        }
        else{
            rightRad = radius;
            leftRad = 2*RobotConstants.wheelDistance+radius;
        }

        double RightVelocity = 0;
        double LeftVelocity = 0;
        double prevtime = 0;

        double RightVelocity2 = 0;
        double LeftVelocity2 = 0;

        this.resetAngle();

        ElapsedTime time = new ElapsedTime();
        time.reset();
        //removed while opmodeisactive
        while(time.seconds()<Math.sqrt(2*theta/alpha) && opMode.opModeIsActive()) {

//            //x = r*theta
//            double expected = (0.5*maxAcceleration*Math.pow(time.seconds(),2) )/RobotConstants.wheelDistance;
//
////            //finding the current position
//            double current = this.getAngle();
//
//            //difference between the two is calculated
//            double error = (expected - currentheading)/12.56*537.6;
//
//            //error is translated into correction(PID)
//            double feedback = kp*(error);

            RightVelocity =(alpha*rightRad)*time.seconds()/12.56*537.6;
            LeftVelocity =(alpha*leftRad)*time.seconds()/12.56*537.6;

            motorFrontRight.setVelocity(RightVelocity);
            motorBackRight.setVelocity(RightVelocity);
            motorFrontLeft.setVelocity(LeftVelocity);
            motorBackLeft.setVelocity(LeftVelocity);

        }
        lastLeftVelo = LeftVelocity;
        lastRightVelo = RightVelocity;

    }
    public void TurntoAngleTrapezoidalGyroCorrection(double theta, double maxVelocity, double maxAcceleration){
        //convert inches to ticks
        //theta in radianxs

        FtcDashboard dashboard = FtcDashboard.getInstance();

        TelemetryPacket packet = new TelemetryPacket();



        double kp = RobotConstants.kpaxial;

        double x = theta*RobotConstants.wheelDistance;

        double v = 0;
        double prevtime = 0;

        this.resetAngle();

        ElapsedTime time = new ElapsedTime();
        time.reset();
        //removed while opmodeisactive
        while(time.seconds()<Math.sqrt(x/maxAcceleration) && opMode.opModeIsActive()) {

            //x = r*theta
            double expected = (0.5*maxAcceleration*Math.pow(time.seconds(),2) )/RobotConstants.wheelDistance;

//            //finding the current position
            double current = this.getAngle();

            //difference between the two is calculated
            double error = (expected - currentheading)/12.56*537.6;

            //error is translated into correction(PID)
            double feedback = kp*(error);

            v = maxAcceleration*time.seconds()/12.56*537.6;
            motorFrontRight.setVelocity(v+feedback);
            motorBackRight.setVelocity(v+feedback);
            motorFrontLeft.setVelocity(-v-feedback);
            motorBackLeft.setVelocity(-v-feedback);

//            packet.put("error", error);
//            packet.put("pos(angle in radians)",currentheading);
//            packet.put("expected(angle in radians)", expected);
//            dashboard.sendTelemetryPacket(packet);
//            opMode.telemetry.addData("time: ",time.milliseconds()-prevtime);
//            prevtime = time.milliseconds();
//            opMode.telemetry.update();


        }
        time.reset();
        while(time.seconds()<Math.sqrt(x/maxAcceleration) && opMode.opModeIsActive()){
            //x =r*theta
            double expected = (-0.5*maxAcceleration*Math.pow(time.seconds(),2)+v*time.seconds()/537.6*12.56)
                    /RobotConstants.wheelDistance + theta/2;
            //finding the current position
            double current = this.getAngle();

            //difference between the two is calculated
            double error = (expected - current)/12.56*537.6;

            //error is translated into correction(PID)
            double feedback = kp*(error);

            double v2 = -maxAcceleration*time.seconds()/12.56*537.6 + v;
            motorFrontRight.setVelocity(v2+feedback);
            motorBackRight.setVelocity(v2+feedback);
            motorFrontLeft.setVelocity(-v2-feedback);
            motorBackLeft.setVelocity(-v2-feedback);

//            packet.put("error", error);
//            packet.put("pos(angle in radians)",current);
//            packet.put("expected(angle in radians)", expected);
//            dashboard.sendTelemetryPacket(packet);
//            opMode.telemetry.addData("time: ",time.milliseconds()-prevtime);
//            prevtime = time.milliseconds();
//            opMode.telemetry.update();


        }
        time.reset();
        while(time.seconds()<Math.sqrt(x/maxAcceleration) && opMode.opModeIsActive()){
            double current  = this.getAngle();
            double error = (theta-current);
            double feedback = RobotConstants.kptrollturn*(error);
            motorFrontRight.setVelocity(feedback);
            motorBackRight.setVelocity(feedback);
            motorFrontLeft.setVelocity(-feedback);
            motorBackLeft.setVelocity(-feedback);

//            packet.put("error", feedback);
//            packet.put("pos", current);
//            dashboard.sendTelemetryPacket(packet);

        }
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);


    }
    public double getAngle(){

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        deltaheading = angles.firstAngle - lastheading;

//        opMode.telemetry.addData("delta", deltaheading);

        if (deltaheading < -Math.PI)
            deltaheading += 2*Math.PI ;
        else if (deltaheading >= Math.PI)
            deltaheading -= 2*Math.PI ;

        currentheading += deltaheading;

        lastheading = angles.firstAngle;

        return currentheading;

    }
    public void resetAngle(){
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        lastheading = angles.firstAngle;
        currentheading = 0;
    }




}
