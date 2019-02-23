package org.firstinspires.ftc.teamcode.FunctionTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Robot.RobotConstants;

@Autonomous
@Disabled
public class TurnTest2 extends LinearOpMode {

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;



    public ElapsedTime time = new ElapsedTime();
    private BNO055IMU imu;



    @Override
    public void runOpMode() throws InterruptedException {


        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");


        motorBackLeft.setDirection(Direction.REVERSE);
        motorFrontLeft.setDirection(Direction.REVERSE);


        BNO055IMU.Parameters gyroParam = new BNO055IMU.Parameters();
        gyroParam.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
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

        //code

        turn(90,1);
//        badturn(87,1);
//        badturn(90.5,-0.35);


    }
    public void badturn(double angle, double speed){
        motorFrontRight.setPower(-speed);
        motorBackRight.setPower(-speed);
        motorBackLeft.setPower(speed);
        motorFrontLeft.setPower(speed);
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if(speed>0){
            while(angle - angles.firstAngle > 0){
                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
        }
        else{
            while(angle - angles.firstAngle < 0){
                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }

        }

        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontLeft.setPower(0);

    }
    public void turn(double angle, double speed){

        motorFrontRight.setMode(RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(RunMode.RUN_USING_ENCODER);

        FtcDashboard dashboard = FtcDashboard.getInstance();

        TelemetryPacket packet = new TelemetryPacket();
        //constant of proportionality
        double kp = RobotConstants.kp;
        double ki = RobotConstants.ki;
        double kd = RobotConstants.kd;
        double threshold = 0;
        //initializing the power variable
        double power;
        double dt;
        double time1 = 0;
        double time2 = 0;
        double roc;
        double sum = 0;





        //initializing the orientation object
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double initval = angles.firstAngle;

        //the starting error is equal to the target
        double target = angle + initval;
        double error = target - initval;
        double err = error;

        //resetting time
        time.reset();

        //run the loop while the difference between the current position and the target is not zero
        while(Math.abs(error)>threshold&& opModeIsActive()){

            time2 = time.time();
            dt = time2 - time1;
            time1 = time2;

            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            error = target - angles.firstAngle;
            roc = (error - err)/dt;
            sum += dt*error;


            power = (kp*error + ki*sum  + kd*roc)*speed;


            if(power < 0.08 && power > 0){
                motorFrontRight.setPower(-0.08);
                motorBackRight.setPower(-0.08);
                motorBackLeft.setPower(0.08);
                motorFrontLeft.setPower(0.08);
            }
            else if(power<0.08 && power<0){
                motorFrontRight.setPower(0.08);
                motorBackRight.setPower(0.08);
                motorBackLeft.setPower(-0.08);
                motorFrontLeft.setPower(-0.08);
            }
            else{
                //setting the motor powers
                motorFrontRight.setPower(-power);
                motorBackRight.setPower(-power);
                motorBackLeft.setPower(power);
                motorFrontLeft.setPower(power);
            }


            //Difference between the target and current position relative to the initial value
            err = error;




            packet.put("error", error);
            packet.put("angle", angles.firstAngle - initval);
            dashboard.sendTelemetryPacket(packet);


        }
        motorBackLeft.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontRight.setPower(0);


    }
}

