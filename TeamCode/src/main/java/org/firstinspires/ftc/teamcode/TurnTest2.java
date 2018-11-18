package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous
public class TurnTest2 extends LinearOpMode {

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    DcMotor leftLift;
    DcMotor rightLift;


    public ElapsedTime time = new ElapsedTime();
    private BNO055IMU imu;



    @Override
    public void runOpMode() throws InterruptedException {
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        leftLift = hardwareMap.dcMotor.get("leftLift");
        rightLift = hardwareMap.dcMotor.get("rightLift");

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

        //code

        turn(90,0.2);
        sleep(2000);
        turn(-90,0.2);

    }
    public void turn(double angle, double speed){
        //constant of proportionality
        double kp = 0.27;
        double ki = 0.15;
        double kd = 0.002;
        double threshold = 0.25;
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


            //setting the motor power based on the constant of proportionality
            power = kp*error + ki*sum  + kd*roc;

            //setting the motor powers
            motorFrontRight.setPower(-speed*power);
            motorBackRight.setPower(-speed*power);
            motorBackLeft.setPower(speed*power);
            motorFrontLeft.setPower(speed*power);

            //Difference between the target and current position relative to the initial value
            err = error;


            telemetry.addData("error: " , error);
            telemetry.addData("angles: ", angles.firstAngle);
            telemetry.update();


        }
        motorBackLeft.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontRight.setPower(0);


    }
}
