package org.firstinspires.ftc.teamcode;

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

/**
 * Created by Shreyas on 7/23/18.
 */
@Disabled
@Autonomous
public class TurnPID extends LinearOpMode {

    public ElapsedTime time = new ElapsedTime();
    private BNO055IMU imu;
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;
    double initval = 0;


    public void turn(double target, double speed){
        double kp = 0.27;
        double ki = 0;
        double kd = 0;
        double dt = 0;
        double sum = 0;
        double temp = 0;
        double roc = 0;
        double err = 0;
        double power;
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        initval = angles.firstAngle;
        double error = target - angles.firstAngle + initval;
        time.reset();
        while(Math.abs(angles.firstAngle-initval)!=Math.abs(target) && opModeIsActive()){
            dt = time.time() - temp;
            roc = (error - err)/dt;
            telemetry.addData("boom",roc);
            sum+= error*dt;
            power = kp*error + ki*sum + kd*roc;
            motorFrontRight.setPower(speed*power);
            motorBackRight.setPower(speed*power);
            motorFrontLeft.setPower(-speed*power);
            motorBackLeft.setPower(-speed*power);

            temp = time.time();
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("error", error);
            telemetry.update();
            err = error;
            error = target-angles.firstAngle+initval;
        }


    }

    @Override
    public void runOpMode() throws InterruptedException {


        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontLeft.setDirection(Direction.REVERSE);
        motorBackLeft.setDirection(Direction.REVERSE);
        motorFrontRight.setMode(RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(RunMode.RUN_USING_ENCODER);

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
        turn(90,-0.2);

    }
}
