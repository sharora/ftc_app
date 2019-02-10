package org.firstinspires.ftc.teamcode.FunctionTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class TestVelocityAcceleration extends LinearOpMode {
    DcMotorEx motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;



    public ElapsedTime time = new ElapsedTime();
    private BNO055IMU imu;



    @Override
    public void runOpMode() throws InterruptedException {


        motorFrontRight = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");

        motorFrontRight.setMode(RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(RunMode.RUN_USING_ENCODER);


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

        FtcDashboard dashboard = FtcDashboard.getInstance();

        TelemetryPacket packet = new TelemetryPacket();

        waitForStart();
        if (isStopRequested()) return;

        while(opModeIsActive()){
            motorFrontRight.setPower(gamepad1.right_stick_y);
            motorFrontLeft.setPower(gamepad1.left_stick_y);
            motorBackRight.setPower(gamepad1.right_stick_y);
            motorBackLeft.setPower(gamepad1.left_stick_y);




            packet.put("speed",motorFrontRight.getVelocity());
            dashboard.sendTelemetryPacket(packet);
        }






    }
}
