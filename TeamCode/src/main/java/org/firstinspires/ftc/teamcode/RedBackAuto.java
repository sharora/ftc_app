package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous
public class RedBackAuto extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    //key needed to use Vuforia Library
    private static final String VUFORIA_KEY = "Aegsx6b/////AAAAGZ1XCL5uwk7gp+PMLDLPoOcm5/yyHm4ex0tWMj1G+87mVQHnJ6oK2EdH" +
            "HthatiYRvKuuvmegcsrLkbjEL7IzSGCh9pjtiavsoCBwMcB1rtOyjwv1X+Veys1noJNxEZF8W7tSXyWDvigaqNmj8y/fIQ+Q03SkEXlytT" +
            "qMTHgSpcs8l1qbd4o22QrfCik+i/YYrpdOPU82yNY54jmdfPX5r8gEt1zboWugVcwewkh7TL8f00CDz4TgvBXdqZN4k76GLdwxKhXIe9Th" +
            "EGS/ghb/yyYoXCmZwX6MZN62V3BcAjiIowbZDkUtlozp2eiAJl/7O4/WXfiKhl+g7bMlFT99ID7m7wWZYmSX/7A4zJsVpE+Q";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    DcMotor leftLift;
    DcMotor rightLift;

    DcMotor intakeDump;

    Servo hangLock;

    Servo TeamMarker;

    CRServo spinnerR;
    CRServo spinnerL;



    public ElapsedTime time = new ElapsedTime();
    private BNO055IMU imu;
    double initval = 0;

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

        TeamMarker = hardwareMap.servo.get("TeamMarker");

        intakeDump = hardwareMap.dcMotor.get("intakeDump");

        spinnerR = hardwareMap.crservo.get("spinnerR");
        spinnerL = hardwareMap.crservo.get("spinnerL");

        hangLock = hardwareMap.servo.get("hangLock");

        BNO055IMU.Parameters gyroParam = new BNO055IMU.Parameters();
        gyroParam.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gyroParam.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyroParam.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        gyroParam.loggingEnabled = true;
        gyroParam.loggingTag = "IMU";
        gyroParam.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize, the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(gyroParam);

        TeamMarker.setPosition(0);


        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        telemetry.addData("System: ", "waiting for initalization");
        telemetry.update();
        waitForStart();

        //code that executes in match
        intakeDump.setPower(0.0125);
        moveLift(-4150, -0.7);
        sleep(300);
        encoderturn(300, 0.15);
        encoderturn(-300, -0.15);
        driveForward(1280, 0.5);
        turn(90, 0.2);

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                while (opModeIsActive()) {
                    updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        if (updatedRecognitions.size() == 1) {
                            //position2
                            if (updatedRecognitions.get(0).getLabel().equals(LABEL_GOLD_MINERAL)) {
                                turn(-90, 0.2);
                                driveForward(800, 0.4);
                                driveForward(-800, -0.4);
                                turn(90, 0.2);
                                driveForward(1400, 0.5);

                                break;

                            } else if (updatedRecognitions.get(0).getLabel().equals(LABEL_SILVER_MINERAL)) {
                                driveForward(-1500, -0.4);
                                while (opModeIsActive()) {
                                    updatedRecognitions = tfod.getUpdatedRecognitions();
                                    //position3
                                    if (updatedRecognitions != null) {
                                        if (updatedRecognitions.size() == 1) {
                                            if (updatedRecognitions.get(0).getLabel().equals(LABEL_GOLD_MINERAL)) {
                                                turn(-90, 0.2);
                                                driveForward(800, 0.4);
                                                driveForward(-800, -0.4);
                                                turn(88, 0.2);
                                                driveForward(2900, 0.5);


                                                break;

                                            }
                                            //position1
                                            else if (updatedRecognitions.get(0).getLabel().equals(LABEL_SILVER_MINERAL)) {
                                                driveForward(2900, 0.4);
                                                turn(-90, 0.2);
                                                driveForward(800, 0.4);
                                                driveForward(-800, -0.4);
                                                turn(90, 0.2);

                                                break;

                                            }
                                        }

                                    }

                                }
                                break;

                            }

                        }
                        if (updatedRecognitions.size() == 2) {
                            driveForward(-100, -0.4);
                        }


                    }

                }


            }
            driveForward(2200, 0.5);
            turn(45, 0.2);
            driveForward(3500, 0.6);
            TeamMarker.setPosition(1);


//            if (tfod != null) {
//                // getUpdatedRecognitions() will return null if no new information is available since
//                // the last time that call was made.
//
//                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//
//                updatedRecognitions = tfod.getUpdatedRecognitions();
//                if (updatedRecognitions != null) {
//                    telemetry.addData("# Object Detected", updatedRecognitions.size());
//                    if (updatedRecognitions.size() == 1) {
//                        telemetry.addData("object", updatedRecognitions.get(0));
//                        if (updatedRecognitions.get(0).getLabel().equals(LABEL_GOLD_MINERAL)) {
//                            turn(90, 0.2);
//                            driveForward(-800, -0.4);
//
//                        } else if (updatedRecognitions.get(0).getLabel().equals(LABEL_SILVER_MINERAL)) {
//                            driveForward(-110, -0.4);
//
//
//                        }
//
//                    }
//
//
//                    telemetry.update();
//                }
//            }


        }

        if (tfod != null) {
            tfod.shutdown();
        }
        stop();

    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
    public void driveForward(int ticks, double power) {
        motorFrontLeft.setMode(RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(RunMode.STOP_AND_RESET_ENCODER);

        motorBackRight.setTargetPosition(-ticks);
        motorBackLeft.setTargetPosition(-ticks);
        motorFrontRight.setTargetPosition(-ticks);
        motorFrontLeft.setTargetPosition(-ticks);


        motorFrontRight.setMode(RunMode.RUN_TO_POSITION);
        motorFrontLeft.setMode(RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(RunMode.RUN_TO_POSITION);


        motorBackLeft.setPower(power);
        motorBackRight.setPower(power);
        motorFrontLeft.setPower(power);
        motorFrontRight.setPower(power);


        while (motorBackLeft.isBusy() && motorBackRight.isBusy() && motorFrontLeft.isBusy() && motorFrontRight.isBusy()&& opModeIsActive()) {
        }

        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);

        motorFrontLeft.setMode(RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(RunMode.RUN_USING_ENCODER);


    }
    public void encoderturn (int ticks, double power) {
        motorFrontLeft.setMode(RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(RunMode.STOP_AND_RESET_ENCODER);

        motorBackRight.setTargetPosition(ticks);
        motorBackLeft.setTargetPosition(-ticks);
        motorFrontRight.setTargetPosition(ticks);
        motorFrontLeft.setTargetPosition(-ticks);


        motorFrontRight.setMode(RunMode.RUN_TO_POSITION);
        motorFrontLeft.setMode(RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(RunMode.RUN_TO_POSITION);


        motorBackLeft.setPower(-power);
        motorBackRight.setPower(power);
        motorFrontLeft.setPower(-power);
        motorFrontRight.setPower(power);


        while (motorBackLeft.isBusy() && motorBackRight.isBusy() && motorFrontLeft.isBusy() && motorFrontRight.isBusy()&& opModeIsActive()) {
        }

        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);

        motorFrontLeft.setMode(RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(RunMode.RUN_USING_ENCODER);


    }
    public void turn(double angle, double speed){


        //constant of proportionality
        double kp = 0.26;
        double ki = 0;
        double kd = 0;
        double threshold = 0.1;
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
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        initval = angles.firstAngle;

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
    public void moveLift(int ticks,double power){
        leftLift.setMode(RunMode.RUN_USING_ENCODER);
        rightLift.setMode(RunMode.STOP_AND_RESET_ENCODER);

        rightLift.setTargetPosition(-ticks);

        rightLift.setMode(RunMode.RUN_TO_POSITION);

        rightLift.setPower(power);



        while (rightLift.isBusy() && opModeIsActive()) {
            leftLift.setPower(-0.2);
            if(rightLift.getCurrentPosition()>1500 && rightLift.getCurrentPosition()<2700){
                motorBackLeft.setPower(-0.2);
                motorFrontLeft.setPower(-0.2);
                motorBackRight.setPower(-0.2);
                motorFrontRight.setPower(-0.2);
                telemetry.addData("encoder pos", rightLift.getCurrentPosition());
                telemetry.update();

            }
            motorBackLeft.setPower(0);
            motorFrontLeft.setPower(0);
            motorBackRight.setPower(0);
            motorFrontRight.setPower(0);
            leftLift.setPower(0);


        }

        rightLift.setPower(0);

        rightLift.setMode(RunMode.RUN_USING_ENCODER);

    }
}
