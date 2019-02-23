package org.firstinspires.ftc.teamcode.OldPrograms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Robot.RobotConstants;

import java.util.List;

@Autonomous
@Disabled

public class CraterFacingAuto extends LinearOpMode {

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
    DcMotor hangingMotor;
    DcMotor verticalLift;
    DcMotor intakeDump;
    DcMotor intakeSlides;

    DistanceSensor sensorRange;

    DigitalChannel mgLimVert;


    CRServo spinner;

    Servo dumper1;
    Servo dumper2;
    Servo teamMarker;


    public ElapsedTime time = new ElapsedTime();
    private BNO055IMU imu;
    double initval = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();

        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        hangingMotor = hardwareMap.dcMotor.get("hangingMotor");
        verticalLift = hardwareMap.dcMotor.get("verticalLift");
        verticalLift.setDirection(Direction.REVERSE);
        verticalLift.setMode(RunMode.RUN_TO_POSITION);

        intakeDump = hardwareMap.dcMotor.get("intakeDump");

        mgLimVert = hardwareMap.digitalChannel.get("mgLimVert");

        intakeDump.setMode(RunMode.RUN_TO_POSITION);
        intakeSlides = hardwareMap.dcMotor.get("intakeSlides");



        motorBackLeft.setDirection(Direction.REVERSE);
        motorFrontLeft.setDirection(Direction.REVERSE);

        dumper1 = hardwareMap.servo.get("dumper1");
        dumper2 = hardwareMap.servo.get("dumper2");

        spinner = hardwareMap.crservo.get("spinner");
        teamMarker = hardwareMap.servo.get("teamMarker");

        sensorRange = hardwareMap.get(DistanceSensor.class, "sensorRange");


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
        teamMarker.setPosition(0);



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

        /** Activate Tensor Flow Object Detection. */
        if (tfod != null) {
            tfod.activate();
        }

//        code that executes in match

        intakeSlides.setPower(-0.3);
        sleep(200);
        intakeSlides.setPower(0);
        moveLift(15200, 1);
        driveForward(-50,0.4);
        driveForward(1000,0.6);
        while(mgLimVert.getState()){
            verticalLift.setPower(0);
        }
        verticalLift.setTargetPosition(0);
        verticalLift.setPower(0.2);
        encoderturn(105, 0.6);
        driveForward(200,0.5);




        int goldpos = 0;
        int silvpos = 0;
        time.reset();

        while (opModeIsActive()) {
            if(time.milliseconds()>500){
                //gold is far left(robot perspective)
                telemetry.addData("Gold is at Position", 1);
                driveForward(-300,0.7);
                encoderturn(50, 0.7);
                driveForward(-1800,0.7);
                driveForward(1800,0.7);
                encoderturn(-50, 0.7);
                driveForward(300,0.5);
                break;
            }
            else if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if(updatedRecognitions.size() == 2){
                        if(updatedRecognitions.get(0).getLabel().equals(LABEL_SILVER_MINERAL) &&
                                updatedRecognitions.get(1).getLabel().equals(LABEL_SILVER_MINERAL)){
                            //gold is far left(robot perspective)
                            telemetry.addData("Gold is at Position", 1);
                            driveForward(-300,0.7);
                            encoderturn(40, 0.7);
                            driveForward(-1800,0.7);
                            driveForward(1800,0.7);
                            encoderturn(-40, 0.7);
                            driveForward(300,0.7);
                            break;

                        }

                        for (Recognition recognition:updatedRecognitions) {
                            if(recognition.getLabel().equals(LABEL_GOLD_MINERAL)){
                                goldpos = (int)recognition.getLeft();
                            }else{
                                silvpos = (int)recognition.getLeft();
                            }
                        }
                        if(silvpos>goldpos){
                            //gold is in middle
                            telemetry.addData("Gold is at Position", 2);
                            encoderturn(-105, 0.7);
                            driveForward(1500,0.7);
                            driveForward(-1500,0.7);
                            encoderturn(105, 0.7);
                            break;
                        }
                        else{
                            //gold is at far right
                            telemetry.addData("Gold is at Position", 3);
                            encoderturn(-50, 0.7);
                            driveForward(1500,0.7);
                            driveForward(-1500,0.7);
                            encoderturn(50, 0.7);
                            break;
                        }



                    }
                    else if(updatedRecognitions.size() == 1){
                        if(updatedRecognitions.get(0).getLabel().equals(LABEL_GOLD_MINERAL)){
                            if(updatedRecognitions.get(0).getLeft()>550){
                                //gold is far right
                                encoderturn(-50, 0.7);
                                driveForward(1500,0.7);
                                driveForward(-1500,0.7);
                                encoderturn(50, 0.7);
                                break;

                            }else{
                                //gold is center
                                telemetry.addData("Gold is at Position", 2);
                                encoderturn(-105, 0.7);
                                driveForward(1500,0.7);
                                driveForward(-1500,0.7);
                                encoderturn(105, 0.7);
                                break;
                            }

                        }else if(updatedRecognitions.get(0).getLabel().equals(LABEL_GOLD_MINERAL)){
                            if(updatedRecognitions.get(0).getLeft()>550){
                                driveForward(-100,0.7);
                            }else{
                                driveForward(100,0.7);
                            }


                        }
                    }

                }
            }
        }
        telemetry.update();
        while(sensorRange.getDistance(DistanceUnit.INCH)>7 && opModeIsActive()){
            motorBackLeft.setPower(0.7);
            motorBackRight.setPower(0.7);
            motorFrontLeft.setPower(0.7);
            motorFrontRight.setPower(0.7);
        }
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
//        absoluteturn(-50,0.8);
        badturn(-58,0.32);
//        badturn(-50, -0.35);

        driveForward(-3500,1);
        encoderturn(-45,0.8);
        teamMarker.setPosition(1);
        sleep(200);
        encoderturn(45,0.8);
        sleep(300);
        driveForward(5300,1);
        intakeSlides.setPower(-0.4);
        sleep(200);
        intakeSlides.setPower(0);

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
    public void encoderturn (double degrees, double power) {
        double wheelDistance = 15;
        int ticks = (int)((degrees/360)*wheelDistance*Math.PI*1280/12.56);


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
    public void absoluteturn(double angle, double speed){

        motorFrontRight.setMode(RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(RunMode.RUN_USING_ENCODER);


        //constant of proportionality
        double kp = RobotConstants.kp;
        double ki = RobotConstants.ki;
        double kd = RobotConstants.kd;
        double threshold = 0.126;
        //initializing the power variable
        double power;
        double dt;
        double time1 = 0;
        double time2 = 0;
        double roc;
        double sum = 0;




        //initializing the orientation object
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


        //the starting error is equal to the target
        double target = angle;
        double error = target - angles.firstAngle;
        double err = error;

        //resetting time
        time.reset();

        //run the loop while the difference between the current position and the target is not zero
        //prev bool was Math.abs(error)>threshold
        while(Math.abs(error)>threshold && opModeIsActive()&& time.milliseconds()<4000){

            time2 = time.time();
            dt = time2 - time1;
            time1 = time2;

            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            error = target - angles.firstAngle;
            roc = (error - err)/dt;
            sum += dt*error;


            //setting the motor power based on the constant of proportionality
            power = (kp*error + ki*sum  + kd*roc)*speed;


//            if(power < 0.15 && power > 0){
//                motorFrontRight.setPower(-0.15);
//                motorBackRight.setPower(-0.15);
//                motorBackLeft.setPower(0.15);
//                motorFrontLeft.setPower(0.15);
//            }
//            else if(power<0.15 && power<0){
//                motorFrontRight.setPower(0.15);
//                motorBackRight.setPower(0.15);
//                motorBackLeft.setPower(-0.15);
//                motorFrontLeft.setPower(-0.15);
//            }
//            else{
//                //setting the motor powers
//                motorFrontRight.setPower(-power);
//                motorBackRight.setPower(-power);
//                motorBackLeft.setPower(power);
//                motorFrontLeft.setPower(power);
//            }
            motorFrontRight.setPower(-power);
            motorBackRight.setPower(-power);
            motorBackLeft.setPower(power);
            motorFrontLeft.setPower(power);

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
        verticalLift.setMode(RunMode.STOP_AND_RESET_ENCODER);
        hangingMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);

        verticalLift.setTargetPosition(-ticks/8);
        hangingMotor.setTargetPosition(ticks + 800);


        verticalLift.setMode(RunMode.RUN_TO_POSITION);
        hangingMotor.setMode(RunMode.RUN_TO_POSITION);

        hangingMotor.setPower(power);
        verticalLift.setPower(power/8);



        while (hangingMotor.isBusy() && opModeIsActive() && verticalLift.isBusy()) {


        }

        hangingMotor.setPower(0);

        hangingMotor.setMode(RunMode.RUN_USING_ENCODER);


    }
    public void badturn(double angle, double speed){

        motorFrontRight.setMode(RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(RunMode.RUN_USING_ENCODER);

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
}
