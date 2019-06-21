package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Robot.Robot;

import java.util.List;

@Autonomous
public class WorldsDepotAuto extends LinearOpMode {
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

    private Robot robot;

    public ElapsedTime time = new ElapsedTime();

    private Servo teamMarkerServo;


    @Override
    public void runOpMode() throws InterruptedException {
        //construct robot object
        robot = new Robot(hardwareMap, this);

        //initialize servo to drop team marker
        robot.setTeamMarkerServo(1);


        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        robot.outake.setHangLock(1);
        robot.intake.setIntakeSlidePosition(0,0.3);

        telemetry.addData("System: ", "waiting for initalization");
        telemetry.update();
        waitForStart();

//        SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, ayganggang);

        /** Activate Tensor Flow Object Detection. */
        if (tfod != null) {
            tfod.activate();
        }



        //code that executes in match
        //go forward
        robot.intake.setIntakeSlidePosition(-150,1);
        robot.outake.unHangInAuto();

        robot.dt.RuntoPositionTrapezoidal(5,40,20);

        //bring down slides
        robot.outake.lowerSlidesAuto();
        robot.outake.setHangMotorPosition(12000,1);

        robot.intake.setIntakeSlidePosition(-1300,1);
        robot.intake.setIntakeDumpPosition(0.93);
        robot.dt.RuntoPositionTrapezoidal(5,40,20);
        sleep(1000);

        robot.intake.setSpinnerPower(0);
        robot.intake.setIntakeDumpPosition(0.23);
        robot.intake.setIntakeSlidePosition(0,1);
        robot.dt.RuntoPositionTrapezoidal(-5,-40,-20);
        robot.intake.setIntakeDumpPosition(0.93);
        sleep(1500);







        boolean scannedOnce = false;

        time.reset();

        while (opModeIsActive()) {
            if(time.milliseconds()>5000){
                if(scannedOnce){
                    telemetry.addLine("Not sure where gold is so guessing");

                    robot.intake.setSpinnerPower(-1);
                    robot.intake.setIntakeSlidePosition(-1000,1);
                    sleep(1500);
                    robot.intake.setSpinnerPower(0);
                    robot.intake.setIntakeDumpPosition(0.2);
                    robot.intake.setIntakeSlidePosition(0,0.8);
                    robot.intake.setStopperServoPosition(0.5);

                    robot.dt.TurntoAngleTrapezoidalGyroCorrection(-Math.PI/6-0.15,40,-20);
                }
                else{
                    robot.intake.setSpinnerPower(-1);
                    robot.intake.setIntakeSlidePosition(-1000,1);
                    sleep(1000);
                    robot.intake.setSpinnerPower(0);
                    robot.intake.setIntakeDumpPosition(0.2);

                    robot.intake.setIntakeSlidePosition(0,0.8);
                    robot.intake.setStopperServoPosition(0.5);

                    break;
                }



                break;
            }
            else if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if(updatedRecognitions.size() == 1){
                        if(updatedRecognitions.get(0).getLabel().equals(LABEL_GOLD_MINERAL)){
                            if(scannedOnce){
                                //left
                                robot.intake.setSpinnerPower(-1);
                                robot.intake.setIntakeSlidePosition(-1000,1);
                                sleep(1000);
                                robot.intake.setSpinnerPower(0);
                                robot.intake.setIntakeDumpPosition(0.2);
                                robot.intake.setIntakeSlidePosition(0,0.8);
                                robot.intake.setStopperServoPosition(0.5);


                                robot.dt.TurntoAngleTrapezoidalGyroCorrection(-Math.PI/6-0.1,40,-20);

                                break;

                            }
                            else{
                                //center
                                robot.intake.setSpinnerPower(-1);
                                robot.intake.setIntakeSlidePosition(-1000,1);
                                sleep(1000);
                                robot.intake.setSpinnerPower(0);
                                robot.intake.setIntakeDumpPosition(0.2);

                                robot.intake.setIntakeSlidePosition(0,0.8);
                                robot.intake.setStopperServoPosition(0.5);

                                break;
                            }


                        }
                        else if(updatedRecognitions.get(0).getLabel().equals(LABEL_SILVER_MINERAL)&& scannedOnce){
                            //right
                            robot.dt.TurntoAngleTrapezoidalGyroCorrection(-Math.PI/3+0.09,-40,-20);
                            robot.intake.setSpinnerPower(-1);
                            robot.intake.setIntakeSlidePosition(-1000,1);
                            sleep(1000);
                            robot.intake.setSpinnerPower(0);
                            robot.intake.setIntakeDumpPosition(0.2);
                            robot.intake.setIntakeSlidePosition(0,0.8);
                            robot.intake.setStopperServoPosition(0.5);


                            robot.dt.TurntoAngleTrapezoidalGyroCorrection(Math.PI/6-0.135,40,20);
                            break;
                        }
                        else if(updatedRecognitions.get(0).getLabel().equals(LABEL_SILVER_MINERAL)){
                            scannedOnce = true;
                            robot.dt.TurntoAngleTrapezoidal(Math.PI/6+0.1,40,20);
                        }
                    }


                }
            }
        }



        //drive to lander


        robot.outake.setDumpPos(0.15);
        robot.intake.setIntakeSlidePosition(-200,1);

        sleep(1500);

        robot.outake.setUpSlidePosition(-1440,1);

        time.reset();

        sleep(500);
        robot.outake.setDumpPos(0.93);
        sleep(1000);

        robot.outake.setDumpPos(0.1);

        robot.outake.lowerSlidesAuto();

        robot.dt.ArctoAngleFirstHalf(Math.PI/2-0.1,40,1.5,1);
        robot.dt.RunAtConstantVelocity(33);
        robot.dt.ArctoAngleSecondHalf(Math.PI/4+0.29,5);
        robot.dt.RuntoPositionTrapezoidalSecondHalf(5);
        robot.intake.setIntakeSlidePosition(-900,1);
        robot.intake.setIntakeDumpPosition(0.93);
        robot.intake.setSpinnerPower(-1);
        sleep(1000);
        robot.intake.setStopperServoPosition(-0.5);
        robot.intake.setIntakeSlidePosition(-1500,1);
        sleep(500);
        robot.dt.TurntoAngleTrapezoidalGyroCorrection(-Math.PI/12,-40,-20);

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
}

