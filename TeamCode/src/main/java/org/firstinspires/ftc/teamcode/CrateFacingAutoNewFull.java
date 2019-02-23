package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Robot.Robot;

import java.util.List;

@Autonomous
public class CrateFacingAutoNewFull extends LinearOpMode {

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


    @Override
    public void runOpMode() throws InterruptedException {
        //construct robot object
        robot = new Robot(hardwareMap, this);

        //initialize servo to drop team marker
        robot.setTeamMarkerServo(0);

        int ayganggang   = hardwareMap.appContext.getResources()
                .getIdentifier("ayganggang",   "raw", hardwareMap.appContext.getPackageName());


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

//        SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, ayganggang);

        /** Activate Tensor Flow Object Detection. */
        if (tfod != null) {
            tfod.activate();
        }

        //code that executes in match
        robot.intake.setIntakeSlidePosition(-600, 0.5);
        sleep(300);
        robot.outake.unHangInAuto();

        //go forward
        robot.dt.RuntoPositionTrapezoidal(13.5,40,20);
        sleep(200);

        //bring down slides
        robot.outake.lowerSlidesAuto();

        //retract intake slides
        robot.intake.setIntakeSlidePosition(0, 0.5);


        //turn to get in position for scanning samples
        robot.dt.TurntoAngleTrapezoidalGyroCorrection(-Math.PI/2, 40, -20);

        int goldpos = 0;
        int silv1pos = 0;
        int silv2pos = 0;
        time.reset();

        while (opModeIsActive()) {
                        if(time.milliseconds()>500){
                            //gold is far right
                            telemetry.addLine("Not sure where gold is so guessing");
                            knockGoldLeft();

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
                                        telemetry.addLine("Gold is at Left");
                                        robot.setTeamMarkerServo(0.7);
                                        robot.dt.RuntoPositionTrapezoidal(-37,-40,-20);
                                        robot.setTeamMarkerServo(0);

                                        break;

                                    }

                                    for (Recognition recognition:updatedRecognitions) {
                                        if(recognition.getLabel().equals(LABEL_GOLD_MINERAL)){
                                            goldpos = (int)recognition.getLeft();
                                        }else{
                                            silv1pos = (int)recognition.getLeft();
                                        }
                                    }
                                    if(silv1pos>goldpos){
                                        //gold is in middle
                                        telemetry.addLine("Gold is at Center");
                                        robot.setTeamMarkerServo(0.7);
                                        robot.dt.RuntoPositionTrapezoidal(4,40,20);
                                        robot.setTeamMarkerServo(0);
                                        robot.dt.RuntoPositionTrapezoidal(-41,-40,-20);

                                        break;
                                    }
                                    else{
                                        //gold is at far right
                                        telemetry.addLine("Gold is at Right");
                                        robot.dt.RuntoPositionTrapezoidal(4,40,20);
                                        robot.setTeamMarkerServo(0.7);
                                        robot.dt.RuntoPositionTrapezoidal(8,40,20);
                                        robot.setTeamMarkerServo(0);
                                        robot.dt.RuntoPositionTrapezoidal(-49,-40,-20);
                                        break;
                                    }



                                }
                                else if(updatedRecognitions.size() == 1){
                                    if(updatedRecognitions.get(0).getLabel().equals(LABEL_GOLD_MINERAL)){
                                        if(updatedRecognitions.get(0).getLeft()>550){
                                            //gold is far right
                                            telemetry.addLine("Gold is at Right");
                                            robot.dt.RuntoPositionTrapezoidal(6,40,20);
                                            robot.setTeamMarkerServo(0.7);
                                            robot.dt.RuntoPositionTrapezoidal(6,40,20);
                                            robot.setTeamMarkerServo(0);
                                            robot.dt.RuntoPositionTrapezoidal(-49,-40,-20);
                                            //knock right
                                            break;

                                        }else{
                                            //gold is center
                                            telemetry.addLine("Gold is at Center");
                                            robot.setTeamMarkerServo(0.7);
                                            robot.dt.RuntoPositionTrapezoidal(8,40,20);
                                            robot.setTeamMarkerServo(0);
                                            robot.dt.RuntoPositionTrapezoidal(-41,-40,-20);
                                            //knock center
                                            break;
                                        }

                                    }
                                    if(updatedRecognitions.get(0).getLabel().equals(LABEL_SILVER_MINERAL)){
                                        if(updatedRecognitions.get(0).getLeft()<550){
                                            //silver is at right
                                            robot.dt.RuntoPositionTrapezoidal(6,40,20);
                                            time.reset();
                                            while (opModeIsActive()) {
                                                if(time.milliseconds()>1000){
                                                    telemetry.addLine("Guessing");
                                                    knockGoldRight();
                                                    break;
                                                }
                                                else if (tfod != null) {
                                                    telemetry.addLine("tfod works wow");
                                                    updatedRecognitions = tfod.getUpdatedRecognitions();
                                                    if(updatedRecognitions != null){
                                                        telemetry.addLine("list is not empty");
                                                        telemetry.addData("number of detects", updatedRecognitions.size());
                                                        if(updatedRecognitions.size() == 1){
                                                            if(updatedRecognitions.get(0).getLabel().equals(LABEL_SILVER_MINERAL)){
                                                                //knock left
                                                                telemetry.addLine("Gold is at Left");
                                                                robot.dt.RuntoPositionTrapezoidal(-6,-40,-20);
                                                                robot.setTeamMarkerServo(0.7);
                                                                robot.dt.RuntoPositionTrapezoidal(-37,-40,-20);
                                                                robot.setTeamMarkerServo(0);
                                                                break;
                                                            }
                                                            else{
                                                                //knock right
                                                                telemetry.addLine("Gold is at Right");
                                                                robot.dt.RuntoPositionTrapezoidal(8,40,20);
                                                                robot.setTeamMarkerServo(0.7);
                                                                robot.dt.RuntoPositionTrapezoidal(9,40,20);
                                                                robot.setTeamMarkerServo(0);
                                                                robot.dt.RuntoPositionTrapezoidal(-60,-40,-20);
                                                                break;
                                                            }
                                                        }
                                                    }

                                                }

                                            }
                                        }
                                        break;
                                    }
                                }

                }
            }
        }
        telemetry.update();
        robot.dt.TurntoAngleTrapezoidalGyroCorrection(-2.27,-40,-20);



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

    public void knockGoldRight(){

    }
    public void knockGoldCenter(){

    }
    public void knockGoldLeft(){

    }
}
