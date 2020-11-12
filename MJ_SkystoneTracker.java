package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

/**
 * This 2019-2020 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Skystone game elements.
 */
@TeleOp(name = "MJ_SkystoneTracker", group = "Concept")
public class MJ_SkystoneTracker extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final String VUFORIA_KEY = "AaELO77/////AAABmddIcskHlExFsZi/SX3A5ChRUJbFGxCtEX/kYRQ892XGS3xnO4iW8NpydPzB7vflBSBZ6y5v/mPhaQdoksPF4cuhOHLeH62478YSBTEu97cxSzlPr0gb02vUi1OB6yFdj5nSk0mb5nCnkOPqNA7g57ngRDu1jUwyte8DAcXJFhII7WBIDko/j68qWAdEcepaxglHyY5E2P4YcdpVpGXertgTjFYXhsszKiWU2e6ogMEsteR6ktLHUeH+TqitDtDSa5XZRvpCxDPHBNiwziSV/kUUQTfXHCkAS8vk+u6gDTyl9rMj++tpZKypMNn58c/ePAXOpwoDWyCvO1I4pslogivjCAFF7UVKEJnlO0EJteGX";
    // Detector object
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private DcMotor motor4;
    // private DistanceSensor twoMDistance;
    // private DistanceSensor sensorColorRange;
    private Gyroscope imu;
    private DigitalChannel digitalTouch;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();


        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        /** Wait for the game to begin */
        // twoMDistance = hardwareMap.get(DistanceSensor.class, "twoMDistance");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");
        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 0) {
                            motor1.setPower(0);
                            motor2.setPower(0);
                        }
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            String stone_type = recognition.getLabel();
                            if (stone_type == "Skystone") {
                                motor1.setPower(0.3);
                                motor2.setPower(0.3);
                                motor3.setPower(0.3);
                                motor4.setPower(0.3);}
                            else if (stone_type == "Stone") {
                                motor1.setPower(0);
                                motor2.setPower(0);
                                motor3.setPower(0);
                                motor4.setPower(0);}
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                        }

                        telemetry.update();
                    }


                        /*
                        if (xValue < 290) {
                            motorLeft.setPower(-0.15);
                            motorRight.setPower(0.15);
                        } else if(xValue > 350) {
                            motorLeft.setPower(0.15);
                            motorRight.setPower(-0.15);}
                        else {
                            motorLeft.setPower(0);
                            motorRight.setPower(0);
                            motorBack.setPower(0);
                            motorFront.setPower(0);

                            double current_Distance = twoMDistance.getDistance(DistanceUnit.CM);
                            if(current_Distance > 30) {
                                motorFront.setPower(0.15);
                                motorBack.setPower(-0.15); }
                        } */ }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
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

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.4;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
        // tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_SECOND_ELEMENT);
    }
}
