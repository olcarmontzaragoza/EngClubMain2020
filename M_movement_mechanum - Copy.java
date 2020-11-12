package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


@TeleOp

// motorTest = front left motor
// motor2 = front right motor
// motorTest3 = bottom left motor
// motorTest4 = bottom right motor
// motors 1/3 are both inverted
public class M_movement_mechanum extends LinearOpMode {
    private Gyroscope imu;
    private DcMotor motor0;
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private DcMotor motors1;
    private DcMotor motors2;
    private DigitalChannel digitalTouch;
    private Servo servo1;
    private Servo servo2;
    // private DistanceSensor sensorColorRange;
    //private Servo servo1;


    @Override
    public void runOpMode() {
        imu = hardwareMap.get(Gyroscope.class, "imu");
        motor0 = hardwareMap.get(DcMotor.class, "motor0");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motors1 = hardwareMap.get(DcMotor.class,"motors1");
        motors2 = hardwareMap.get(DcMotor.class,"motors2");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class,"servo2");

        //digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
        // sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        double tgtPower = 0;
        double verticalPower = 0;
        double horizontalPower = 0;
        double rotationPower = 0;
        double armpower;
        double armpower2;

        while (opModeIsActive()) {


            tgtPower = 1;
            verticalPower = this.gamepad1.left_stick_y;
            horizontalPower = this.gamepad1.left_stick_x;
            rotationPower = this.gamepad1.right_stick_x;
            armpower = Range.clip(gamepad2.left_stick_x, -10.0, 10.0);
            armpower2 = Range.clip(gamepad2.left_stick_y, -10.0, 10.0);


            if (gamepad1.atRest()) {
                motor0.setPower(0);
                motor1.setPower(0);
                motor2.setPower(0);
                motor3.setPower(0);
                telemetry.addData("Target Power", 0);
                telemetry.addData("Motor Power 0", motor0.getPower());
                telemetry.addData("Motor Power 1", motor1.getPower());
                telemetry.addData("Motor Power 2", motor2.getPower());
                telemetry.addData("Motor Power 3", motor3.getPower());
                telemetry.addData("Status", "Running");
                telemetry.update();
            }
            // to go straight
            if (Math.abs(verticalPower) > 0.1) {
                motor0.setPower(-verticalPower);
                motor1.setPower(verticalPower);
                motor2.setPower(-verticalPower);
                motor3.setPower(verticalPower);
                telemetry.addData("Target Power", verticalPower);
                telemetry.addData("Motor Power 0", motor0.getPower());
                telemetry.addData("Motor Power 1", motor1.getPower());
                telemetry.addData("Motor Power 2", motor2.getPower());
                telemetry.addData("Motor Power 3", motor3.getPower());
                telemetry.addData("Status", "Running");
                telemetry.update();
            }
            // to go horizontal
            if (Math.abs(horizontalPower) > 0.1) {
                motor0.setPower(horizontalPower);
                motor1.setPower(horizontalPower);
                motor2.setPower(-horizontalPower);
                motor3.setPower(-horizontalPower);
                telemetry.addData("Target Power", horizontalPower);
                telemetry.addData("Motor Power 0", motor0.getPower());
                telemetry.addData("Motor Power 1", motor1.getPower());
                telemetry.addData("Motor Power 2", motor2.getPower());
                telemetry.addData("Motor Power 3", motor3.getPower());
                telemetry.addData("Status", "Running");
                telemetry.update();
            }

            // to go diagonal (right)
            if (gamepad1.right_bumper) {
                motor0.setPower(tgtPower);
                motor3.setPower(-tgtPower);
                telemetry.addData("Target Power", tgtPower);
                telemetry.addData("Motor Power", motor0.getPower());
                telemetry.addData("Motor Power", motor3.getPower());
                telemetry.addData("Status", "Running");
                telemetry.update();

            }

            // to go diagonal (left)
            if (gamepad1.left_bumper) {
                motor1.setPower(-tgtPower);
                motor2.setPower(tgtPower);
                telemetry.addData("Target Power", tgtPower);
                telemetry.addData("Motor Power", motor1.getPower());
                telemetry.addData("Motor Power", motor2.getPower());
                telemetry.addData("Status", "Running");
                telemetry.update(); }

            // to go diagonal (right back)
            if (gamepad1.right_trigger > 0.5) {
                motor0.setPower(-tgtPower);
                motor3.setPower(tgtPower);
                telemetry.addData("Target Power", tgtPower);
                telemetry.addData("Motor Power", motor0.getPower());
                telemetry.addData("Motor Power", motor3.getPower());
                telemetry.addData("Status", "Running");
                telemetry.update();
            }

            // to go diagonal (left back)
            if (gamepad1.left_trigger > 0.5) {
                motor1.setPower(tgtPower);
                motor2.setPower(-tgtPower);
                telemetry.addData("Target Power", tgtPower);
                telemetry.addData("Motor Power", motor1.getPower());
                telemetry.addData("Motor Power", motor2.getPower());
                telemetry.addData("Status", "Running");
                telemetry.update(); }

            // rotation center (both)
            if (Math.abs(rotationPower) > 0.1) {
                motor0.setPower(rotationPower);
                motor1.setPower(rotationPower);
                motor2.setPower(rotationPower);
                motor3.setPower(rotationPower);
                telemetry.addData("Target Power", rotationPower);
                telemetry.addData("Motor Power 0", motor0.getPower());
                telemetry.addData("Motor Power 1", motor1.getPower());
                telemetry.addData("Motor Power 2", motor2.getPower());
                telemetry.addData("Motor Power 3", motor3.getPower());
                telemetry.addData("Status", "Running");
                telemetry.update();
            }
            if (gamepad1.y){
                motors1.setPower(1);
                motors2.setPower(-1);
                servo1.setPosition(0.3);
                servo2.setPosition(0.3);

            }

            else if (gamepad1.a){
                motors1.setPower(-1);
                motors2.setPower(1);
                servo1.setPosition(0.3);
                servo2.setPosition(0.3);
            }
            else {
                motors1.setPower(0);
                motors2.setPower(0);
                servo1.setPosition(0.3);
                servo2.setPosition(0.3);
            }





    }}}