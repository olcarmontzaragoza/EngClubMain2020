package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

// motorTest = front left motor
// motor2 = front right motor
// motorTest3 = bottom left motor
// motorTest4 = bottom right motor
// motors 1/3 are both inverted

@Autonomous(name="Mmechanum_autoV2_1", group ="Concept")
public class Mmechanum_autoV2_1 extends LinearOpMode {
    private Gyroscope imu;
    private DcMotor motor0;
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private DcMotor motors1;
    private DcMotor motors2;
    private Servo servo1;
    private Servo servo2;

    //private DigitalChannel digitalTouch;
    // private DistanceSensor sensorColorRange;
    //private Servo servo1;


    @Override
    public void runOpMode() {
        imu = hardwareMap.get(Gyroscope.class, "imu");
        motor0 = hardwareMap.get(DcMotor.class, "motor0");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motors1 = hardwareMap.get(DcMotor.class, "motors1");
        motors2 = hardwareMap.get(DcMotor.class, "motors2");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        int x=0;
       // digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
        // sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
        //servo1 = hardwareMap.get(Servo.class, "servo");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        double tgtPower = 0;
        double verticalPower = 0;
        double horizontalPower = 0;
        double rotationPower = 0;
        while (opModeIsActive()) {
            tgtPower = 1;
            verticalPower = this.gamepad1.left_stick_y;
            horizontalPower = this.gamepad1.left_stick_x;
            rotationPower = this.gamepad1.right_stick_x;
            motors1.setPower(0);
            motors2.setPower(0);

            //move under the bridge
            if (x<1) {
                motor0.setPower(-1);
                motor1.setPower(1);
                motor2.setPower(-1);
                motor3.setPower(1);
                x = 1;
                try {
                    Thread.sleep(600);
                } catch (InterruptedException ex) {
                    // Stop immediately and go home
                }
            }

            else {
                motor0.setPower(0);
                motor1.setPower(0);
                motor2.setPower(0);
                motor3.setPower(0);
            }
            //preparing to grab
            servo1.setPosition(0.3);
            telemetry.addData("Servo Position", servo1.getPosition());
            try {
                Thread.sleep(200);
            } catch (InterruptedException ex) {
                // Stop immediately and go home
            }
            servo2.setPosition(0.3);
            telemetry.addData("Servo Position", servo2.getPosition());

            motors1.setPower(1);
            motors2.setPower(-1);
            try {
                Thread.sleep(200);
            } catch (InterruptedException ex) {
                // Stop immediately and go home
            }
            motors1.setPower(0);
            motors2.setPower(-0);

            }
        }}

