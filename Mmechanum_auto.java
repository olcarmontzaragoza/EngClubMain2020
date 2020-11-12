package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

// motorTest = front left motor
// motor2 = front right motor
// motorTest3 = bottom left motor
// motorTest4 = bottom right motor
// motors 1/3 are both inverted

@Autonomous(name="Mmechanum_auto", group ="Concept")
public class Mmechanum_auto extends LinearOpMode {
    private Gyroscope imu;
    private DcMotor motor0;
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private DcMotor motorC1;
    private DcMotor motorC2;
    private DcMotor motorC3;

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
        motorC1 = hardwareMap.get(DcMotor.class, "motorC1");
        motorC2 = hardwareMap.get(DcMotor.class, "motorC2");
        motorC3 = hardwareMap.get(DcMotor.class, "motorC3");
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
            if (x<1) {
                motor0.setPower(1);
                motor1.setPower(1);
                motor2.setPower(-1);
                motor3.setPower(-1);
                x = 1;
                try {
                    Thread.sleep(800);
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
            ///for(x=0; x<2; x++){
                ///if (x<2) {
                   /// motor0.setPower(1.0);
                    ///motor3.setPower(-1.0);
                ///}
                ///if (x>2) {
                   /// motor0.setPower(0);
                   /// motor3.setPower(0);
                ///}
            }

            }
        }

