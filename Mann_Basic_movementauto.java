
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gyroscope;

@TeleOp(name="Mann_Basic_movementauto", group ="Concept")
public class Mann_Basic_movementauto extends LinearOpMode {
    private Gyroscope imu;
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private DcMotor motor4;
    private DcMotor motor5;
    private DigitalChannel digitalTouch;
    int x;
    //private DistanceSensor sensorColorRange;
    //private Servo servoTest;
    @Override
    public void runOpMode() {
        imu = hardwareMap.get(Gyroscope.class, "imu");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");
        digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
        x = 0;
        //sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
        //servoTest = hardwareMap.get(Servo.class, "servoTest");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        double tgtPower = 0;
        double tgtPower2 = 0;
        double tgtPower3 = 0;
        double x = 0;

        while (opModeIsActive()) {
            if (x<1) {
                motor1.setPower(1);
                motor2.setPower(1);
                motor3.setPower(-1);
                motor4.setPower(-1);
                x = 1;
                try {
                    Thread.sleep(600);
                } catch (InterruptedException ex) {
                    // Stop immediately and go home
                }
            }
            else {
                motor1.setPower(0);
                motor2.setPower(0);
                motor3.setPower(0);
                motor4.setPower(0);
            }

    }
}}