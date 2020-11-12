package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;



@TeleOp

// motorTest = front left motor
// motor2 = front right motor
// motorTest3 = bottom left motor
// motorTest4 = bottom right motor
// motors 1/3 are both inverted in rotation direction
public class Marcel_Test extends LinearOpMode {
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
    private Servo servo1;
    private Servo servo2;



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
        //digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
        // sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
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
                telemetry.addData("Status", "At Rest");
                telemetry.update();
            }
            // to go straight
            if (Math.abs(verticalPower) > 0.3) {
                motor0.setPower(verticalPower);
                motor1.setPower(-verticalPower);
                motor2.setPower(verticalPower);
                motor3.setPower(-verticalPower);
                telemetry.addData("Target Power", verticalPower);
                telemetry.addData("Motor Power 0", motor0.getPower());
                telemetry.addData("Motor Power 1", motor1.getPower());
                telemetry.addData("Motor Power 2", motor2.getPower());
                telemetry.addData("Motor Power 3", motor3.getPower());
                telemetry.addData("Status", "Running");
                telemetry.update();
            }
            // to go horizontal
            if (Math.abs(horizontalPower) > 0.3) {
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


        }
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

        // Suck mech in
        if (gamepad1.dpad_down) {
            motorC1.setPower(tgtPower);
            motorC2.setPower(-tgtPower);
            telemetry.addData("Target Power", tgtPower);
            telemetry.addData("Motor Crane Power 1", motorC1.getPower());
            telemetry.addData("Motor Crane Power 2", motorC2.getPower());
        }

        // suck mech out
        if (gamepad1.dpad_up) {
            motorC1.setPower(-tgtPower);
            motorC2.setPower(tgtPower);
            telemetry.addData("Target Power", -tgtPower);
            telemetry.addData("Motor Crane Power 1", motorC1.getPower());
            telemetry.addData("Motor Crane Power 2", motorC2.getPower());
        }

        // move servo motor
        if(gamepad1.y) {
            // move to 0 degrees.
            servo1.setPosition(0);
        } else if (gamepad1.x || gamepad1.b) {
            // move to 90 degrees.
            servo1.setPosition(0.5);
        } else if (gamepad1.a) {
            // move to 180 degrees.
            servo1.setPosition(1);
            telemetry.addData("Servo Position", servo1.getPosition());
        }



    }}