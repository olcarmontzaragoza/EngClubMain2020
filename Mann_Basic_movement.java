
package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DigitalChannel;
        import com.qualcomm.robotcore.hardware.DistanceSensor;
        import com.qualcomm.robotcore.hardware.Gyroscope;
        import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Mann_Basic_movement extends LinearOpMode {
    private Gyroscope imu;
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private DcMotor motor4;
    private DcMotor motor5;
    private DigitalChannel digitalTouch;
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
        while (opModeIsActive()) {
            tgtPower = this.gamepad1.left_stick_y;
            tgtPower2 = this.gamepad1.right_stick_y;
            motor1.setPower(tgtPower);
            telemetry.addData("Target Power", tgtPower);
            telemetry.addData("Motor Power", motor1.getPower());
            motor2.setPower(tgtPower2);
            telemetry.addData("Motor Power", motor2.getPower());
            motor3.setPower(tgtPower2);
            telemetry.addData("Motor Power", motor3.getPower());
            motor4.setPower(tgtPower);
            telemetry.addData("Motor Power", motor4.getPower());
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}