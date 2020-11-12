/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class BaseBuildOpMode3 extends LinearOpMode {
    private DcMotor motorLeft;
    private DcMotor motorRight;
    private DcMotor motorRaiseL;
    private DcMotor motorRaiseR;
    private Servo hook;
    private Servo servoUp;
    private Servo servoOpen;
    private DistanceSensor twoMDistance;
    private Gyroscope imu;
    private DigitalChannel digitalTouch;
    @Override
    public void runOpMode() {
        motorLeft = hardwareMap.get(DcMotor.class, "motorLeft");
        motorRight = hardwareMap.get(DcMotor.class, "motorRight");
        motorRaiseL = hardwareMap.get(DcMotor.class, "motorRaiseL");
        motorRaiseR = hardwareMap.get(DcMotor.class, "motorRaiseR");
        hook = hardwareMap.get(Servo.class, "hook");
        servoUp = hardwareMap.get(Servo.class, "ServoUp");
        servoOpen = hardwareMap.get(Servo.class, "ServoOpen");
        twoMDistance = hardwareMap.get(DistanceSensor.class, "twoMDistance");
        imu = hardwareMap.get(Gyroscope.class, "imu");
        digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");


        // set digital channel to input mode.
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY) waitForStart();
        waitForStart();

        // run until the end of the match (driver presses STOP)
        double tgtPower = 0;
        double tgtPower2 = 0;
        double tgtPowerLeft = 0;
        double tgtPowerRight = 0;
        double hookPosition = 1;
        while (opModeIsActive()) {
            telemetry.addData("Hook Position", hook.getPosition());
            telemetry.addData("Target Power", tgtPower);
            telemetry.addData("Target Power2", tgtPower2);
            telemetry.addData("Servo Open", servoOpen.getPosition());
            telemetry.addData("Servo Up", servoUp.getPosition());
            telemetry.addData("Motor Power", motorLeft.getPower());
            telemetry.addData("Motor Power2", motorRight.getPower());
            telemetry.addData("Distance (cm)", twoMDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Work done by KC:", "Still minimal");


            if(1 == 2) {

            } else if (gamepad1.x) {
                // Close arms
                servoOpen.setPosition(1);
            } else if (gamepad1.y) {
                // Open arms
            servoOpen.setPosition(0.2);
            }  else if (gamepad1.b) {
                //Move parallel to floor
                servoUp.setPosition(0.4);
            } else if (gamepad1.a) {
                //Move perpendicular to floor
                servoUp.setPosition(0);
            } else if (gamepad1.dpad_down) {
                //Move below robot
                servoUp.setPosition(0.6);
            } else if (gamepad1.dpad_up) {
                //Move less than perpendicular
                servoUp.setPosition(0.1);
            } else if (gamepad1.dpad_left) {
                long t= System.currentTimeMillis();
                long end = t+2000;
                while(System.currentTimeMillis() < end) {
                    motorRaiseL.setPower(-0.2);
                    motorRaiseR.setPower(0.2);
                }
            } else if (gamepad1.dpad_right) {
                long t= System.currentTimeMillis();
                long end = t+2000;
                while(System.currentTimeMillis() < end) {
                    motorRaiseL.setPower(0.2);
                    motorRaiseR.setPower(-0.2);
                }
            }


            // Move Forward/Backwards
            tgtPower = -this.gamepad1.left_stick_y;
            motorLeft.setPower(tgtPower);
            motorRight.setPower(-tgtPower);


            // Turn in a direction
            tgtPowerLeft = -this.gamepad1.left_stick_x;
            motorLeft.setPower(tgtPowerLeft);
            tgtPowerRight = -this.gamepad1.left_stick_x;
            motorRight.setPower(tgtPowerRight);

            // Raise HookArm
            tgtPower2 = -this.gamepad1.right_stick_y;
            motorRaiseL.setPower(tgtPower2);
            motorRaiseR.setPower(-tgtPower2);

            // Move Hook
            hookPosition = this.gamepad1.right_stick_x + 1;
            hook.setPosition(hookPosition);

            // is button pressed?
            if (!digitalTouch.getState()) {
            // button is pressed.
                telemetry.addData("Button", "PRESSED");
            } else {
                // button is not pressed.
                telemetry.addData("Button", "NOT PRESSED");
            }

            telemetry.addData("Status", "Running");
            telemetry.addData("Work done by KC:", "Still minimal");
            telemetry.update(); } } }