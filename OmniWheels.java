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
public class OmniWheels extends LinearOpMode {
    private DcMotor motorLeft;
    private DcMotor motorRight;
    private DcMotor motorFront;
    private DcMotor motorBack;
    private DistanceSensor twoMDistance;
    private Gyroscope imu;
    private DigitalChannel digitalTouch;
    @Override
    public void runOpMode() {
        motorLeft = hardwareMap.get(DcMotor.class, "motorLeft");
        motorRight = hardwareMap.get(DcMotor.class, "motorRight");
        motorFront = hardwareMap.get(DcMotor.class, "motorFront");
        motorBack= hardwareMap.get(DcMotor.class, "motorBack");
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
        double tgtPowerTurn = 0;

        while (opModeIsActive()) {
            telemetry.addData("Target Power", tgtPower);
            telemetry.addData("Target Power2", tgtPower2);
            telemetry.addData("Target Power Turn", tgtPowerTurn);
            telemetry.addData("Motor Power Left", motorLeft.getPower());
            telemetry.addData("Motor Power Right", motorRight.getPower());
            telemetry.addData("Motor Power Front", motorFront.getPower());
            telemetry.addData("Motor Power Back", motorBack.getPower());
            telemetry.addData("Distance (cm)", twoMDistance.getDistance(DistanceUnit.CM));

            // Move Forward/Backwards
            tgtPower = -this.gamepad1.left_stick_y;
            motorFront.setPower(tgtPower);
            motorBack.setPower(-tgtPower);

            // Move Forward/Backwards
            tgtPower2 = -this.gamepad1.left_stick_x;
            motorLeft.setPower(-tgtPower);
            motorRight.setPower(tgtPower);

            // Turn in a direction
            tgtPowerTurn = -this.gamepad1.left_stick_x;
            motorLeft.setPower(tgtPowerTurn);
            motorRight.setPower(-tgtPowerTurn);


            telemetry.addData("Status", "Running");
            telemetry.update(); } } }