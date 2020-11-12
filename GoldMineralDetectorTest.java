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

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name="GoldMineralDetectorTest", group="DogeCV")

public class GoldMineralDetectorTest extends OpMode
{
    // Detector object
    private GoldMineralDetector detector;
    private DcMotor motorLeft;
    private DcMotor motorRight;
    private DcMotor motorRaiseL;
    private DcMotor motorRaiseR;
    private DistanceSensor twoMDistance;
    private Servo hook;
    private Servo servoUp;
    private Servo servoOpen;
    private DistanceSensor sensorColorRange;
    private Gyroscope imu;
    private DigitalChannel digitalTouch;
    double tgtPower = 0;
    double tgtPowerR = 0;
    double tgtPowerLeft = 0;
    double tgtPowerRight = 0;

    @Override
    public void init() {
        telemetry.addData("Status", "DogeCV 2018.0 - Gold Align Example");

        motorLeft = hardwareMap.get(DcMotor.class, "motorLeft");
        motorRight = hardwareMap.get(DcMotor.class, "motorRight");
        twoMDistance = hardwareMap.get(DistanceSensor.class, "twoMDistance");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Set up detector
        detector = new GoldMineralDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment

        detector.enable(); // Start the detector!



    }

    /*
     * Code to run REPEATEDLY when the driver hits INIT
     */
    @Override
    public void init_loop() {
    }

    @Override
    public void start() {

            // run until the end of the match (driver presses STOP)

         }

    /*
     * Code to run REPEATEDLY when the driver hits PLAY
     */
    @Override
    public void loop() {
        telemetry.addData("X Pos" , detector.getScreenPosition().x); // Gold X position.
        telemetry.addData("Y Pos" , detector.getScreenPosition().y); // Gold Y position.
        telemetry.addData("Target Power", tgtPower);
        telemetry.addData("Target PowerR", tgtPowerR);
        telemetry.addData("Distance (cm)", twoMDistance.getDistance(DistanceUnit.CM));

        double xValue = detector.getScreenPosition().x;
        double YValue = detector.getScreenPosition().y;

        if (xValue < 150 | xValue > 300) {
            motorLeft.setPower(1); } else {

        tgtPower = -this.gamepad1.left_stick_y;
        motorLeft.setPower(tgtPower);
        motorRight.setPower(-tgtPower);

        tgtPowerLeft = -this.gamepad1.left_stick_x;
        motorLeft.setPower(tgtPowerLeft);
        tgtPowerRight = -this.gamepad1.left_stick_x;
        motorRight.setPower(tgtPowerRight);} }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // Disable the detector
        detector.disable();
    }

}