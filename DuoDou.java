/* Copyright (c) 2019 FIRST. All rights reserved.
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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Mat;

import java.util.Iterator;
import java.util.List;

import detectors.FoundationPipeline.Pipeline;
import detectors.FoundationPipeline.SkyStone;
import detectors.OpenCvDetector;

@Autonomous(name = "The Three <<Holy Systems>>", group = "Primordial Artifact")
public class DuoDou extends LinearOpMode {
    // my robot parts
    // motors
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;
    private DcMotor sweepMotorLeft;
    private DcMotor sweepMotorRight;
    private DcMotor lift;

    // servos
    private Servo arm;
    private Servo claw;
    private Servo sweepLeft;
    private Servo sweepRight;

    // imu stuff
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;

    public void runOpMode() throws InterruptedException {

        /*
        ----------------- hardware setup ----------------------
         */
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");

        sweepMotorLeft = hardwareMap.get(DcMotor.class, "sweepMotorLeft");
        sweepMotorRight = hardwareMap.get(DcMotor.class, "sweepMotorRight");
        sweepLeft = hardwareMap.get(Servo.class, "sweepLeft");
        sweepRight = hardwareMap.get(Servo.class, "sweepRight");

        lift = hardwareMap.get(DcMotor.class, "lift");
        arm = hardwareMap.get(Servo.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");

        telemetry.setAutoClear(true);


        telemetry.addData("Booting Up", " . . .");
        telemetry.update();
        /*
        ---------------------------------------------------------
         */

        OpenCvDetector fieldDetector = new OpenCvDetector(this);
        //ImageDetector detector = new ImageDetector(this, false);
        //StoneDetector stone = new StoneDetector(this, false);

        //stone.start();
        //detector.start();
        fieldDetector.start();

        /*
        ------------------------IMU SETUP------------------------
         */
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        // caliblrating imu
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();
        /*
        ----------------------------------------------------
         */

        // block deetection

        while (!isStopRequested()) {
            //detector.printposition(detector.getPosition());
            //fieldDetector.print(fieldDetector.getObjectsFoundations());
            //Log.d("GO TO MO","go");
            SkyStone[] skyStone = fieldDetector.getObjectsSkyStones();

            telemetry.addData("Skystones Found", skyStone.length);
            int p = 0;
            for (SkyStone s : skyStone) {
                p++;
                telemetry.addLine("" + s);
            }

            Mat m = new Mat();
            m = null;

            //also, enabling/disabling detection for individual field elements:
                /*
                Pipeline.doFoundations=false;
                Pipeline.doStones=false;
                Pipeline.doSkyStones=false;
                */

            // getting positions of each element. Will return Objects for each element that all contain
            // Phone camera <x,y> coordinates for each element's center.

            // List<Foundation> foundations = Pipeline.foundations;
            //List<Stone> stones = Pipeline.stones;
            List<SkyStone> skystones = Pipeline.skyStones;

            for (int j = 0; j < skystones.size(); j++) {
                telemetry.addData("Skystone center", "(" + skystones.get(j).x + ", " + skystones.get(j).y + ")");
            }

                // finding stone closesst to camera
                SkyStone closestStone = null;
                if(skystones.size() > 1) {
                    for(int j = 0; j < skystones.size(); j++) {
                        double closest;
                        closest = Math.abs(skystones.get(0).x - 300) + Math.abs(skystones.get(0).y - 300);
                        for(int k = 0; k < skystones.size(); k++) {
                            if(closest > (Math.abs(skystones.get(k).x - 300) + Math.abs(skystones.get(k).y - 300)));
                                closestStone = skystones.get(k);
                        }
                    }
                }
                else {
                    closestStone = skystones.get(0);
                }

                telemetry.addData("==========", "Loop delimiter");
                telemetry.update();

                while(closestStone.x > 310 || closestStone.x < 290) {
                    goBackward(0.1);
                }

            rotate(-90, 0.1);
            collect(0.75);
            stopMotors();
            goBackward(0.75);

        }

        // wait for start command.
        waitForStart();

        // Disable Tracking when we are done
        //detector.stop();
        //stone.stop();
        fieldDetector.stop();
        //imu.stop();
    }

    /*
    ----------------------------imu methods----------------------------
     */
    /*
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /*
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     *
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /* -----------------------------------------------------
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     * ------------------------------------------------------
     */
    private void rotate(int degrees, double power) {
        double leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0) {   // turn right.
            leftPower = power;
            rightPower = -power;
        } else if (degrees > 0) {   // turn left.
            leftPower = -power;
            rightPower = power;
        } else return;

        // set power to rotate.
        frontLeft.setPower(leftPower);
        frontRight.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
            }

            while (opModeIsActive() && getAngle() > degrees) {
            }
        } else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {
            }

        // turn the motors off.
        frontLeft.setPower(leftPower);
        frontRight.setPower(rightPower);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }
    /*
    --------------------------------------------------------
     */

    /*
    ----------------------- movement methods ---------------
     */

    public void collect(double tgtPower) {
        sweepMotorLeft.setPower(0.75);
        sweepMotorRight.setPower(0.75);
        goForward(0.3);
        sweepLeft.setPosition(0);
        sweepRight.setPosition(1);
    }

    public void goForward(double tgtPower) {
        frontRight.setPower(-tgtPower);
        frontLeft.setPower(tgtPower);
        backRight.setPower(-tgtPower);
        backLeft.setPower(tgtPower);
    }

    public void goBackward(double tgtPower) {
        frontRight.setPower(tgtPower);
        frontLeft.setPower(-tgtPower);
        backRight.setPower(tgtPower);
        backLeft.setPower(-tgtPower);
    }


    public void stopMotors() {
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
        sweepMotorLeft.setPower(0);
        sweepMotorRight.setPower(0);
    }


    public void strafeRight(double tgtPower) {
        frontRight.setPower(tgtPower);
        frontLeft.setPower(tgtPower);
        backRight.setPower(-tgtPower);
        backLeft.setPower(-tgtPower);
    }

    public void strafeLeft(double tgtPower) {
        frontRight.setPower(-tgtPower);
        frontLeft.setPower(-tgtPower);
        backRight.setPower(tgtPower);
        backLeft.setPower(tgtPower);
    }

    public void turnRight(double angle) {

    }

    public void turnLeft(double tgtPower) {
        frontLeft.setPower(-tgtPower);
        backRight.setPower(-tgtPower);
    }

    /*
    --------------------------------------------------
     */

    public void listhardware() {
        telemetry.setAutoClear(false);

        Iterator<com.qualcomm.robotcore.hardware.HardwareDevice> t = hardwareMap.iterator();
        while (t.hasNext()) {

            telemetry.addData("device found", (t.next().getDeviceName()));
            telemetry.update();
        }

        telemetry.setAutoClear(true);
    }
}