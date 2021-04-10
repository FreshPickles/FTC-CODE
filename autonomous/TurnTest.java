package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 * <p>
 * The code REQUIRES that you DO have encoders on the wheels,
 * otherwise you would use: PushbotAutoDriveByTime;
 * <p>
 * This code ALSO requires that the drive Motors have been configured such that a positive
 * power command moves them forwards, and causes the encoders to count UP.
 * <p>
 * The desired path in this example is:
 * - Drive forward for 48 inches
 * - Spin right for 12 Inches
 * - Drive Backwards for 24 inches
 * - Stop and close the claw.
 * <p>
 * The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 * that performs the actual movement.
 * This methods assumes that each movement is relative to the last stopping place.
 * There are other ways to perform encoder based moves, but this method is probably the simplest.
 * This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "TurnTest", group = "Linear OpMode")
//@Disabled
public class TurnTest extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY =
            "AeflKlH/////AAABmQAedZNXCkKmqQ2CkC55GVkZGoxK0YlVMNeDwQgN5B9Jq26R9J8TZ0qlrBQVz2o3vEgIjMfV8rZF2Z7PPxZJnScBap/Jh2cxT0teLCWkuBk/mZzWC0bRjhpwT0JkU3AGpztJHL4oJZDEaf4fUDilG1NdczNT5V8nL/ZraZzRZvGBwYO7q42b32DKKb+05OemiCOCx34h0qq0lkahDKKO7k1UTpznzyK33IPVtvutSgGvdrpNe/Jv5ApIvHcib4bKom7XVqf800+Adi0bDD94NSWFeJq+i/IZnJJqH9iXXdl3Qjptri6irrciVJtmjtyZCnFB0n4ni90VmmDb5We3Dvft6wjdPrVO5UVAotWZJAnr";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    private ElapsedTime runtime = new ElapsedTime();

    // moving motors
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    // sweeping motors
    private DcMotor wheelRack;
    private DcMotor belt;

    // launching motor
    private DcMotorEx launcher;

    //armotor
    private DcMotorEx claw;

    // servos
    private Servo wobbleArm;
    private Servo launcherPush;  // moves the rings into the launching motor

    final double launcherVelocity = 223;
    double motorPower = 0.4;  // set the vehicle DC motor power.

    // imu stuff
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;

    static final double COUNTS_PER_MOTOR_REV = 383.6;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP... maybe 1??
    static final double WHEEL_DIAMETER_INCHES = 10 / 2.54;     // For figuring circumference
    static final double COUNTS_PER_INCH = (12 / 9.6)/* (12/10.25) */ * (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = .1;
    static final double TURN_SPEED = 0.5;
    static final double ringDistance = 39; // distance from origin to the location of ring detection (inch)

    static final double AZoneDistance = 12; // distance from origin to the location of ring detection (inch)
    static final double shootToAZoneDrop = 28; // distance from origin to the location of ring detection (inch)
    static final double BZoneDistance = 34; // distance from origin to the location of ring detection (inch)
    static final double shootToBZoneDrop = 6; // distance from origin to the location of ring detection (inch)
    static final double CZoneDistance = 64; // distance from origin to the location of ring detection (inch)
    static final double shootToCZoneDrop = 32; // distance from origin to the location of ring detection (inch)
    double goForBakDistanceCorrection = 1.10;  //Calibration showed setting to go forward/backward 32 inches command, but actually achieved 29 inches when motor power was set as 0.6.
    double goStrafeDistanceCorrection = 1.34;  //Calibration showed setting to strafe left/right 32 inches command, but actually achieved 24 inches when motor power was set as 0.6.

    static final double shootRight = 12;
    static final double shootDistance = 22;  //The distance from the ring detection to shooting position X direction
    static final double shootLeft = 33;  //The distance from the ring detection to shooting position Y direction to strafe left
    static final double goalRoom = 6;
    static final double moreGoalRoom = 6;
    static final double powerShot1 = 13.5;
    static final double powerShot2 = 7;
    static final double powerShot3 = 6.5;
    static final double toShoot = 5;
    static final double backUpRing = 5;
    static final double ringToGoal = 5;

    @Override
    public void runOpMode() {
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcherPush = hardwareMap.get(Servo.class, "launcherPush");

        wheelRack = hardwareMap.get(DcMotor.class, "wheelRack");
        belt = hardwareMap.get(DcMotor.class, "belt");

        //arm is god servo
        wobbleArm = hardwareMap.get(Servo.class, "arm");
        claw = hardwareMap.get(DcMotorEx.class, "claw");

        wobbleArm.setPosition(0.65);
        telemetry.addData("position arm", wobbleArm.getPosition());
        telemetry.addData("position armfter", wobbleArm.getPosition());

        /*
        ------------------------IMU SETUP------------------------
        */

        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();

        imuParameters.mode = BNO055IMU.SensorMode.IMU;
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(imuParameters);

        // caliblrating imu
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        claw.setPower(-0.4);
        sleep(500);
        claw.setPower(0);

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();
        /*
        ----------------------------------------------------------------
        */

        /*
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setPositionPIDFCoefficients(150);
        frontRight.setPositionPIDFCoefficients(150);
        backLeft.setPositionPIDFCoefficients(150);
        backRight.setPositionPIDFCoefficients(150);
        */
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setVelocityPIDFCoefficients(150, 0, 0, 0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //put encoder stuff IN HERE; while opModeisActive is for everything
        sleep(500);

        goForwardEncoder(ringDistance*goForBakDistanceCorrection, motorPower);

        //*

        String objectsFound = "None";

        //Detecting the rings
        for (int j = 0; j < 2500000; j++) {
            if (tfod != null) { //tfod !=null is just for the camera thing
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        objectsFound = recognition.getLabel();
                    }
                    telemetry.update();
                }
            }
        }
        //start up the launcher
        launcherPush.setPosition(0.7);
        launcher.setVelocity(-launcherVelocity / 60 * 383.6);

        //go to the line
        goForwardEncoder(shootDistance*goForBakDistanceCorrection, motorPower);

        //first adjust before shoot
        rotate((int)(0 - getAngle()),0.1);
        resetAngle();

        //Starts shooting to all 3 goals
        sleep(200);
        rotate((int)(9),0.05);
        while(launcher.getVelocity()/383.6*60 > -205){
            telemetry.addData("Velocity: ", launcher.getVelocity()/383.6*60);
            telemetry.update();
        }
        launcherPush.setPosition(0.3);

        sleep(200);
        launcherPush.setPosition(0.7);
        rotate((int)(14),0.05);
        while(launcher.getVelocity()/383.6*60 > -205){
            telemetry.addData("Velocity: ", launcher.getVelocity()/383.6*60);
            telemetry.update();
        }

        launcherPush.setPosition(0.3);
        sleep(200);
        launcherPush.setPosition(0.7);
        rotate((int)(18),0.05);
        while(launcher.getVelocity()/383.6*60 > -205){
            telemetry.addData("Velocity: ", launcher.getVelocity()/383.6*60);
            telemetry.update();
        }
        launcherPush.setPosition(0.3);

        sleep(200);
        launcherPush.setPosition(0.7);
        launcher.setVelocity(0);

        telemetry.addData("Velocity: ", getAngle());
        telemetry.update();

        rotate((int)(-getAngle()),0.1);
        resetAngle();


        //goes back to the position
        // moving to the correct square based on the amount of rings
        if (objectsFound.equals("None")) {
            // go to box
            strafeRightEncoder((shootToAZoneDrop)*goStrafeDistanceCorrection, motorPower);
            //claw grabs
            goForwardEncoder(AZoneDistance*goForBakDistanceCorrection, motorPower);

            // drop wobble goal
            stopMotors();
            sleep(200);
            while (wobbleArm.getPosition() < 0.95) {
                wobbleArm.setPosition(wobbleArm.getPosition() + 0.0020);
            }
            claw.setPower(0.4);
            sleep(600);
            claw.setPower(0);

            //time to go for wobble goal
            rotate((int)(0 - getAngle()),0.1);
            resetAngle();
            goBackwardEncoder(AZoneDistance*goForBakDistanceCorrection, motorPower);
            strafeLeftEncoder(15*goForBakDistanceCorrection, motorPower);
            goBackwardEncoder(40*goStrafeDistanceCorrection, motorPower);

            //pickup
            claw.setPower(-0.4);
            sleep(600);
            claw.setPower(0);
            while (wobbleArm.getPosition() > 0.67) {
                wobbleArm.setPosition(wobbleArm.getPosition() - 0.0025);
            }
            //first adjust before shoot
            rotate((int)(0 - getAngle()),0.1);
            resetAngle();
            goForwardEncoder((AZoneDistance+40)*goStrafeDistanceCorrection, motorPower);
            strafeRightEncoder(15*goForBakDistanceCorrection, motorPower);
            while (wobbleArm.getPosition() < 0.97) {
                wobbleArm.setPosition(wobbleArm.getPosition() + 0.0020);
            }
            claw.setPower(0.4);
            sleep(600);
            claw.setPower(0);
            while (wobbleArm.getPosition() > 0.67) {
                wobbleArm.setPosition(wobbleArm.getPosition() - 0.0025);
            }
            claw.setPower(-0.4);
            sleep(600);
            claw.setPower(0);
        } else if (objectsFound.equals("Single")) {
            // go to box
            strafeRightEncoder((shootToBZoneDrop)*goStrafeDistanceCorrection, motorPower);
            //claw grabs
            goForwardEncoder(BZoneDistance*goForBakDistanceCorrection, motorPower);

            // drop wobble goal
            stopMotors();
            sleep(200);
            while (wobbleArm.getPosition() < 0.95) {
                wobbleArm.setPosition(wobbleArm.getPosition() + 0.0020);
            }
            claw.setPower(0.4);
            sleep(600);
            claw.setPower(0);

            //time to go for wobble goal
            rotate((int)0,0.1);
            resetAngle();
            goBackwardEncoder(BZoneDistance*goForBakDistanceCorrection, motorPower);
            strafeLeftEncoder(15*goForBakDistanceCorrection, motorPower);
            goBackwardEncoder(40*goStrafeDistanceCorrection, motorPower);

            //pickup
            claw.setPower(-0.4);
            sleep(600);
            claw.setPower(0);
            while (wobbleArm.getPosition() > 0.67) {
                wobbleArm.setPosition(wobbleArm.getPosition() - 0.0025);
            }
            //first adjust before shoot
            rotate((int)(0 - getAngle()),0.1);
            resetAngle();
            goForwardEncoder((BZoneDistance+40)*goStrafeDistanceCorrection, motorPower);
            strafeRightEncoder(15*goForBakDistanceCorrection, motorPower);
            while (wobbleArm.getPosition() < 0.97) {
                wobbleArm.setPosition(wobbleArm.getPosition() + 0.0020);
            }
            claw.setPower(0.4);
            sleep(600);
            claw.setPower(0);
            while (wobbleArm.getPosition() > 0.67) {
                wobbleArm.setPosition(wobbleArm.getPosition() - 0.0025);
            }
            claw.setPower(-0.4);
            sleep(600);
            claw.setPower(0);
        } else {
            // go to box
            strafeRightEncoder((shootToCZoneDrop)*goStrafeDistanceCorrection, motorPower);
            //claw grabs
            goForwardEncoder(CZoneDistance*goForBakDistanceCorrection, motorPower);

            // drop wobble goal
            stopMotors();
            sleep(200);
            while (wobbleArm.getPosition() < 0.95) {
                wobbleArm.setPosition(wobbleArm.getPosition() + 0.0020);
            }
            claw.setPower(0.4);
            sleep(600);
            claw.setPower(0);

            //time to go for wobble goal
            rotate((int)(0 - getAngle()),0.1);
            resetAngle();
            goBackwardEncoder(CZoneDistance*goForBakDistanceCorrection, motorPower);
            strafeLeftEncoder(15*goForBakDistanceCorrection, motorPower);
            goBackwardEncoder(40*goStrafeDistanceCorrection, motorPower);

            //pickup
            claw.setPower(-0.4);
            sleep(600);
            claw.setPower(0);
            while (wobbleArm.getPosition() > 0.67) {
                wobbleArm.setPosition(wobbleArm.getPosition() - 0.0025);
            }
            //first adjust before shoot
            rotate((int)(0 - getAngle()),0.1);
            resetAngle();
            goForwardEncoder((CZoneDistance+40)*goStrafeDistanceCorrection, motorPower);
            strafeRightEncoder(15*goForBakDistanceCorrection, motorPower);
            while (wobbleArm.getPosition() < 0.97) {
                wobbleArm.setPosition(wobbleArm.getPosition() + 0.0020);
            }
            claw.setPower(0.4);
            sleep(600);
            claw.setPower(0);
            while (wobbleArm.getPosition() > 0.67) {
                wobbleArm.setPosition(wobbleArm.getPosition() - 0.0025);
            }
            claw.setPower(-0.4);
            sleep(600);
            claw.setPower(0);
        }
        stopMotors();
        sleep(500);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
        //*/

        if (tfod != null) {
            tfod.shutdown();
        }
    }


    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */

    protected void encoderDrive(double speed,
                                double FrontLeftInches, double FrontRightInches, double BackLeftInches, double BackRightInches,
                                double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        //double adjustRT = 1.01; // Right distance adjustment 1% more than left
        // Speed ramp on start of move to avoid wheel slip
        final double MINSPEED = 0.30; // Start at this power
        final double SPEEDINCR = 0.2; // And increment by this much each cycle
        double curSpeed; // Keep track of speed as we ramp

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            //DUE TO ORIENTATION OF MOTORS, LEFT MOTORS HAVE TO HAVE SIGN REVERSED FOR DISTANCES. maybe.
            newFrontLeftTarget = frontLeft.getCurrentPosition() + (int) (-FrontLeftInches * COUNTS_PER_INCH);
            newFrontRightTarget = frontRight.getCurrentPosition() + (int) (FrontRightInches * COUNTS_PER_INCH);
            newBackLeftTarget = backLeft.getCurrentPosition() + (int) (-BackLeftInches * COUNTS_PER_INCH);
            newBackRightTarget = backRight.getCurrentPosition() + (int) (BackRightInches * COUNTS_PER_INCH);
            frontLeft.setTargetPosition(newFrontLeftTarget);
            frontRight.setTargetPosition(newFrontRightTarget);
            backLeft.setTargetPosition(newBackLeftTarget);
            backRight.setTargetPosition(newBackRightTarget);

            while (frontLeft.getTargetPosition() != newFrontLeftTarget) {
                frontLeft.setTargetPosition(newFrontLeftTarget);
                sleep(1);
            }
            while (frontRight.getTargetPosition() != newFrontRightTarget) {
                frontRight.setTargetPosition(newFrontRightTarget);
                sleep(1);
            }
            while (backLeft.getTargetPosition() != newBackLeftTarget) {
                backLeft.setTargetPosition(newBackLeftTarget);
                sleep(1);
            }
            while (backRight.getTargetPosition() != newBackRightTarget) {
                backRight.setTargetPosition(newBackRightTarget);
                sleep(1);
            }


            // Turn On RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            curSpeed = Math.abs(speed); // Make sure its positive
            //curSpeed = Math.min(MINSPEED, speed);


            frontLeft.setPower(curSpeed);
            backLeft.setPower(curSpeed);
            frontRight.setPower(curSpeed);
            backRight.setPower(curSpeed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy()&& backRight.isBusy())) { //might need to change this to all being busy..
/*
                    // Ramp up motor powers as needed
                    if (Math.abs(frontLeft.getCurrentPosition()- newFrontLeftTarget) < (0.3*Math.abs(newFrontLeftTarget)))
                    {
                        telemetry.addData("test", "qao hi");
                        if (curSpeed > MINSPEED) {
                            curSpeed -= SPEEDINCR;
                        }
                        else curSpeed = MINSPEED;

                    }
                    else {
                        // Ramp up motor powers as needed
                        if (curSpeed < speed) {
                            curSpeed += SPEEDINCR;
                        } else curSpeed = speed;
                    }

 */
                // And rewrite the motor speeds
                frontLeft.setPower(curSpeed);
                backLeft.setPower(curSpeed);
                frontRight.setPower(curSpeed);
                backRight.setPower(curSpeed);


                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d : %7d : %7d", newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d : %7d : %7d",
                        frontLeft.getCurrentPosition(),
                        frontRight.getCurrentPosition(), backLeft.getCurrentPosition(), backRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(100);   // optional pause after each move
        }
    }

    protected void goBackward(double tgtPower) {
        frontRight.setPower(-tgtPower);
        frontLeft.setPower(tgtPower);
        backRight.setPower(-tgtPower);
        backLeft.setPower(tgtPower);
    }

    protected void goForward(double tgtPower) {
        frontRight.setPower(tgtPower);
        frontLeft.setPower(-tgtPower);
        backRight.setPower(tgtPower);
        backLeft.setPower(-tgtPower);
    }

    protected void stopMotors() {
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }

    private void strafeRight(double tgtPower) {
        frontRight.setPower(tgtPower);
        frontLeft.setPower(tgtPower);
        backRight.setPower(-tgtPower);
        backLeft.setPower(-tgtPower);
    }

    private void strafeLeft(double tgtPower) {
        frontRight.setPower(-tgtPower);
        frontLeft.setPower(-tgtPower);
        backRight.setPower(tgtPower);
        backLeft.setPower(tgtPower);
    }


    protected void goBackwardEncoder(double goBakInches, double mymotorPower) {
        //frontRight.setPower(tgtPower);
        //frontLeft.setPower(-tgtPower);
        //backRight.setPower(tgtPower);
        //backLeft.setPower(-tgtPower);
        encoderDrive(mymotorPower, -goBakInches, -goBakInches, -goBakInches, -goBakInches, 30);

    }

    protected void goForwardEncoder(double goForInches, double mymotorPower) {
        //frontRight.setPower(-tgtPower);
        //frontLeft.setPower(tgtPower);
        //backRight.setPower(-tgtPower);
        //backLeft.setPower(tgtPower);
        encoderDrive(mymotorPower, goForInches, goForInches, goForInches, goForInches, 30);

    }

    private void strafeRightEncoder(double strafeRightInch, double mymotorPower) {
        //frontRight.setPower(tgtPower);
        //frontLeft.setPower(tgtPower);
        //backRight.setPower(-tgtPower);
        //backLeft.setPower(-tgtPower);
        encoderDrive(mymotorPower, strafeRightInch, -strafeRightInch, -strafeRightInch, strafeRightInch, 30);

    }

    private void strafeLeftEncoder(double strafeLeftInch, double mymotorPower) {
        //frontRight.setPower(-tgtPower);
        //frontLeft.setPower(-tgtPower);
        //backRight.setPower(tgtPower);
        //backLeft.setPower(tgtPower);
        encoderDrive(mymotorPower, -strafeLeftInch, strafeLeftInch, strafeLeftInch, -strafeLeftInch, 30);
    }

    /*
       ----------------------------imu methods----------------------------
        */
    /*
     * Resets the cumulative angle tracking to zero.
     */
    protected void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /*
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    protected double getAngle() {

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

    /*
     * See if we are moving in a straight line and if not return a power correction value.
     *
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    protected double checkDirection() {
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
    /*
    -----------------------------------------------------------
     */

    protected void rotate(int degrees, double Power) {
        stopMotors();
        double tgtPower = Power;
        // getAngle() returns + when rotating counter clockwise (left) and - when rotating clockwise (right).

        if (degrees > 0) {   // turn left
            frontLeft.setPower(tgtPower);
            backLeft.setPower(tgtPower);
            frontRight.setPower(tgtPower);
            backRight.setPower(tgtPower);
            while (getAngle() < degrees) {
                sleep(1);
            }
        } else if (degrees < 0) {   // turn right
            frontLeft.setPower(-tgtPower);
            backLeft.setPower(-tgtPower);
            frontRight.setPower(-tgtPower);
            backRight.setPower(-tgtPower);
            while (getAngle() > degrees) {
                sleep(1);
            }

        } else return;

        stopMotors();
        sleep(100);
        return;
    }

}
