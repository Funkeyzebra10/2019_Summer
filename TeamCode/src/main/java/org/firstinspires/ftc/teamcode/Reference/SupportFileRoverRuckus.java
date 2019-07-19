/* Copyright (c) 2018 FIRST. All rights reserved.
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
 *//*


package org.firstinspires.ftc.teamcode.Reference;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.Dogeforia;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

//@Autonomous(name = "Nudgeee")

*/
/*public class SupportFileRoverRuckus extends OpMode {
    int step = 0;
    DcMotor fLeft, fRight, bLeft, bRight;

    //Gyro define
    BNO055IMU imu;

    //Gyro telemetry
    Orientation angles;
    Acceleration gravity;

    String tData ="";



    SamplingOrderDetector detector;

    double cvtDegreesR(double degrees) {
        double retVal;
        //makes the negative degrees from the raw gyro yaw into the 180 to 360 degrees
        if (degrees < 0) {
            retVal = 180 + (180 + degrees);
        } else {
            //return the raw gyro as itself since its already 0 to 180
            retVal = degrees;
        }
        return retVal;
    }//end of cvtDegreesR

    *//*


    */
/**
     * this method converts the original gyro input(-180 to 180), to a
     * 0 to 360 form. This method is used for turning left(counter-clockwise)
     * @param degrees the raw degrees that the gyro returns
     * @return turn the inputted -180 to 180 degrees to 0- to 60 degree format.
     *//*

    */
/*
    double cvtDegreesL(double degrees) {
        double retVal;
        //makes the negative degrees from the raw gyro into positive degrees(0 to 180)
        if (degrees <= 0) {
            retVal = Math.abs(degrees);
        } else {
            //makes the positive degrees from the raw gyro into 180 to 360 degrees
            retVal = 180 + (180-degrees);
        }
        return retVal;
    }//end of cvtDegreesL

*//*


    */
/*
    private static final String VUFORIA_KEY = "AbbjdQb/////AAABmRRAhmtd30mMq0AwAw9r0DZliL1sYh3xSveL4qZ+dFMO5DZUoOAbWApkduegT7NPvv++REpW3zljlQ1CPubxwaPaSbdGZBLffMxO8nsiledzIVnslT5Uf5Vp+bEvbJQ/3hUcAHRv1VS58XGOQkl8ryHgEJx3hwbeY46V6PeuV/Q7VK8749ipxUjnvnd1dsaYsc15xGzGBaeg/RjRS+89BUNRFHr6gxy6cj4KFj/qpxtUiWHxAa7XqENogqyGEEB46u2bjk9x27LMH+yC+PHOpVpEVrxnQC0N8ZQJrhYJfNrH+qqE4OM9jtd9nFP6yKMbjPFckp/hxDsj+CBKeIcGHtB0kB892U0q1KXsTsAwf1GB";

    private static final float mmPerInch = 25.4f;
    private static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    private static final Dogeforia.CameraDirection CAMERA_CHOICE = BACK;

    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;


    static final double     COUNTS_PER_MOTOR_REV    = 28.0 ;    // eg: AndyMark NeverRest40 Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 20.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

            *//*



    */
/**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     *//*


    */
/*
     Dogeforia vuforia;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    @Override
    public void init() {

        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");

        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        //Gyro parameters
        BNO055IMU.Parameters gParameters = new BNO055IMU.Parameters();
        gParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gParameters.calibrationDataFile = "BNO055IMUCalibration.json"; //See the calibration sample opmode
        gParameters.loggingEnabled = true;
        gParameters.loggingTag = "IMU";
        gParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        //Gyro initialize
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(gParameters);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        Dogeforia.Parameters parameters = new Dogeforia.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = new Dogeforia(parameters);

        // Load the data sets that for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection

        allTrackables.addAll(targetsRoverRuckus);

        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        *//*


        */
/**
         * To place the RedFootprint target in the middle of the red perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 180 around the field's Z axis so the image is flat against the red perimeter wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative Y axis to the red perimeter wall.
         *//*


        */
/*
        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        /**
         * To place the FrontCraters target in the middle of the front perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 90 around the field's Z axis so the image is flat against the front wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative X axis to the front perimeter wall.
         *//*


        */
/*
        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        /**
         * To place the BackSpace target in the middle of the back perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it -90 around the field's Z axis so the image is flat against the back wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the X axis to the back perimeter wall.
         *//*

        */
/*
        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        /**
         * Create a transformation matrix describing where the phone is on the robot.
         *
         * The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         *
         * The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
         * pointing to the LEFT side of the Robot.  It's very important when you test this code that the top of the
         * camera is pointing to the left side of the  robot.  The rotation angles don't work if you flip the phone.
         *
         * If using the rear (High Res) camera:
         * We need to rotate the camera around it's long axis to bring the rear camera forward.
         * This requires a negative 90 degree rotation on the Y axis
         *
         * If using the Front (Low Res) camera
         * We need to rotate the camera around it's long axis to bring the FRONT camera forward.
         * This requires a Positive 90 degree rotation on the Y axis
         *
         * Next, translate the camera lens to where it is on the robot.
         * In this example, it is centered (left to right), but 110 mm forward of the middle of the robot, and 200 mm above ground level.
         *//*



        */
/*
        final int CAMERA_FORWARD_DISPLACEMENT = 110;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

        /**  Let all the trackable listeners know where the phone is.  *//*


        */
/*
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        /** Wait for the game to begin *//*




        */
/*(
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();


        /** Start tracking the data sets we care about. *//*


        */
/*
        targetsRoverRuckus.activate();

        detector = new SamplingOrderDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        detector.downscale = 0.4; // How much to downscale the input frames

        // Optional Tuning
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.001;

        detector.ratioScorer.weight = 15;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();

        detector.init(hardwareMap.appContext,CameraViewDisplay.getInstance(), 0, true); // This disables the built in camera and view

        vuforia.setDogeCVDetector(detector);
        vuforia.enableDogeCV();
        vuforia.showDebug();
        vuforia.start();
    }

    public void loop() {

        DogeCVNudge();



    }

    public void stop() {
        fLeft.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);

        detector.disable();

        vuforia.stop();


    }


    public void DogeCVNudge(){
        String order = detector.getLastOrder().toString();
        if(order == "RIGHT"){
            telemetry.addLine("right :)");
        }
        else if(order == "LEFT"){
            telemetry.addLine("left :)");
        }
        else if(order == "CENTER"){
            telemetry.addLine("center :)");
        }
        else{
            telemetry.addLine("no nudge");
        }
    }

    public void GyroTest(double targetAngle, double speed) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        double currentAngle = angles.thirdAngle;
        telemetry.addData("thirdAngle: ", angles.thirdAngle);
        telemetry.update();


        while (Math.abs(currentAngle - targetAngle) > 40) {
            currentAngle = angles.thirdAngle;
            telemetry.update();
            if (currentAngle > targetAngle) {//turn left
                telemetry.update();
                telemetry.update();
                fLeft.setPower(-1 * speed);
                fRight.setPower(speed);
                bLeft.setPower(-1 * speed);
                bRight.setPower(speed);

                telemetry.update();

            }
            else if (currentAngle < targetAngle) {

                telemetry.update();
                fLeft.setPower(speed);
                fRight.setPower(-1 * speed);
                bLeft.setPower(speed);
                bRight.setPower(-1 * speed);

                telemetry.update();
            }
            else {
                stop();
                telemetry.update();
            }
            telemetry.update();
            telemetry.addData("angle: ", angles.firstAngle);
        }
        stop();
        telemetry.update();

    }



    public void gyroFull(double targetAngle){

        //uses the right convert degrees method to convert degrees to 0 to 360
        double currentAngle = cvtDegreesR(angles.firstAngle);
        //the speed of the robot
        double speed;
        //minimum speed of the robot
        double min = 0.125;
        //direction the robot is turning
        int direction;

        //used if the intended yaw equals or is less than 180
        if(targetAngle <= 180) {
            while (currentAngle < targetAngle - 1 || currentAngle > targetAngle + 1) {
                //uses the right convert degrees method to convert degrees to 0 to 360 clockwise
                currentAngle = cvtDegreesR(angles.firstAngle);

                //sets the direction the robot turns. Used to go opposite direction if the robot overshoots its target
                if (targetAngle - currentAngle > 0) {
                    direction = 1;
                } else {
                    direction = -1;
                }

                //sets the speed of the robot turning. this algorithm will slow down the robot as it approaches its intended angle
                speed = (1 - (currentAngle / targetAngle)) * ((targetAngle - currentAngle) * 0.01);

                //if the speed gets under the min speed it will use the min speed
                if (Math.abs(speed) < min && Math.abs(speed) != 0) {
                    speed = min;
                }

                telemetry.addData("Yaw: ", currentAngle);
                telemetry.update();

                //sets power to motors
                fLeft.setPower(-speed * direction);
                bLeft.setPower(-speed * direction);
                fRight.setPower(speed * direction);
                bRight.setPower(speed * direction);
            }//end of while
        }//end of less than 180
        //used if the intended yaw is greater than 180
        else
        {
            //makes the target angle from greater to 180 to less than 180 so it can be used with the degrees converting method
            targetAngle = Math.abs(targetAngle - 360);
            //uses the left convert degrees method to convert degrees to 0 to 360 counter-clockwise
            currentAngle = cvtDegreesL(angles.firstAngle);
            while (currentAngle < targetAngle - 1 || currentAngle > targetAngle + 1) {
                currentAngle = cvtDegreesL(angles.firstAngle);

                //sets the direction the robot turns. Used to go opposite direction if the robot overshoots its target
                if (targetAngle - currentAngle > 0) {
                    direction = 1;
                } else {
                    direction = -1;
                }

                //sets the speed of the robot turning. this algorithm will slow down the robot as it approaches its intended angle
                speed = (1 - (currentAngle / targetAngle)) * ((targetAngle - currentAngle) * 0.01);

                //if the speed gets under the min speed it will use the min speed
                if (Math.abs(speed) < min && Math.abs(speed) != 0) {
                    speed = min;
                }

                telemetry.addData("Target: ",targetAngle);
                telemetry.addData("Yaw: ", currentAngle);
                telemetry.update();

                //sets power to motors
                fLeft.setPower(speed * direction);
                bLeft.setPower(speed * direction);
                fRight.setPower(-speed * direction);
                bRight.setPower(-speed * direction);
            }//end of while
        }//end of over 180 else

        //sets robot to rest
        fLeft.setPower(0);
        bLeft.setPower(0);
        fRight.setPower(0);
        bRight.setPower(0);
    }//end of gyro full test

    //nthroot for gyroFullExp only. Calculates "n"th root of "A" at "p" precision. From Rosetta Code.
    public static double nthroot(int n, double A, double p) {
        if(A < 0) {
            return -1;
        } else if(A == 0) {
            return 0;
        }
        double x_prev = A;
        double x = A / n;  // starting "guessed" value...
        while(Math.abs(x - x_prev) > p) {
            x_prev = x;
            x = ((n - 1.0) * x + A / Math.pow(x, n - 1.0)) / n;
        }
        return x;
    }

    public void vuforiaDistance() {
        // check all the trackable target to see which one (if any) is visible.
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }
        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            double tX = translation.get(0) / mmPerInch;
            double tY = translation.get(1) / mmPerInch;
            double tZ = translation.get(2) / mmPerInch;

            if (tY > -5.5) {//-45
                telemetry.addLine("move forward");
                fLeft.setPower(0.15);
                fRight.setPower(0.15);
                bLeft.setPower(0.15);
                bRight.setPower(0.15);
            } else if (tY < -6.0) {//-55
                telemetry.addLine("move backward");
                fLeft.setPower(-0.15);
                fRight.setPower(-0.15);
                bLeft.setPower(-0.15);
                bRight.setPower(-0.15);
            } else {
                stop();
            }

            telemetry.addData("tY:", tY);
            telemetry.addData("tX:", tX);

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("ot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        } else {
            telemetry.addData("Visible Target", "none");
        }
        telemetry.update();
    }

    public void vuforiaTurn() {
        // check all the trackable target to see which one (if any) is visible.
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }
        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            double tX = translation.get(0) / mmPerInch;
            double tY = translation.get(1) / mmPerInch;
            double tZ = translation.get(2) / mmPerInch;

            telemetry.addData("tZ:", tZ);

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);

            double rX = rotation.firstAngle;
            double rY = rotation.secondAngle;
            double rZ = rotation.thirdAngle;

            if (rZ > -160) {//-85
                telemetry.addLine("turn right");
                bLeft.setPower(0.15);
                fLeft.setPower(0.15);
                bRight.setPower(-0.15);
                fRight.setPower(-0.15);
            } else if (rZ < -165) {//-95
                telemetry.addLine("turn left");
                fLeft.setPower(-.15);
                bLeft.setPower(-0.15);
                fRight.setPower(0.15);
                bRight.setPower(0.15);
            } else {
                stop();
            }

        } else {
            telemetry.addData("Visible Target", "none");
        }
        telemetry.update();
    }

    public void driveStraight(String direction, int distance){
        resetEncoder();
        int mDirection = -1;
        if (direction.equals("backward")){
            mDirection = 1;
        }
        bRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set position
        fRight.setTargetPosition((int) COUNTS_PER_INCH * distance * mDirection);
        fLeft.setTargetPosition((int) COUNTS_PER_INCH * distance * mDirection);
        bRight.setTargetPosition((int) COUNTS_PER_INCH * distance * mDirection);
        bLeft.setTargetPosition((int) COUNTS_PER_INCH * distance * mDirection);
        //set power
        fLeft.setPower(0.2);
        fRight.setPower(0.2);
        bLeft.setPower(0.2);
        bRight.setPower(0.2);

        //while busy
        while (fLeft.isBusy()){telemetry.update();}

        fLeft.setPower(0);
        fRight.setPower(0);
        resetEncoder();

    }

    public void resetEncoder(){
        bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }


}
*//*




*/
