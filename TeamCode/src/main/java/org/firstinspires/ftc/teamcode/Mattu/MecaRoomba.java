package org.firstinspires.ftc.teamcode.Mattu;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import java.util.Arrays;

@Autonomous(name = "MecaRoomba")
public class MecaRoomba extends OpMode {
    //Motors and sensors
    DcMotor fLeft, fRight, bLeft, bRight;
    DistanceSensor distanceL, distanceR, distanceC;
    BNO055IMU imu;

    //left = left distance sensor return value
    //right = right distance sensor return value
    //mid = center distance sensor return value
    //distance = value to determine if sensors detect obstacle
    double left, right, mid;
    double distance;

    // State used for updating telemetry
    //sensorState = array to hold boolean values of whether or not each sensor detects an object {left, mid, right}
    //movingState = object to hold direction of motion intended for the switch statement in loop
    boolean[] sensorState = {false, false, false};

    //MovingState is an enum for the states of movingState
    enum MovingState {
        M_START, M_NULL, M_BACK_LEFT, M_BACK_RIGHT, M_FORWARD, M_UP_RIGHT, M_UP_LEFT;
    }
    MovingState movingState = MovingState.M_FORWARD;

    int hit;

    //IMU variables
    //gravity holds acceleration values in x, y, and z
    //angular holds angular velocity values in x, y and z //lastAngularz holds previous z axis angular velocity for collision detection
    //speedX and speedY hold speed values in the x and y axis, gotten from accelerometer and dTime (delta time)
    Acceleration gravity;
    AngularVelocity angular;
    double lastAngularZ;
    double speedX, speedY, dTime;

    //Amount of loops the code has run through in loop
    int loops;

    public void init() {
        //Hardware mapping of the four motors
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");

        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bRight.setDirection(DcMotor.Direction.FORWARD);
        bLeft.setDirection(DcMotor.Direction.REVERSE);

        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set up imu parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        //Hardware map the distance sensors
        distanceL = hardwareMap.get(DistanceSensor.class, "distanceL");
        distanceR = hardwareMap.get(DistanceSensor.class, "distanceR");
        distanceC = hardwareMap.get(DistanceSensor.class, "distanceC");

        //Setup variables so errors aren't thrown
        distance = 31;
        lastAngularZ = 0;
        dTime = 0;
        speedX = 0;
        speedY = 0;
        loops = 0;
        hit = 0;

        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void loop() {
        //Acceleration and angular velocity variables, respectably
        gravity = imu.getAcceleration();
        angular = imu.getAngularVelocity();

        //Set variables for distance values
        left = distanceL.getDistance(DistanceUnit.CM);
        mid = distanceC.getDistance(DistanceUnit.CM);
        right = distanceR.getDistance(DistanceUnit.CM);

        //Setup sensorState with boolean values
        sensorState[0] = left <= distance;
        sensorState[1] = mid <= distance;
        sensorState[2] = right <= distance;

        //Get speed values from acceleration values
        speedX += gravity.xAccel * (System.currentTimeMillis() - dTime) / 1000;
        speedY += gravity.yAccel * (System.currentTimeMillis() - dTime) / 1000;
        dTime = System.currentTimeMillis();

        //Telemetry of states
        telemetry.addData("Moving State: ", movingState);
        telemetry.addData("Sensor State: ", Arrays.toString(sensorState));
        telemetry.addData("Amount of turning hits: ", hit);

        //Telemetry of distances
        telemetry.addData("Left Distance (cm): ", left);
        telemetry.addData("Center Distance (cm): ", mid);
        telemetry.addData("Right Distance (cm): ", right);

        //Telemetry of speed, acceleration, and angular velocity
        telemetry.addData("X Speed (m/s): ", speedX);
        telemetry.addData("Y Speed (m/s): ", speedY);
        telemetry.addData("X Acceleration (m/s/s): ", gravity.xAccel);
        telemetry.addData("Y Acceleration (m/s/s): ", gravity.yAccel);
        telemetry.addData("Z Acceleration (m/s/s): ", imu.getGravity().zAccel);
        telemetry.addData("Angular Velocity", angular.zRotationRate);

        //Logic for moving
        if (sensorState[1]) {       //Check middle sensor first as it is most important
            if (sensorState[2] && !sensorState[0]) {   //If only right and middle sensors fire
                movingState = MovingState.M_BACK_LEFT;
            }
            else {                  //Whether left sensor is fired or not, code will be the same, as long as right sensor is not fired
                movingState = MovingState.M_BACK_RIGHT;
            }
        }
        else if (sensorState[0]) {  //Check left sensor before right because right turn should be default turn direction
            movingState = MovingState.M_BACK_RIGHT;
        }
        else if (sensorState[2]) {  //Finally check right sensor when nothing else as left turning should be a last case scenario
            movingState = MovingState.M_BACK_LEFT;
        }
        else {                      //When no sensors fire, drive straight
            movingState = MovingState.M_FORWARD;
        }

        switch (movingState) {
            case M_START:
                movingState = MovingState.M_FORWARD;
                break;
            //Move forward
            case M_FORWARD:
                fLeft.setPower(0.2);
                fRight.setPower(0.2);
                bLeft.setPower(0.2);
                bRight.setPower(0.2);
                break;
            //Turn right and move back
            case M_BACK_RIGHT:
                if (loops % 3 == 0) {
                    //Collision detection
                    if (angular.zRotationRate < lastAngularZ - 1) {
                        hit++;
                        /*movingState = MovingState.M_UP_RIGHT;
                        break;*/
                    } else {
                        fLeft.setPower(0.2);
                        fRight.setPower(-0.4);
                        bLeft.setPower(0.2);
                        bRight.setPower(-0.4);
                    }
                    lastAngularZ = angular.zRotationRate;
                }
                break;
            //Turn left and move back
            case M_BACK_LEFT:
                if (loops % 3 == 0) {
                    //Collision detection
                    if (angular.zRotationRate > lastAngularZ + 1) {
                        hit++;
                        /*movingState = MovingState.M_UP_LEFT;
                        break;*/
                    } else {
                        fLeft.setPower(-0.4);
                        fRight.setPower(0.2);
                        bLeft.setPower(-0.4);
                        bRight.setPower(0.2);
                    }
                    lastAngularZ = angular.zRotationRate;
                }
                break;
            //Turn right and move up
            case M_UP_RIGHT:
                if (loops % 3 == 0) {
                    //Collision detection
                    if (angular.zRotationRate < lastAngularZ - 1) {
                        hit++;
                        /*movingState = MovingState.M_BACK_RIGHT;
                        break;*/
                    } else {
                        fLeft.setPower(0.4);
                        fRight.setPower(-0.2);
                        bLeft.setPower(0.4);
                        bRight.setPower(-0.2);
                    }
                    lastAngularZ = angular.zRotationRate;
                }
                break;
            //Turn left and move up
            case M_UP_LEFT:
                if (loops % 3 == 0) {
                    //Collision detection
                    if (angular.zRotationRate < lastAngularZ - 1) {
                        hit++;
                        /*movingState = MovingState.M_BACK_LEFT;
                        break;*/
                    } else {
                        fLeft.setPower(-0.2);
                        fRight.setPower(0.4);
                        bLeft.setPower(-0.2);
                        bRight.setPower(0.4);
                    }
                    lastAngularZ = angular.zRotationRate;
                }
                break;
            //If nothing else, stop
            case M_NULL:
                stop();
        }

        loops++;
    }

    public void stop() {
        fLeft.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);
    }
}
