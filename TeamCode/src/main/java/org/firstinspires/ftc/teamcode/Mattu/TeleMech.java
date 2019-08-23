package org.firstinspires.ftc.teamcode.Mattu;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@TeleOp(name = "TeleMech")
public class TeleMech extends OpMode {
    DcMotor fLeft, fRight, bLeft, bRight;
    DistanceSensor distanceL, distanceR, distanceC;
    BNO055IMU imu;

    //Imu variables
    Acceleration gravity;
    AngularVelocity angular;

    double leftX, leftY, rightX, rightY;

    double leftTrigger, rightTrigger;

    double distance = 31;

    String accelState;

    boolean stopped;
    double startTime, sysTime, dt;

    //Calculate the power needed to move in a specific direction.
    double fL() {
        return (leftY - leftX) / 2;
    }

    double fR() {
        return (rightY + rightX) / 2;
    }

    double bL() {
        return (leftY + leftX) / 2;
    }

    double bR() {
        return (rightY - rightX) / 2;
    }

    public void init() {
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");

        fLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        fRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotor.Direction.REVERSE);
        bLeft.setDirection(DcMotor.Direction.FORWARD);

        distanceL = hardwareMap.get(DistanceSensor.class, "distanceL");
        distanceR = hardwareMap.get(DistanceSensor.class, "distanceR");
        distanceC = hardwareMap.get(DistanceSensor.class, "distanceC");

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

        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Hardware map the distance sensors
        distanceL = hardwareMap.get(DistanceSensor.class, "distanceL");
        distanceR = hardwareMap.get(DistanceSensor.class, "distanceR");
        distanceC = hardwareMap.get(DistanceSensor.class, "distanceC");
    }

    public void loop() {
        sysTime = System.currentTimeMillis();

        gravity = imu.getAcceleration();
        angular = imu.getAngularVelocity();

        //Get inputs from left and right sticks and left and right sticks.
        /*leftX = gamepad1.left_stick_x;
        leftY = gamepad1.left_stick_y;
        rightX = gamepad1.right_stick_x;
        rightY = gamepad1.right_stick_y;
        leftTrigger = gamepad1.left_trigger;
        rightTrigger = gamepad1.right_trigger;*/

        //Logic for accelerometer data
        if (gravity.xAccel >= 1) {
            accelState = "Right";
        }
        else if (gravity.xAccel <= -1) {
            accelState = "Left";
        }
        else {
            accelState = "";
        }

        //Telemetry of distances
        telemetry.addData("Left Distance", distanceL.getDistance(DistanceUnit.CM));
        telemetry.addData("Right Distance", distanceR.getDistance(DistanceUnit.CM));
        telemetry.addData("Center Distance", distanceC.getDistance(DistanceUnit.CM));

        //Telemetry of speed, acceleration, and angular velocity
        telemetry.addData("Acceleration: ", accelState);
        telemetry.addData("X Acceleration (m/s/s): ", gravity.xAccel);
        telemetry.addData("Y Acceleration (m/s/s): ", gravity.yAccel);
        telemetry.addData("Angular Velocity", angular.zRotationRate);

        /*if (leftTrigger > 0.1) {
            if (distanceL.getDistance(DistanceUnit.CM) >= distance) {
                fLeft.setPower(0.5);
                fRight.setPower(-0.5);
                bLeft.setPower(-0.5);
                bRight.setPower(0.5);
            }
        }

        if (rightTrigger > 0.1) {
            if (distanceR.getDistance(DistanceUnit.CM) >= distance) {
                fLeft.setPower(-0.5);
                fRight.setPower(0.5);
                bLeft.setPower(0.5);
                bRight.setPower(-0.5);
            }
        }

        setPower();*/

        //Tests for accelerometer
        if ((accelState.equals("Right") || accelState.equals("Left")) && !stopped) {
            stopped = true;
            startTime = System.currentTimeMillis();
            fLeft.setPower(0);
            fRight.setPower(0);
            bLeft.setPower(0);
            bRight.setPower(0);
        }
        else if (!stopped){
            fLeft.setPower(0.3);
            fRight.setPower(0.3);
            bLeft.setPower(0.3);
            bRight.setPower(0.3);
        }

        //A timer for when hit
        if(stopped) {
            if(sysTime - startTime >= 5000) {
                stopped = false;
            }
        }

    }

    public void stop () {
        fLeft.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);
    }

    public void setPower() {
        fLeft.setPower(fL());
        fRight.setPower(fR());
        bLeft.setPower(bL());
        bRight.setPower(bR());
    }
}