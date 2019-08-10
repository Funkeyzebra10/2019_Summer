package org.firstinspires.ftc.teamcode.Mattu;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Accelerometer Test")
public class Accelerometer extends OpMode {
    //Sensor and motor variables
    DcMotor fLeft, fRight, bLeft, bRight;
    BNO055IMU imu;

    // State used for updating telemetry
    String state="";

    //Time variables
    ElapsedTime mTime = new ElapsedTime();
    double goalTime;

    //Acceleration variables
    Acceleration gravity;
    AngularVelocity angular;
    double lastAngularZ;

    //Variables for auto-correction
    double xTotal, yTotal;
    int count;
    double xGoal, yGoal;

    //Encoder variables
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.937 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                        (WHEEL_DIAMETER_INCHES * 3.1415);

    public void init() {
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");

        fLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        fRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotor.Direction.REVERSE);
        bLeft.setDirection(DcMotor.Direction.FORWARD);

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

        lastAngularZ = 0;
    }

    public void loop() {
        gravity = imu.getAcceleration();
        angular = imu.getAngularVelocity();

        //Telemetry of acceleration
        telemetry.addData("X Acceleration", gravity.xAccel);
        telemetry.addData("Y Acceleration", gravity.yAccel);
        telemetry.addData("Z Acceleration (Auto-calibrated for gravity)", gravity.zAccel);
        telemetry.addData("Angular Velocity", angular.zRotationRate);
        telemetry.addData("State: ", state);

        if (System.currentTimeMillis() % 10 == 0) {
            if (angular.zRotationRate > lastAngularZ + 1 || angular.zRotationRate < lastAngularZ - 1) {
                state="HIT";
            } else {
                state = "MISS";
            }
            lastAngularZ = angular.zRotationRate;
        }

        /*if (goalTime <= 0) {
            stop();
            if (gravity.xAccel >= 0.1 || gravity.xAccel <= -0.1 || gravity.yAccel >= 0.1 || gravity.yAccel <= -0.1) {
                mTime.reset();
                xTotal += gravity.xAccel;
                yTotal += gravity.yAccel;
                count++;
            } else {
                goalTime = mTime.time(TimeUnit.SECONDS);
                xGoal = 39.3700787 * (-(xTotal / count) * goalTime);
                yGoal = 39.3700787 * (-(yTotal / count) * goalTime);
            }
        } else {
            encoderDrive(xGoal, yGoal);
            goalTime = 0;
        }

        if (goalTime == 0 && !fLeft.isBusy() && !fRight.isBusy() && !bLeft.isBusy() && !bRight.isBusy()) {
            fLeft.setPower(0);
            fRight.setPower(0);
            bLeft.setPower(0);
            bRight.setPower(0);

            fLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }*/
    }

    public void encoderDrive(double xDist, double yDist) {
        // Determine new target position, and pass to motor controller
        int fLTarget = fLeft.getCurrentPosition() + (int)((yDist + xDist) * COUNTS_PER_INCH);
        int fRTarget = fRight.getCurrentPosition() + (int)((yDist - xDist) * COUNTS_PER_INCH);
        int bLTarget = bLeft.getCurrentPosition() + (int)((yDist - xDist) * COUNTS_PER_INCH);
        int bRTarget = bRight.getCurrentPosition() + (int)((yDist + xDist) * COUNTS_PER_INCH);
        fLeft.setTargetPosition(fLTarget);
        fRight.setTargetPosition(fRTarget);
        bLeft.setTargetPosition(bLTarget);
        bRight.setTargetPosition(bRTarget);

        // Turn On RUN_TO_POSITION
        fLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        fLeft.setPower(Math.abs(100/128));
        fRight.setPower(Math.abs(100/128));
        bLeft.setPower(Math.abs(100/128));
        bRight.setPower(Math.abs(100/128));
    }

    public void stop() {
        fLeft.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);
    }
}