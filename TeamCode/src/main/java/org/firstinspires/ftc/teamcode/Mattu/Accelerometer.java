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

    //IMU variables
    Acceleration gravity; //Acceleration
    AngularVelocity angular; //Angular Velocity
    double lastAngularZ; //Angular Velocity
    double speedX, speedY, dTime; //Speed

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
        dTime = 0;
        speedX = 0;
        speedY = 0;
    }

    public void loop() {
        gravity = imu.getAcceleration();
        angular = imu.getAngularVelocity();

        speedX += gravity.xAccel * (System.currentTimeMillis() - dTime) / 1000;
        speedY += gravity.yAccel * (System.currentTimeMillis() - dTime) / 1000;
        dTime = System.currentTimeMillis();

        //Telemetry of speed, acceleration, and angular velocity
        telemetry.addData("X Speed (m/s): ", speedX);
        telemetry.addData("Y Speed (m/s): ", speedY);
        telemetry.addData("X Acceleration (m/s/s): ", gravity.xAccel);
        telemetry.addData("Y Acceleration (m/s/s): ", gravity.yAccel);
        telemetry.addData("Z Acceleration (m/s/s): ", imu.getGravity().zAccel);
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
    }

    public void stop() {
        fLeft.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);
    }
}