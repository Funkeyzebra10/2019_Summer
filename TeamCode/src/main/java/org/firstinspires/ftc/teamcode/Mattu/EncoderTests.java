package org.firstinspires.ftc.teamcode.Mattu;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

@TeleOp(name="EncoderTest")
public class EncoderTests extends OpMode {

    DcMotor fLeft, fRight, bLeft, bRight;
    DistanceSensor distanceL, distanceR, distanceC;
    BNO055IMU imu;
    Orientation angles;

    public enum DriveState {
        M_FORWARD,
        M_BACKWARD,
        M_LEFT,
        M_RIGHT,
        M_NULL
    }
    public DriveState driveState = DriveState.M_FORWARD;
    public DriveState prevState = DriveState.M_NULL;

    double distanceRemaining = 0;

    //used for encoders
    static final double     EXTERNAL_GEARING        = 1;
    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;  // eg: AndyMark NeverRest40 Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.937 ;     // For figuring circumference
    static final double     ROLLER_WHEEL_CIRC       = 13 ;    //Used for strafing
    public static final double     COUNTS_PER_INCH_NORM         = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415)) / EXTERNAL_GEARING;       //Used for driving straight
    public static final double      COUNTS_PER_INCH_STRAFE      = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            ROLLER_WHEEL_CIRC) / EXTERNAL_GEARING;          //Used for strafing

    public void init(){
        //Hardware mapping of the four motors
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");

        distanceL = hardwareMap.get(DistanceSensor.class, "distanceL");
        distanceR = hardwareMap.get(DistanceSensor.class, "distanceR");
        distanceC = hardwareMap.get(DistanceSensor.class, "distanceC");

        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bRight.setDirection(DcMotor.Direction.FORWARD);
        bLeft.setDirection(DcMotor.Direction.REVERSE);

        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set up imu parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
        });

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }
    public void loop(){
        //telemetry.addData("target ", fRight.getCurrentPosition()*COUNTS_PER_INCH);
        telemetry.addData("Current ",fRight.getCurrentPosition()/COUNTS_PER_INCH_NORM);

        switch (driveState) {
            case M_FORWARD:
                if (prevState != DriveState.M_FORWARD) {
                    resetEncoderWithoutEncoder();
                    distanceRemaining = 15;
                }
                driveStraight(distanceRemaining, "forward");
                driveState = DriveState.M_BACKWARD;
                break;
            case M_BACKWARD:
                if (prevState != DriveState.M_BACKWARD) {
                    resetEncoderWithoutEncoder();
                    distanceRemaining = 15;
                }
                driveStraight(15, "backward");
                driveState = DriveState.M_LEFT;
                break;

            case M_LEFT:
                if (prevState != DriveState.M_LEFT) {
                    resetEncoderWithoutEncoder();
                    distanceRemaining = 15;
                }
                driveStrafe(distanceRemaining, "left");
                driveState = DriveState.M_RIGHT;
                break;
            case M_RIGHT:
                if (prevState != DriveState.M_RIGHT) {
                    resetEncoderWithoutEncoder();
                    distanceRemaining = 15;
                }
                driveStrafe(distanceRemaining, "right");
                driveState = DriveState.M_FORWARD;
                break;
            case M_NULL:
                break;
        }

        prevState = driveState;
    }

    public void driveStrafe(double distance, String direction) {
        double startPos = fRight.getCurrentPosition() / COUNTS_PER_INCH_STRAFE;

        int mDirection = 1;
        if (direction.equals("right")) {
            mDirection = -1;
        }

        double target = distance * mDirection;

        if ((mDirection == 1 && fRight.getCurrentPosition() / COUNTS_PER_INCH_STRAFE < target) || (mDirection == -1 && fRight.getCurrentPosition() / COUNTS_PER_INCH_STRAFE > target)){
            fLeft.setPower(-0.2 * mDirection);
            fRight.setPower(0.2 * mDirection);
            bLeft.setPower(0.2 * mDirection);
            bRight.setPower(-0.2 * mDirection);
        }
        else {
            fLeft.setPower(0);
            fRight.setPower(0);
            bLeft.setPower(0);
            bRight.setPower(0);
        }

        distanceRemaining -= fRight.getCurrentPosition() - startPos;
    }

    public void driveStraight(double distance, String direction) {
        double startPos = fRight.getCurrentPosition() / COUNTS_PER_INCH_NORM;

        int mDirection = 1;
        if (direction.equals("backward")) {
            mDirection = -1;
        }

        double target=distance * mDirection;

        if ((mDirection == 1 && fRight.getCurrentPosition() / COUNTS_PER_INCH_NORM < target) || (mDirection == -1 && fRight.getCurrentPosition() / COUNTS_PER_INCH_NORM > target)){
            fLeft.setPower(0.2 * mDirection);
            fRight.setPower(0.2 * mDirection);
            bLeft.setPower(0.2 * mDirection);
            bRight.setPower(0.2 * mDirection);
        }
        else {
            fLeft.setPower(0);
            fRight.setPower(0);
            bLeft.setPower(0);
            bRight.setPower(0);
        }

        distanceRemaining -= fRight.getCurrentPosition() - startPos;
    }

    public void stop(){
        fLeft.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);
    }

    /**
     * Resets drive encoders without running using encoders
     */
    public void resetEncoderWithoutEncoder(){
        bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     *
     * @param destination
     */
    public void Rotation(float destination) {
        fRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double speed = 0;
        double min = 0.1;
        double max = 0.8;
        double iTime=System.currentTimeMillis();

        //standard current angle
        double heading = cvtDegrees(angles.firstAngle);

        /**
         * This will determine the actual intended heading from 0-360
         */
        //check if over 360
        if (Math.abs(destination)>360) {
            //positive
            if(destination>0) {
                while(destination>360) {
                    destination-=360;
                    System.out.println(destination);
                }//end while
            }
            //negative
            else {
                while(destination<-360) {
                    destination+=360;
                    System.out.println(destination);
                }//end while
            }//end if else
        }//end greater than 360
        destination = Math.abs(destination);// convert to positive value
        if (destination==0) {//if 360 set to 0 as they are the same heading
            destination = 360;
        }
        if (heading==0) {//if 360 set to 0 as they are the same heading
            heading = 360;
        }

        //main phase of method
        while (heading < destination - 2 || heading > destination + 2) {
            telemetry.addData("heading", heading);
            telemetry.addData("speed: ", speed);
            telemetry.addData("destination:", destination);
            telemetry.update();

            double delta = destination-heading; //the difference between destination and heading
            heading = cvtDegrees(angles.firstAngle);
            //decreases speed as robot approaches destination
            speed = (1 - ((heading) / destination)) * ((destination - heading) * 0.01);


            //if the speed gets under the min speed it will use the min speed
            if (Math.abs(speed) < min && Math.abs(speed) != 0) {
                speed = min;
            }
            //if the speed is over the max it will use max speed
            if(Math.abs(speed) > max){
                speed=max;
            }
            if (!(Math.abs(delta) == 360 || Math.abs(delta) == 0)) {//determine if we are at the intended heading
                if (((delta + 360) % 360) > 180) { //Chooses fastest route by determining if the arc length is longer to the right or left. Chooses fastest route by
                    fLeft.setPower(-speed);
                    bLeft.setPower(-speed);
                    bRight.setPower(speed);
                    fRight.setPower(speed);
                } else {
                    fLeft.setPower(speed);
                    bLeft.setPower(speed);
                    bRight.setPower(-speed);
                    fRight.setPower(-speed);
                }
            } else {
                fLeft.setPower(0);
                bLeft.setPower(0);
                bRight.setPower(0);
                fRight.setPower(0);
            }
            if(System.currentTimeMillis()>iTime+4500){
                break;
            }
        }
        fLeft.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);
        fRight.setPower(0);
    }

    public double cvtDegrees(double heading) {
        /**
         * convert degrees from -180<->180 to 0<->360
         */
        if (heading <0 ) {
            return 360 + heading;
        } else {
            return heading;
        }
    }

    /**
     * a function that is needed to format the gyro angle
     * @param angleUnit
     * @param angle
     * @return
     */
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    /**
     * a function that is needed to format the gyro angle
     * @param degrees
     * @return
     */
    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    /**
     * will stall the program for the inputted amount of time
     * wait time cannot be more than 5 seconds
     * @param milTime the amount of time to wait in seconds
     */
    public void pause(double milTime){
        double iTime =System.currentTimeMillis();
        double fTime=iTime+milTime;
        while(System.currentTimeMillis()<fTime){

        }
    }
}
