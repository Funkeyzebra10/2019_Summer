package org.firstinspires.ftc.teamcode.HelloItsAdam;

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

@TeleOp(name="AroundObjectTest")
public class AroundObjectTest extends OpMode {

    DcMotor fLeft, fRight, bLeft, bRight;
    DistanceSensor distanceL, distanceR, distanceC;
    BNO055IMU imu;
    Orientation angles;
    int step=0;

    //used for encoders
    static final double     EXTERNAL_GEARING        = 1;
    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;  //28  // eg: AndyMark NeverRest40 Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.937 ;     // For figuring circumference
    public static final double     COUNTS_PER_INCH         = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415))/EXTERNAL_GEARING;

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
        telemetry.addData("current ",fRight.getCurrentPosition()/COUNTS_PER_INCH);
        if(step==0) {
            driveSafe("left", 48);
        }

        step++;
    }
    public void stop(){

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

    public void driveSafe(String direction, int distance) {
        boolean saw=false;
        double initialDistance;//distance driven before driving around object
        double clearDistance;//distance driven along side object
        double turnDistance;//distance driven to avoid object

        resetEncoderWithoutEncoder();

        while(fRight.getCurrentPosition()/COUNTS_PER_INCH < distance){
            fLeft.setPower(0.1);
            fRight.setPower(0.1);
            bLeft.setPower(0.1);
            bRight.setPower(0.1);
            if(distanceC.getDistance(DistanceUnit.CM)<10){
                saw=true;
                break;
            }
        }
        if(saw){
            pause(3000);
            if(distanceC.getDistance(DistanceUnit.CM)<10) {
                saw = true;
            }

            initialDistance=fRight.getCurrentPosition()/COUNTS_PER_INCH;

            if(saw && direction.equals("left")){

                Rotation(270);
                resetEncoderWithoutEncoder();
                while(distanceR.getDistance(DistanceUnit.CM)<20){
                    fLeft.setPower(0.2);
                    fRight.setPower(0.2);
                    bLeft.setPower(0.2);
                    bRight.setPower(0.2);
                }
                turnDistance=fRight.getCurrentPosition()/COUNTS_PER_INCH;

                Rotation(90);
                resetEncoderWithoutEncoder();
                while(distanceR.getDistance(DistanceUnit.CM)<20){
                    fLeft.setPower(0.2);
                    fRight.setPower(0.2);
                    bLeft.setPower(0.2);
                    bRight.setPower(0.2);
                }
                clearDistance=fRight.getCurrentPosition()/COUNTS_PER_INCH;

                Rotation(90);
                resetEncoderWithoutEncoder();
                driveStraight("forward",turnDistance);

                Rotation(270);
                resetEncoderWithoutEncoder();
                driveStraight("forward",distance-(initialDistance+clearDistance));
            }
            else if(saw && direction.equals("right")){

            }
        }

        fLeft.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);
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
        while(System.currentTimeMillis()<fTime){}
    }

    public void driveStraight(String direction, double distance) {
        resetEncoderWithoutEncoder();
        int mDirection = 1;
        if (direction.equals("backward")) {
            mDirection = -1;
        }

        double target=distance * mDirection;

        while(fRight.getCurrentPosition()/COUNTS_PER_INCH < target){
            fLeft.setPower(0.2);
            fRight.setPower(0.2);
            bLeft.setPower(0.2);
            bRight.setPower(0.2);
        }

        fLeft.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);
    }
}
