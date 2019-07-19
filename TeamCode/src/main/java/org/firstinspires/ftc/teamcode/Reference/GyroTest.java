package org.firstinspires.ftc.teamcode.Reference;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

/**
 * Created by emory on 8/15/2018.
 */
//F@Autonomous(name = "GyroTest")

public class GyroTest extends OpMode {
    DcMotor bRight, bLeft, fRight, fLeft; //left, right;
    //gyro define
    BNO055IMU imu;
    // gyro telemetry
    Orientation angles;
    Acceleration gravity;
    double retValR;
    int step = 0;
    String tData ="";
     /**
     * this method converts the original gyro input(-180 to 180), to a
     * 0 to 360 form. This method is used for turning right(clockwise)
     * @param degrees the raw degrees that the gyro returns
     * @return turn the inputted -180 to 180 degrees to 0- to 60 degree format.
     */
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

    /**
     * this method converts the original gyro input(-180 to 180), to a
     * 0 to 360 form. This method is used for turning left(counter-clockwise)
     * @param degrees the raw degrees that the gyro returns
     * @return turn the inputted -180 to 180 degrees to 0- to 60 degree format.
     */
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

    static final double     COUNTS_PER_MOTOR_REV    = 28.0 ;    // eg: AndyMark NeverRest40 Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 20.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    public void init() {
        /*left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");*/
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");

        /*left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.FORWARD);*/
        fLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        fRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotor.Direction.REVERSE);
        bLeft.setDirection(DcMotor.Direction.FORWARD);


        //gyro parameters
        BNO055IMU.Parameters gParameters = new BNO055IMU.Parameters();
        gParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        gParameters.loggingEnabled = true;
        gParameters.loggingTag = "IMU";
        gParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        //gyro initialize
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(gParameters);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        //you didn't have this line to give the angls object and data
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


        telemetry.addData("status: ", imu.getSystemStatus().toShortString());
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
            }
        });
        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });


        //waitForStart();


        //telemetry.update();

    }

    public void init_loop(){
        telemetry.addData("Angle: ", angles.firstAngle);
        telemetry.addData("Langle: ", cvtDegreesL(angles.firstAngle));
        telemetry.addData("Rangle: ", cvtDegreesR(angles.firstAngle));
    }

    public void loop(){

        switch (step) {
            case 0:
                gyroEm(90);
                break;
            case 1:
                gyroEm(-90);//0.35
                break;
            case 2:
                requestOpModeStop();
                break;
            default:
                requestOpModeStop();
                break;
        }step++;

    }

    public void stop(){
        /*left.setPower(0);
        right.setPower(0);*/
        fLeft.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);
    }


    public void turnToAngle(double targetAngle) {
        double currentAngle = cvtDegreesR(angles.firstAngle);
        telemetry.addData("angle: ", currentAngle);

        while (currentAngle < targetAngle - 2.5 || currentAngle > targetAngle + 2.5) {
            currentAngle = cvtDegreesR(angles.firstAngle);
            if (currentAngle < targetAngle + 10) {//turn right
                fLeft.setPower(0.25);
                fRight.setPower(-0.25);
                bLeft.setPower(0.25);
                bRight.setPower(-0.25);
            } else if (currentAngle > targetAngle + 10 || currentAngle < targetAngle + 15) {
                fLeft.setPower(-0.25);
                fRight.setPower(0.25);
                bLeft.setPower(-0.25);
                bRight.setPower(0.25);
            }
            fLeft.setPower(0);
            fRight.setPower(0);
            bLeft.setPower(0);
            bRight.setPower(0);
        }
    }

    //gets stuck in loop :(
    public void Gyro2Right (double targetAngle, double speed) {

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentAngle = cvtDegreesR(angles.firstAngle);
        telemetry.addData("angle: ", cvtDegreesR(angles.firstAngle));
        telemetry.update();

        while (Math.abs(currentAngle - targetAngle) > (speed * 40)) {
            currentAngle = cvtDegreesR(angles.firstAngle);
            telemetry.update();

            if (currentAngle > targetAngle) {//turn left
                telemetry.update();
                fLeft.setPower(-speed);
                fRight.setPower(speed);
                bLeft.setPower(-speed);
                bRight.setPower(speed);
                telemetry.update();
            }
            else if (currentAngle < targetAngle) {
                telemetry.update();
                fLeft.setPower(speed);
                fRight.setPower(-speed);
                bLeft.setPower(speed);
                bRight.setPower(-speed);
                telemetry.update();
            }
            else {
                fLeft.setPower(0);
                fRight.setPower(0);
                bLeft.setPower(0);
                bRight.setPower(0);
                telemetry.update();
            }

        }
        telemetry.update();
        stop();
    }

    public void gyroEm(double targetAngle){
        double constant = 0.005;//for the sensor bot
        double minSpeed = 0.15;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentAngle = angles.firstAngle;
        telemetry.addData("angle: ", angles.firstAngle);
        telemetry.update();

        fRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (Math.abs(currentAngle - targetAngle) > 4) {
            currentAngle = angles.firstAngle;
            telemetry.update();

            double speed = 0;
            if(Math.abs(constant*(targetAngle-currentAngle)) > 0.15) {
                speed = constant*(targetAngle-currentAngle);
            }
            else{
                speed = minSpeed;
            }

            if (currentAngle > targetAngle) {//turn left
                telemetry.update();
                fLeft.setPower(speed);
                fRight.setPower(-speed);
                bLeft.setPower(speed);
                bRight.setPower(-speed);
            }
            else if (currentAngle < targetAngle) {
                telemetry.update();
                fLeft.setPower(speed);
                fRight.setPower(-speed);
                bLeft.setPower(speed);
                bRight.setPower(-speed);
            }
            else {
                fLeft.setPower(0);
                fRight.setPower(0);
                bLeft.setPower(0);
                bRight.setPower(0);
            }
            telemetry.update();
            telemetry.addData("angle: ", angles.firstAngle);
        }
        telemetry.update();
        fLeft.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);

    }

    public void resetEncoder(){
        /*right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/
        fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    //@return shows what return @error shows what errors that output
    /**
     * this method takes the yaw that has been inputted, and turn the robot to to that degree.
     * it will slow down as it approaches the desired angle
     * it will choose the shortest direction to the angle
     * this method does not add the inputted degrees too the current yaw, it will turn
     * to the actual yaw position
     * @param /targetAngle  Must be 0-360 degrees. The degree you intend robot to be facing
     */
    /*public void gyroFull(double targetAngle){

        //uses the right convert degrees method to convert degrees to 0 to 360
        double currentAngle = cvtDegreesR(angles.firstAngle);
        double startAngle=currentAngle;
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

    public void gyroFullExp (double targetAngle){
        //Sets current angle
        double currentAngle = cvtDegreesR(angles.firstAngle);

        //Sets telemetry
        tData = currentAngle+"";

        double speed = 1;
        int direction;
        double expSpeed = 1;
        int currentAngleInt = (int)currentAngle;
        int targetAngleInt = (int)(targetAngle);
        int distanceToTravel = Math.abs(targetAngleInt - currentAngleInt);
        double a = 0.125;
        double b = nthroot(distanceToTravel, 8, 0.000001);

        //Minimum speed
        double min = 0.125;

        //int currentDirection;


        while ((currentAngle < targetAngle - 10.5) || (currentAngle > targetAngle + 9.5))
        {

            *//*
            //Updates current angle, rawSpeed, and telemetry
            currentAngle = cvtDegreesR(angles.firstAngle);
            tData = currentAngle+"";
            *//*
            //Sets direction
            if (targetAngle - currentAngle > 0)
            {
                direction = 1;
            }
            else
            {
                direction = -1;
            }

            expSpeed = direction * (Math.pow(Math.abs((180 - (targetAngle - currentAngle))/180), Math.abs(((targetAngle - currentAngle)) / 180)));
            telemetry.addData("Speed: ", expSpeed);

            //Detects if hit minimum
            if(Math.abs(speed) < min && Math.abs(speed) != 0)
            {
                speed = min * direction;
            }
            else if (expSpeed > 1 || expSpeed < 1)
            {
                speed =  direction;
            }
            else
            {
                speed = expSpeed;
            }
            //Updates telemetry
            telemetry.addData("Yaw: ",currentAngle);
            currentAngle = cvtDegreesR(angles.firstAngle);
            telemetry.update();

            //Sets motor speed
            *//*
            fLeft.setPower(-speed);
            bLeft.setPower(-speed);
            fRight.setPower(speed);
            bRight.setPower(speed);
            *//*
        }

        //Zeroes out at end.
        fLeft.setPower(0);
        fLeft.setPower(0);
        fLeft.setPower(0);
        fLeft.setPower(0);

        //Complete!
        telemetry.addLine("Completed");

    }

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
    //end of gyroFullExp*/


    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void Rotation(float destination) {
        double speed = 0;
        double min = 0.5;
        //standard current angle
        double heading = cvtDegrees(angles.firstAngle);
        /**
         * This will determine the actual intended heading from 0-360
         */

        if (Math.abs(destination)>360) { //check if over 360
            //System.out.println("Destination more than 360 degrees");
            if(destination>0) {//positive
                //System.out.println("positive");
                while(destination>360) {
                    destination-=360;
                    System.out.println(destination);
                }//end while
            }else {//negative
                //  System.out.println("negative");
                while(destination<-360) {
                    destination+=360;
                    System.out.println(destination);
                }//end while
            }//end if else
        }//end greater than 360
        destination = Math.abs(destination);// convert to positive value
        if (destination==0) {//if 360 set to 0 as they are the same heading
            destination = 360;
            //System.out.println("destination 0 converted to 360");
        }
        if (heading==0) {//if 360 set to 0 as they are the same heading
            heading = 360;
            //  System.out.println("heading 0 converted to 360");
        }
        //System.out.println("Start:"+heading+"\nFinal destination:" + destination);

        /**
         * This will determine which direction to turn
         */
        double delta = destination-heading; //the difference between destination and heading;
        while (heading < destination - 0.5 || heading > destination + 0.5) {
            telemetry.addData("heading", heading);
            telemetry.addData("speed: ", speed);
            telemetry.addData("destination:", destination);
            telemetry.update();
            heading = cvtDegrees(angles.firstAngle);
            speed = (1 - (heading / destination)) * ((destination - heading) * 0.05);

            //if the speed gets under the min speed it will use the min speed
            if (Math.abs(speed) < min && Math.abs(speed) != 0) {
                speed = min;
            }
            if (!(Math.abs(delta) == 360 || Math.abs(delta) == 0)) {//determine if we are at the intended heading
                if (((delta + 360) % 360) > 180) { //determines if he arc length is longer to the right or left
                    // System.out.println("turn left\n-----");
                   // left.setPower(speed);
                    //right.setPower(-speed);
                } else {
                    //   System.out.println("turn right\n-----");
                    //left.setPower(-speed);
                    //right.setPower(speed);
                }
            } else {
                // System.out.println("Already at inteneded heading\n-----");

            }
        }
        //left.setPower(0);
        //right.setPower(0);

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

}
