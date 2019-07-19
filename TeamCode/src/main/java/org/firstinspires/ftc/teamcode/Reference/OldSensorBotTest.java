package org.firstinspires.ftc.teamcode.Reference;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Locale;

/**
 * Created by emory on 8/19/2017.
 */
//@Autonomous(name = "OldSensorBotTest")

public class OldSensorBotTest extends OpMode {
    //defining devices
    DcMotor bRight, bLeft, fRight, fLeft;
    //Servo lHand,rHand;
    ColorSensor color;
    ModernRoboticsI2cRangeSensor range;

    ElapsedTime timer = new ElapsedTime();
    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;


    //navx micro gyro define
    //PIDCoefficients yawPIDController;
    //navxPIDController yawPIDController;
    //IntegratingGyroscope gyro;
    //NavxMicroNavigationSensor navxMicro;


    public void init(){

        //initializing devices
        fLeft =hardwareMap.dcMotor.get("fLeft");
        fRight =hardwareMap.dcMotor.get("fRight");
        bLeft=hardwareMap.dcMotor.get("bLeft");
        bRight=hardwareMap.dcMotor.get("bRight");
        color = hardwareMap.colorSensor.get("color");
        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");
        //lHand=hardwareMap.servo.get("lHand");
        //rHand=hardwareMap.servo.get("rHand");
      //  navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
      //  gyro = navxMicro;

        //setting motor direction
        fLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        fRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        bRight.setDirection(DcMotorSimple.Direction.REVERSE);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameter = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        //parameters for range sensor
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "imu";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        parameter.vuforiaLicenseKey = "Afe8mtn/////AAAAGSEHpcPbGksXnZDY1oNZ4stF2BoZwT7Xgcdi3AuWqz3ZGmgN8lUJqEQzvS9hmsvT+5ERk+B2c9iol+6TxH/AhiWr/D8jExF0BSuV22m29ctHLg0QHoo5xJH9Dqr98eojHO3w7181LIaPKBPUvu4WyeODzlNXYDy8IQ5Xq3CuOGEuq/e967FWjhr1Z/OsNgrMh9Gwh28vPqleIfZ3kvSQEArGesl3BuRKYs2w3CX1deteJRjDZ6ayuEBlGFERsy2phkd7uqa9tHiNxVgQt6KBL/mcAJIzvU1rTuq2HPumNMsmw0UMnH35IHyT7X+uieeh+ooHH9XePCL2GFWjHyNQOPUKXe5uSlcWfK2kRzkt8xFF";

        parameter.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameter);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */


        // Set up our telemetry dashboard
        //composeTelemetry();

        // Wait until we're told to go

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

       /* timer.reset();
        while (navxMicro.isCalibrating())  {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds())%2==0 ? "|.." : "..|");
            telemetry.update();
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear(); telemetry.update();

        */

       //telemetry for internal gyro
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

        //navx micro gyro define for telemetry


      //  AngularVelocity rates = gyro.getAngularVelocity(AngleUnit.DEGREES);
     //   Orientation angles = gyro.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

/*
        yawPIDController = new yawPIDController( gyro,
                navXPIDController.navXTimestampedDataSource.YAW);

        // Configure the PID controller
        yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(TimestampedPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
        yawPIDController.enable(true);*/

        /*telemetry.addData("status: ", new Func<String>() {
            @Override
            public String value() {
                return imu.getSystemStatus().toShortString();
            }
        });
        telemetry.addData("blue:", new Func<Integer>() {
            @Override
            public Integer value() {
                return color.blue();
            }
        });
        telemetry.addData("red:", new Func<Integer>() {
            @Override
            public Integer value() {
                return color.red();
            }
        });*/

        //telemetry for range sensor
        telemetry.addData("Range:", new Func<Double>() {
            @Override
            public Double value() {
                return range.getDistance(DistanceUnit.CM);
            }
        });

        //navx micro telemetry
       /* telemetry.addLine()
                .addData("dx", rates.xRotationRate)
                .addData("dy", rates.yRotationRate)
                .addData("dz", "%s deg/s", rates.zRotationRate);

        telemetry.addLine()
                .addData("heading", formatAngle(angles.angleUnit, angles.firstAngle))
                .addData("roll", formatAngle(angles.angleUnit, angles.secondAngle))
                .addData("pitch", "%s deg", formatAngle(angles.angleUnit, angles.thirdAngle));
        telemetry.update();
*/
       /* telemetry.addData("calib", new Func<String>() {
            @Override
            public String value() {
                return imu.getCalibrationStatus().toString();
            }
        });


        telemetry.addData("heading", new Func<String>() {
            @Override
            public String value() {
                return formatAngle(angles.angleUnit, angles.firstAngle);
            }
        });
        telemetry.addData("roll", new Func<String>() {
            @Override
            public String value() {
                return formatAngle(angles.angleUnit, angles.secondAngle);
            }
        });
        telemetry.addData("pitch", new Func<String>() {
            @Override
            public String value() {
                return formatAngle(angles.angleUnit, angles.thirdAngle);
            }
        });*/

        //telemetry for internal gyro
        telemetry.addData("grvty", new Func<String>() {
            @Override
            public String value() {
                return gravity.toString();
            }
        });
        telemetry.addData("mag", new Func<String>() {
            @Override
            public String value() {
                return String.format(Locale.getDefault(), "%.3f",
                        Math.sqrt(gravity.xAccel * gravity.xAccel
                                + gravity.yAccel * gravity.yAccel
                                + gravity.zAccel * gravity.zAccel));
            }
        });

        //setting servos
        //rHand.setDirection(Servo.Direction.REVERSE);
        //rHand.setPosition(0.5);
        //lHand.setPosition(0);

    }//end of init
    public void loop(){

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        relicTrackables.activate();

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            telemetry.addData("VuMark", "%s visible", vuMark);
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();

            //telemetry.addData("Pose", pose);
            if (pose != null) {
                VectorF trans = pose.getTranslation();
                Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                // Extract the X, Y, and Z components of the offset of the target relative to the robot
                double tX = trans.get(0);
                double tY = trans.get(1);
                double tZ = trans.get(2);

                // Extract the rotational components of the target relative to the robot
                double rX = rot.firstAngle;
                double rY = rot.secondAngle;
                double rZ = rot.thirdAngle;

                if(tZ <= -450.0){
                    fLeft.setPower(0.3);
                    fRight.setPower(0.3);
                }
                else if(tZ <= -10.0 && tZ >= -250.0){
                    fLeft.setPower(-0.3);
                    fRight.setPower(-0.3);
                }
                else{
                    fLeft.setPower(0);
                    fRight.setPower(0);
                }
            }
        }






/*
        telemetry.addData("rHand: ",rHand.getPosition());
        telemetry.addData("lHand: ",lHand.getPosition());

        boolean rGlyph = gamepad1.right_bumper;
        boolean zero = gamepad1.b;

        if(zero){
            rHand.setPosition(0.5);
            lHand.setPosition(0);
        }

        if(rGlyph){
            rHand.setPosition(1);
            lHand.setPosition(1);

        }*/



        telemetry.update();

        /*if(range.getDistance(DistanceUnit.CM) < 10){
            fLeft.setPower(1);
            bLeft.setPower(1);
            fRight.setPower(1);
            bRight.setPower(1);
        }
        else if(color.red() >= 1){
            fLeft.setPower(-1);
            bLeft.setPower(-1);
            fRight.setPower(-1);
            bRight.setPower(-1);
        }
        else{
            fLeft.setPower(0);
            bLeft.setPower(0);
            fRight.setPower(0);
            bRight.setPower(0);
        }
        */


        //assigning functions to controllers
        double lDrive=gamepad1.left_stick_y;
        double rDrive=gamepad1.right_stick_y;

        //for joystick control
        fLeft.setPower((lDrive*100)/128);
        bLeft.setPower((lDrive*100)/128);
        fRight.setPower((rDrive*100)/128);
        bRight.setPower((rDrive*100)/128);

    }//end of loop

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    public void stop(){
        //stopping all moving devices
        fLeft.setPower(0);
        bLeft.setPower(0);
        fRight.setPower(0);
        bRight.setPower(0);
    }//end of stop

/*
    public void TurnAngle(String direction, float angle) {
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        int tDirection = 1;
        if (direction.equals("left")){
            tDirection = -1;
        }

        if(angle > 0) {
            while (gyro. < angle) {
                m_drive.tankDrive(0.5 * tDirection, -0.5 * tDirection);
                if(!(isAutonomous()&&isEnabled())) {
                    break;
                }
            }
            m_drive.stopMotor();
        }
        else if(angle < 0) {
            while (gyro.getAngle() > angle && m_timer.get() < 5) {
                m_drive.tankDrive(0.5 * tDirection, -0.5 * tDirection);
                if(!(isAutonomous()&&isEnabled())) {
                    break;
                }
                m_drive.stopMotor();
            }
        }
    }
*/
   /* public void turnToAngle(int degrees){
        //direction modifier
        int directM = -1;
        //find shortest path
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double yaw=AngleUnit.DEGREES.fromUnit(angles.angleUnit,angles.firstAngle);

        if(degrees-yaw>0){
            directM=1;
        }


        //direction of turn
        //turn
    }//end of turnToAngle method

   String formatAngle(AngleUnit angleUnit, double angle) {
       return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
   }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    */
}//end of OpMode
