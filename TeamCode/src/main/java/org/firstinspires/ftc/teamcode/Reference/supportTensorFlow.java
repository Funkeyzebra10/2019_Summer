package org.firstinspires.ftc.teamcode.Reference;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.Locale;

public class supportTensorFlow extends OpMode{
    //starting case
    int step = 0, stepNudgeNudge = 0;
    //motors
    DcMotor left, right, liftMotor, shoulder, elbow;
    //servos
    CRServo latch;
    //sensors
    DigitalChannel magSwitchUp, magSwitchDown, magLatchOpen, magLatchClose;
    AnalogInput pShoulder, pElbow;

    //gyro define
    BNO055IMU imu;
    //gyro telemetry
    Orientation angles;
    Acceleration gravity;
    String position;

    //used for encoders
    static final double     EXTERNAL_GEARING        = 1.9;
    static final double     COUNTS_PER_MOTOR_REV    = 28.0 ;    // eg: AndyMark NeverRest40 Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 40.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415))/EXTERNAL_GEARING;

    //TensorFlow
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "Ad2f6Yj/////AAABmXK3skr/8k3JphGJ3t4x40MrNaFtHWPxASQcuhl0pNyZt3t14n1CXxgO7IaCDGEuoLgUCuGvbqurJ8guQ4RPja+5rPoGASD3NZYsHLoePBf2ngj406AkQ/Vnu1kqELtIQ1M/VjForpboQLGVwDPpSWfFgq3JRFQGr5H7sJ3boigJpE6dLbIM+58TVahcgoSqqILbDe5Zq8Epv1YMJz9brXN+AmRvRfE5AkV+N5ohsxud5HCjbEv/pmtPoYXTUiFf7zlQJz+0x9mBbeBaDkwzqoXcrTvJkDyKclwPLUTz9AJiOeMMHI+VfLpflfFtOT/ucVUYV91ogpFnsyXVEpd/C+LW5kwdseqzdflR4EZT9sAw";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    public void init(){

        //hardware map for config file
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        liftMotor = hardwareMap.dcMotor.get("lift");
        shoulder = hardwareMap.dcMotor.get("arm");
        //elbow = hardwareMap.dcMotor.get("elbow");
        latch = hardwareMap.crservo.get("latch");
        magSwitchUp = hardwareMap.digitalChannel.get("magSwitchUp");
        magSwitchDown = hardwareMap.digitalChannel.get("magSwitchDown");
        magLatchClose = hardwareMap.digitalChannel.get("magLatchClose");
        magLatchOpen = hardwareMap.digitalChannel.get("magLatchOpen");
        pShoulder = hardwareMap.analogInput.get("pShoulder");
        pElbow = hardwareMap.analogInput.get("pElbow");

        //set directions
        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shoulder.setDirection(DcMotorSimple.Direction.FORWARD);
        //elbow.setDirection(DcMotorSimple.Direction.FORWARD);
        latch.setDirection(DcMotorSimple.Direction.REVERSE);

        //brake for wheels
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //gyro telemetry
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

        //gyro parameters
        BNO055IMU.Parameters gParameters = new BNO055IMU.Parameters();
        gParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        gParameters.loggingEnabled = true;
        gParameters.loggingTag = "IMU";
        //gParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        //gyro initialize
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(gParameters);

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

        //initialize TensorFlow
        initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Activate Tensor Flow Object Detection. */
        if (tfod != null) {
            tfod.activate();
        }
    }

    public void init_loop(){
        //get TensorFlow info
        goldALlign();
        telemetry.addData("order:", position);
    }

    public void loop(){
    }

    /**
     * Stops all motors on the robot
     */
    public void stop(){
        left.setPower(0);
        right.setPower(0);
        liftMotor.setPower(0);
        shoulder.setPower(0);
        latch.setPower(0);
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

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    /**
     * The robot will drive straight a given distance
     * @param direction direction of the robot. Forward is moving in the direction of the arm
     * @param distance how far robot goes. "Should" be in inches
     * @param speed how fast the robot goes.
     */
    public void driveStraight(String direction, int distance, double speed) {
        resetEncoder();
        int mDirection = 1;
        if (direction.equals("backward")) {
            mDirection = -1;
        }
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set position
        right.setTargetPosition((int) COUNTS_PER_INCH * distance * mDirection);
        left.setTargetPosition((int) COUNTS_PER_INCH * distance * mDirection);

        //set power
        left.setPower(speed);
        right.setPower(speed);

        //while busy
        while (right.isBusy()&& left.isBusy()) {
            telemetry.addData("left:",left.getCurrentPosition()*COUNTS_PER_INCH);
            telemetry.addData("right:",right.getCurrentPosition()*COUNTS_PER_INCH);
            telemetry.update();
        }
        left.setPower(0);
        right.setPower(0);
        //resetEncoder();
    }

    /**
     * Resets drive encoders
     */
    public void resetEncoder(){
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * turns off tensorFlow
     */
    public void disableDetector(){
        tfod.shutdown();
    }


    /**
     * sets the position of the mineral using
     * tensorFlow by reading the 2 rightmost minerals
     */
    public void goldALlign()
    {
        //String retVal = "NULL";
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                if (updatedRecognitions.size() > 0) {
                    int goldMineralX = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                        }
                    }
                    if (goldMineralX != -1) {
                        if(goldMineralX>265)
                        {
                            position="RIGHT";
                        }
                        else
                        {
                            position="CENTER";
                        }
                        telemetry.addData("X-Position:",goldMineralX);
                    }
                    else
                    {
                        position="LEFT";
                    }
                    telemetry.addData("Order: ",position);
                }
                telemetry.update();
            }
        }


       // return retVal;
    }


    /**
     *sets the position of the shoulder using the
     * potentiometer
     * @param servoPosition position of the shoulder
     */
    public void mervoShoulder(double servoPosition) {
        double servoPos = servoPosition;
        double voltReadingS = (float) pShoulder.getVoltage();
        telemetry.addData("voltage:", voltReadingS);
        double mervoValueS = voltReadingS / 3.25;
        telemetry.addData("servoValue shoulder:", mervoValueS);

        double inc1 = servoPos - 0.01;//
        double inc2 = servoPos + 0.01;

        while (mervoValueS < (servoPos - 0.01) || mervoValueS > (servoPos + 0.01)) {
            voltReadingS = (float) pShoulder.getVoltage();
            mervoValueS = voltReadingS / 3.25;
            if (mervoValueS < inc1) {
                if (mervoValueS < 0.7 && mervoValueS > 0.4) {
                    shoulder.setPower(0.7);//0.3
                } else {
                    shoulder.setPower(0.7);//0.4
                }
                telemetry.addLine("move motor forward");
            } else if (mervoValueS > inc2) {
                if (mervoValueS < 0.7 && mervoValueS > 0.4) {
                    shoulder.setPower(-0.7);//-0.4
                } else {
                    shoulder.setPower(-0.7);//-0.5
                }
                telemetry.addLine("move motor backward");
            } else {
                stop();
            }

        }
        stop();

    }


    /**
     * Brings the lift down until it
     * hits the magnetic switch
     */
    public void liftDown() {
        if (!magSwitchDown.getState()) {
            liftMotor.setPower(0);
        }
        else
        {
            liftMotor.setPower(-1);
        }
    }

    /**
     * Brings the lift up until it
     * hits the magnetic switch
     */
    public void liftUp()
    {
        if(!magSwitchUp.getState())
        {
            liftMotor.setPower(0);
        }
        else
        {
            liftMotor.setPower(1);
        }
    }

    /**
     * Opens the latch until it hits the
     * magnetic switch
     */
    public void latchOpen()
    {
        if(!magLatchOpen.getState())
        {
            latch.setPower(0);
        }
        else
        {
            latch.setPower(1);
        }
    }

    /**
     * Turns accurate angles depending on the robot's initial position
     * @param targetAngle the angle you want the robot to
     * @param speed
     */
    public void gyroTest(double targetAngle, double speed) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentAngle = angles.firstAngle;
        telemetry.addData("angle: ", angles.firstAngle);
        telemetry.update();

        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (Math.abs(currentAngle - targetAngle) > 4) {
            currentAngle = angles.firstAngle;
            telemetry.update();

            if (currentAngle > targetAngle) {//turn left
                telemetry.update();
                left.setPower(-speed);
                right.setPower(speed);
                telemetry.update();
            }
            else if (currentAngle < targetAngle) {
                telemetry.update();
                left.setPower(speed);
                right.setPower(-speed);
                telemetry.update();
            }
            else {
                left.setPower(0);
                right.setPower(0);
                telemetry.update();
            }
            telemetry.update();
            telemetry.addData("angle: ", angles.firstAngle);
        }
        telemetry.update();
        left.setPower(0);
        right.setPower(0);

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



}
