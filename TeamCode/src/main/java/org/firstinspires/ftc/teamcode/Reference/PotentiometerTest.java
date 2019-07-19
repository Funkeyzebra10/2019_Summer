package org.firstinspires.ftc.teamcode.Reference;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by emory on 8/15/2018.
 */
//@TeleOp(name = "PotentiometerTestttt")

public class PotentiometerTest extends OpMode {
    DcMotor bRight, bLeft, fRight, fLeft;
    AnalogInput potentiometer;
    boolean potPosition0, potPosition1;




    public void init() {
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");
        potentiometer = hardwareMap.analogInput.get("potentiometer");

        fLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        fRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotor.Direction.REVERSE);
        bLeft.setDirection(DcMotor.Direction.FORWARD);





    }

    public void loop(){
        potPosition0 = gamepad1.x;
        potPosition1 = gamepad1.b;


        double voltReading = (float) potentiometer.getVoltage();
        telemetry.addData("voltage:", voltReading);
        double servoValue = voltReading/3.25;
        telemetry.addData("servoValue:", servoValue);

        if (potPosition0) {
            while(servoValue < .05 || servoValue > .07) {
                voltReading = (float) potentiometer.getVoltage();
                servoValue = voltReading / 3.25;
                if (servoValue < .05) {
                    bLeft.setPower(0.15);
                } else if (servoValue > 0.07) {
                    bLeft.setPower(-0.15);
                } else {
                    stop();
                }
            }
        }
        if (potPosition1) {
            while(servoValue < 0.99|| servoValue > 1.1) {
                voltReading = (float) potentiometer.getVoltage();
                servoValue = voltReading / 3.25;
                if (servoValue < .95) {
                    bLeft.setPower(0.15);
                } else if (servoValue > 1.01) {
                    bLeft.setPower(-0.15);
                } else {
                    stop();
                }
            }
        }


        /*voltReading = (float) potentiometer.getVoltage();
        servoValue = voltReading / 3.25;
        if (servoValue < .25) {
            fRight.setPower(-0.15);
        } else if (servoValue > 0.35) {
            fRight.setPower(0.15);
        } else {
            stop();
        }*/



        //reads voltage value from 0 to 3.2599
        //double voltReading = (float) potentiometer.getVoltage();

        //reads percentage turned
        double currentPer = voltReading/3.25 * 100;
        telemetry.addData("percentage:", currentPer);

        //supposed to read angle turned
        double potDegree = voltReading/3.25 * 270;
        telemetry.addData("degree:", potDegree);




        telemetry.update();
    }

    public void stop(){
        fLeft.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);
    }


   /* public void mervoTest (){

        //reads voltage value from 0 to 3.2599
        double voltReading = (float) potentiometer.getVoltage();
        telemetry.addData("voltage:", voltReading);

        //reads percentage turned
        double currentPer = voltReading/3.25 * 100;
        telemetry.addData("percentage:", currentPer);

        //supposed to read angle turned
        double potDegree = voltReading/3.25 * 270;
        telemetry.addData("degree:", potDegree);

        double servoValue = voltReading/3.25;
        telemetry.addData("servoValue:", servoValue);





    }
*/


}
