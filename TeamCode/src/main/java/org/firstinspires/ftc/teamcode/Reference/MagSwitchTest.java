package org.firstinspires.ftc.teamcode.Reference;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

/**
 * Created by emory on 8/15/2018.
 */
//@TeleOp(name = "MagSwitchTest")

public class MagSwitchTest extends OpMode {
    //Defining motors
    DcMotor bRight;
    DcMotor bLeft;
    DcMotor fRight;
    DcMotor fLeft;

    //Defining functions
    float lift;

    //Defining magSwitch
    DigitalChannel magSwitch;




    public void init() {

        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");
        magSwitch = hardwareMap.digitalChannel.get("magSwitch");

        fLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        fRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotor.Direction.REVERSE);
        bLeft.setDirection(DcMotor.Direction.FORWARD);



    }

    public void loop(){

       boolean switchState = magSwitch.getState();
       telemetry.addData("switch:", switchState);

       if (switchState == true){
           lift = gamepad1.left_stick_y;
           fRight.setPower(lift);
       }
       else if (switchState == false){
           fRight.setPower(-lift);

           telemetry.addLine("at limit");
        }



        telemetry.update();
    }

    public void stop(){
        fLeft.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);
    }


}
