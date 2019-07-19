package org.firstinspires.ftc.teamcode.Reference;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by emory on 8/19/2017.
 */

//@TeleOp(name = "NewREVTest")

public class NewREVKit extends OpMode{

    //defining devices
    DcMotor left, right;



    public void init(){

        //initializing devices
        left = hardwareMap.dcMotor.get("lMotor");
        right = hardwareMap.dcMotor.get("rMotor");

        //setting motor direction
        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.REVERSE);

    }//end of init

    public void loop(){

        //assigning functions to controllers
        double lDrive = gamepad1.left_stick_y;
        double rDrive= gamepad1.right_stick_y;

        //for joystick control
        left.setPower((lDrive*100)/128);
        right.setPower((rDrive*100)/128);



    }//end of loop

    public void stop(){

        //stopping all moving devices
        left.setPower(0);
        right.setPower(0);

    }//end of stop

}//end of OpMode
