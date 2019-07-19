package org.firstinspires.ftc.teamcode.JimmyFuture50thPresident;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "bRight")
public class SwerveTest extends OpMode {
   Servo bRight;
   Servo bLeft;
   Servo fRight;
   Servo fLeft;
    public void init (){
        bRight =hardwareMap.servo.get("bRight");
        bLeft =hardwareMap.servo.get("bLeft");
        fRight =hardwareMap.servo.get("fRight");
        fLeft =hardwareMap.servo.get("fLeft");
        bRight.setPosition(1);
        bLeft.setPosition(1);
        fRight.setPosition(1);
        fLeft.setPosition(1);
    }
    public void loop (){
        boolean b = gamepad1.b;
        float rt = gamepad1.right_trigger;
        telemetry.addData("rightTrigger",rt);
        if (rt>0.5) {
            bRight.setPosition(1);
            bLeft.setPosition(1);
            fRight.setPosition(0);
            fLeft.setPosition(0);
        }
        else if (rt<0.5 && rt>0.1){
            fRight.setPosition(1);
            fLeft.setPosition(1);
            bRight.setPosition(0);
            bLeft.setPosition(0);
        }
        else if (b){
            bRight.setPosition(0.5);
            bLeft.setPosition(0.5);
            fRight.setPosition(0.5);
            fLeft.setPosition(0.5);
        }
    }
    public void stop (){

    }

}
