package org.firstinspires.ftc.teamcode.HelloItsAdam;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;



//FIXME hi
@TeleOp(name="swerve psoitions")
public class SwervePositions extends OpMode {
    //DC Motors
    DcMotor fLeft, fRight, bLeft, bRight;
    float left,right;
    //Servos
    Servo sFLeft, sFRight, sBLeft, sBRight;
    public void init(){
        //Initializing/mapping motors and servos
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");

        sFLeft = hardwareMap.servo.get("sFLeft");
        sFRight = hardwareMap.servo.get("sFRight");
        sBLeft = hardwareMap.servo.get("sBLeft");
        sBRight = hardwareMap.servo.get("sBRight");

        //Directions
        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);

        //Directions
        sFLeft.setDirection(Servo.Direction.FORWARD);
        sFRight.setDirection(Servo.Direction.FORWARD);
        sFLeft.setDirection(Servo.Direction.FORWARD);;
        sFRight.setDirection(Servo.Direction.FORWARD);

        sFLeft.setPosition(0.5);
        sFRight.setPosition(0.5);
        sBLeft.setPosition(0.5);
        sBRight.setPosition(0.5);

    }
    public void loop(){
        left=gamepad1.left_stick_y;
        right=gamepad1.right_stick_y;


    }
    public void stop(){

    }
}
