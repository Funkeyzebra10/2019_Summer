package org.firstinspires.ftc.teamcode.Reference;

//Imports (duh)
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//@TeleOp(name = "Potentiomeetr")

public class potentiometerTest2 extends OpMode {
    AnalogInput potentiometer;

    public void init() {

        //Mapping
        potentiometer = hardwareMap.analogInput.get("potentiometer");


    }

    //Time to endlessly repeat!
    public void loop() {
        telemetry.addData("potentiometer:", potentiometer.getVoltage() / 3.25);
    }//end of loop


    //Stops all motors
    public void stop() {
    }//end of stop
}//end

