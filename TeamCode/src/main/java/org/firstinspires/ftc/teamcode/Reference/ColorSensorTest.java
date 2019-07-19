/*package org.firstinspires.ftc.teamcode.Reference;

//@Autonomous (name = "ColorSensorTest")

public class ColorSensorTest extends SupportFileRoverRuckusArchive {

    //Declaring color sensor
    ColorSensor cSensor;
    DistanceSensor dSensor;

    public void init() {

        //Hardware mapping
        cSensor = hardwareMap.get(ColorSensor.class, "cdSensor");
        dSensor = hardwareMap.get(DistanceSensor.class, "cdSensor");
    }

    public void loop() {

        //Telemetry
        telemetry.addData("Blue: ", cSensor.blue());
        telemetry.addData("Green: ", cSensor.green());
        telemetry.addData("Red: ", cSensor.red());

    }

    public void stop() {

    }

    public void tooCloseToBlock()
    {
        double distance = dSensor.getDistance(DistanceUnit.MM);
        if (distance < 5)
        {
            fLeft.setPower(0);
            fRight.setPower(0);
            bLeft.setPower(0);
            bRight.setPower(0);
        }
    }

    public void senseForBlock(int red, int green, int blue)
    {
        if (cSensor.red() > 0 && cSensor.green() > 0 && cSensor.blue() > 0)
        {
            driveStraight("Forward", 1);
        }
    }

    public void Compare2ColorsandDrive(String color1, String color2, int distance)
    {

        int colorOne = 0;
        int colorTwo = 0;

        if(color1 == "Green") {
            colorOne = cSensor.green();
        }
        else if (color1 == "Red")
        {
            colorOne = cSensor.red();
        }
        else if (color1 == "Blue")
        {
            colorOne = cSensor.blue();
        }

        if(color2 == "Green") {
            colorTwo = cSensor.green();
        }
        else if (color2 == "Red")
        {
            colorTwo = cSensor.red();
        }
        else if (color2 == "Blue")
        {
            colorTwo = cSensor.blue();
        }
        else
        {
            stop();
        }


        while (colorOne < colorTwo)
        {
            if(color1 == "Green") {
                colorOne = cSensor.green();
            }
            else if (color1 == "Red")
            {
                colorOne = cSensor.red();
            }
            else if (color1 == "Blue")
            {
                colorOne = cSensor.blue();
            }

            if(color2 == "Green") {
                colorTwo = cSensor.green();
            }
            else if (color2 == "Red")
            {
                colorTwo = cSensor.red();
            }
            else if (color2 == "Blue")
            {
                colorTwo = cSensor.blue();
            }
            else
            {
                stop();
            }
            if (colorOne > colorTwo)
            {
                driveStraight("Forward", distance);
            }
        }
        if (colorOne > colorTwo)
        {
            driveStraight("Forward", distance);
        }
    }

    //If color1 > color2 && color2 > color3
    public void Compare3ColorsandDrive(String color1, String color2, String color3, int distance)
    {

        int colorOne = 0;
        int colorTwo = 0;
        int colorThree = 0;

        if(color1 == "Green") {
            colorOne = cSensor.green();
        }
        else if (color1 == "Red")
        {
            colorOne = cSensor.red();
        }
        else if (color1 == "Blue")
        {
            colorOne = cSensor.blue();
        }

        if(color2 == "Green") {
            colorTwo = cSensor.green();
        }
        else if (color2 == "Red")
        {
            colorTwo = cSensor.red();
        }
        else if (color2 == "Blue")
        {
            colorTwo = cSensor.blue();
        }
        else
        {
            stop();
        }

        if(color3 == "Green") {
            colorThree = cSensor.green();
        }
        else if (color3 == "Red")
        {
            colorThree = cSensor.red();
        }
        else if (color3 == "Blue")
        {
            colorThree = cSensor.blue();
        }
        else
        {
            stop();
        }


        while (colorOne > colorTwo && colorTwo > colorThree)
        {
            if(color1 == "Green") {
                colorOne = cSensor.green();
            }
            else if (color1 == "Red")
            {
                colorOne = cSensor.red();
            }
            else if (color1 == "Blue")
            {
                colorOne = cSensor.blue();
            }

            if(color2 == "Green") {
                colorTwo = cSensor.green();
            }
            else if (color2 == "Red")
            {
                colorTwo = cSensor.red();
            }
            else if (color2 == "Blue")
            {
                colorTwo = cSensor.blue();
            }
            else
            {
                stop();
            }
            if (colorOne > colorTwo)
            {
                driveStraight("Forward", distance);
            }
        }
        if (colorOne > colorTwo)
        {
            driveStraight("Forward", distance);
        }
    }

}
        */