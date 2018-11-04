package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
/*Autonompus Code:
This code is assuming that the left motor is mounted backwards. If the right motor is mounted
backwards, commend out the line motorLeft.setDirection(DcMotor.Direction.REVERSE); and un-commend
the line motorRight.setDirection(DcMotor.Direction.REVERSE);
*/
@Autonomous(name = "Tele-Op Tutorial", group = "Tutorials")
public class AutonomousTutorial extends LinearOpMode {

    private DcMotor motorLeft;
    private DcMotor motorRight;

    private Servo armServo;

    private static final double ARM_RETRACTED_POSITION = 0.2;
    private static final double ARM_EXTENDED_POSITION = 0.8;
    private static final int ENCODER_CLICKS = 2240;

    @Override
    public void runOpMode() throws InterruptedException
    {
        motorLeft = hardwareMap.dcMotor.get("leftMotor");
        motorRight = hardwareMap.dcMotor.get("rightMotor");

        //one of the motors will be mounted backwards, so it should be reversed.
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        //motorRight.setDirection(DcMotor.Direction.REVERSE);


        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        armServo = hardwareMap.servo.get("armServo");

        armServo.setPosition(ARM_RETRACTED_POSITION);

        //code before waitForStart is run when Init button is pressed
        waitForStart();
        //code after waitForStart is run when the start button is pressed
        Drive(-30, -0.8);
    }

    public static int getEncoderClicks (double distance) // Distance in centimeters
     {
        //Wheel diameter in centimeters,
        double wheelDiam = 10.0;
        double wheelCircum = Math.PI * wheelDiam;
        double clicksPerCm = ENCODER_CLICKS / wheelCircum;
        int outputClicks = (int)Math.floor(clicksPerCm * distance);

        return outputClicks;
    }

    public void Drive( double distance, double power)
    {
        /* Entering positive values into distance and power will drive the robot forward. Entering
        negative values will drive the robot backward.
         */
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setPower(power);
        motorRight.setPower(power);
        motorLeft.setTargetPosition(getEncoderClicks(distance));
        motorRight.setTargetPosition(getEncoderClicks(distance));
        motorRight.setPower(0.0);
        motorLeft.setPower(0.0);

        while (opModeIsActive() && motorLeft.isBusy() && motorRight.isBusy())
        {
            telemetry.addData("encoder-fwd" , motorLeft.getCurrentPosition()
                    + "busy" + motorLeft.isBusy() + motorRight.getCurrentPosition()
                    + "busy" + motorRight.isBusy());
            telemetry.update();
            idle();
        }


    }

    public void TurnLeft( double degrees, double power)
    {
        double robotDiam = 10.0;
        double robotCirc = robotDiam * Math.PI;
        double turn = robotCirc * (degrees / 360);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setPower(power);
        motorRight.setTargetPosition(getEncoderClicks(turn));
        motorRight.setPower(0.0);

        while (opModeIsActive() && motorRight.isBusy())
        {
            telemetry.addData("encoder-turn" , motorRight.getCurrentPosition() + "busy" + motorRight.isBusy());
            telemetry.update();
            idle();
        }

    }

    public void TurnRight(double degrees, double power) {

        double robotDiam = 10.0;
        double robotCirc = robotDiam * Math.PI;
        double turn = robotCirc * (degrees / 360);
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setPower(power);
        motorLeft.setTargetPosition(getEncoderClicks(turn));
        motorLeft.setPower(0.0);

        while (opModeIsActive() && motorLeft.isBusy())
        {
            telemetry.addData("encoder-turn" , motorLeft.getCurrentPosition() + "busy" + motorLeft.isBusy());
            telemetry.update();
            idle();
        }

    }



}
