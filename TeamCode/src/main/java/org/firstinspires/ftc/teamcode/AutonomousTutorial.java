package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
/*Autonomous Code:
This code is assuming that the left motor is mounted backwards. If the right motor is mounted
backwards, commend out the line motorLeft.setDirection(DcMotor.Direction.REVERSE); and un-commend
the line motorRight.setDirection(DcMotor.Direction.REVERSE);
*/
@Autonomous(name = "Auto01", group = "Autonomous")
public class AutonomousTutorial extends LinearOpMode {

    private DcMotor motorLeft;
    private DcMotor motorRight;
    private DcMotor armLeft;
    private DcMotor armRight;

    private Servo armServo;

    //Static Variables

    private static final double ARM_RETRACTED_POSITION = 0.2;
    private static final double ARM_EXTENDED_POSITION = 0.8;
    private static final int DRIVE_ENCODER_CLICKS = 2240;
    private static final int ARM_ENCODER_CLICKS = 2240;
    private static final double ROBOT_DIAM = 10.2; // Robot diameter in cm
    private static final double ROBOT_CIRC = ROBOT_DIAM * Math.PI;
    private static final double WHEEL_DIAM = 3.7; //Wheel diameter in cm
    private static final double WHEEL_CIRC = WHEEL_DIAM * Math.PI;
    private static final double CLICKS_PER_CM = DRIVE_ENCODER_CLICKS / WHEEL_CIRC;

    @Override
    public void runOpMode() throws InterruptedException
    {
        motorLeft = hardwareMap.dcMotor.get("leftMotor");
        motorRight = hardwareMap.dcMotor.get("rightMotor");
        armLeft = hardwareMap.dcMotor.get("leftArm");
        armRight = hardwareMap.dcMotor.get("rightArm");

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



    public static int getEncoderClicks (double distanceInCM) // Distance in centimeters
     {
        int outputClicks = (int)Math.floor(CLICKS_PER_CM * distanceInCM);

        return outputClicks;
    }

    public void Drive( double distanceInCM, double power)
    {
        /* Entering positive values into distance and power will drive the robot forward. Entering
        negative values will drive the robot backward.
         */
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setPower(power);
        motorRight.setPower(power);
        motorLeft.setTargetPosition(getEncoderClicks(distanceInCM));
        motorRight.setTargetPosition(getEncoderClicks(distanceInCM));
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

    public void TurnLeft( double degrees, double power, boolean turnStyle)
    {
        double turn = ROBOT_CIRC * (degrees / 360);
        if (turnStye == true)
        {
            motorLeft.setDirection(DcMotor.Direction.FOREWARD);
            motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorRight.setPower(power);
            motorRight.setTargetPosition(getEncoderClicks(turn));
            motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLeft.setPower(power);
            motorLeft.setTargetPosition(getEncoderClicks(turn));

        } else {
            motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorRight.setPower(power);
            motorRight.setTargetPosition(getEncoderClicks(turn));
        }
        motorLeft.setPower(0.0);
        motorRight.setPower(0.0);



        while (opModeIsActive() && motorRight.isBusy())
        {
            telemetry.addData("encoder-turn-left" , motorRight.getCurrentPosition() + "busy" + motorRight.isBusy());
            telemetry.update();
            idle();
        }

    }

    public void TurnRight(double degrees, double power, boolean turnStyle) {


        double turn = ROBOT_CIRC * (degrees / 360);
        if (turnStye == true) {
            motorRight.setDirection(DcMotor.Direction.REVERSE);
            motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorRight.setPower(power);
            motorRight.setTargetPosition(getEncoderClicks(turn));
            motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLeft.setPower(power);
            motorLeft.setTargetPosition(getEncoderClicks(turn));
        } else{
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setPower(power);
        motorLeft.setTargetPosition(getEncoderClicks(turn));}

        motorLeft.setPower(0.0);
        motorRight.setPower(0.0);

        while (opModeIsActive() && motorLeft.isBusy())
        {
            telemetry.addData("encoder-turn-right" , motorLeft.getCurrentPosition() + "busy" + motorLeft.isBusy());
            telemetry.update();
            idle();
        }

    }



//Adding a comment to test commit from Windows CL



}
