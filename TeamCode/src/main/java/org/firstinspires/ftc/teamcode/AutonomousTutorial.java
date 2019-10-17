package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
/*Autonomous Code:
This code is assuming that the left motor is mounted backwards. If the right motor is mounted
backwards, commend out the line motorLeft.setDirection(DcMotor.Direction.REVERSE); and un-commend
the line motorRight.setDirection(DcMotor.Direction.REVERSE);
*/
@Autonomous(name = "Auto01", group = "Autonomous")
@Disabled

public class AutonomousTutorial extends LinearOpMode {

    private DcMotor motorLeft;
    private DcMotor motorRight;
    private DcMotor armLeft;
    private DcMotor armRight;

    private Servo armServo;
    // Detector object
    private GoldAlignDetector detector;

    //Static Variables

    private static final double ARM_RETRACTED_POSITION = 0.2;
    private static final double ARM_EXTENDED_POSITION = 0.8;
    private static final int DRIVE_ENCODER_CLICKS = 2240;
    private static final int ARM_ENCODER_CLICKS = 2240;
    private static final double ROBOT_DIAM = 40.0; // Robot diameter in cm
    private static final double ROBOT_CIRC = ROBOT_DIAM * Math.PI;
    private static final double WHEEL_DIAM = 10.0; //Wheel diameter in cm
    private static final double WHEEL_CIRC = WHEEL_DIAM * Math.PI;
    private static final double CLICKS_PER_CM = DRIVE_ENCODER_CLICKS / WHEEL_CIRC;
    private static final double DRIVE_GEAR_RATIO = 1/2; // This may need to change

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
        telemetry.addData("Status", "DogeCV 2018.0 - Gold Align Example");

        // Set up detector
        detector = new GoldAlignDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment

        detector.enable(); // Start the detector!

        //code before waitForStart is run when Init button is pressed
        waitForStart();
        //code after waitForStart is run when the start button is pressed

        TurnLeft(50, .5, true);
        Drive(116.84, .5);
        TurnRight(90, .5, true);
        Drive(93.98,.5);
        TurnRight(180, .5, true);
        //trophy arm code
        Drive(92.98, .5);
        TurnLeft(90, .5, true);
        Drive(116.84, .5);
        TurnLeft(90,.5,true);
        Drive(116.84,.5);
        TurnRight(90,.5,true);
        Drive( 106.68,1);


        /* PSEUDOCODE: *********
        The detector has two features we can use just based on sample code: getAligned(0
        returns a boolean, getXPosition(0 returns a number from 0 t0 640 depending on the cube's
        position. So the code should read something like :
        while getAligned is false, getXPosition. Depending on whether it indicates left or right,
        turn the robot slightly to the left or the right. Once getAligned is true, move forward.
        PSEUDOCODE **************
         */

        Drive(-30, -0.8);
    }



    public static int getEncoderClicks (double distanceInCM) // Distance in centimeters
     {
        int outputClicks = (int)Math.floor((CLICKS_PER_CM * distanceInCM)* DRIVE_GEAR_RATIO);

        return outputClicks;
    }

    public void Drive( double distanceInCM, double power) throws InterruptedException {
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

        while(motorLeft.isBusy() && motorRight.isBusy())
        {
            int a=motorRight.getCurrentPosition();
            Thread.sleep(200);
            int b=motorRight.getCurrentPosition();
            if (a == b) {
                motorRight.setPower(0.0);

            }

            int c=motorLeft.getCurrentPosition();
            Thread.sleep(200);
            int d=motorLeft.getCurrentPosition();
            if (c == d) {
                motorLeft.setPower(0.0);

            }
        }


    }

    public void TurnLeft( double degrees, double power, boolean turnStyle)
    {
        double turn = ROBOT_CIRC * (degrees / 360);
        if (turnStyle == true)
        {
            motorLeft.setDirection(DcMotor.Direction.FORWARD);
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
            telemetry.addData("encoder-turn-left" , motorRight.getCurrentPosition()
                    + "busy" + motorRight.isBusy());
            telemetry.update();
            idle();
        }

    }

    public void TurnRight(double degrees, double power, boolean turnStyle) {


        double turn = ROBOT_CIRC * (degrees / 360);
        if (turnStyle == true) {
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

    //-



//Adding a comment to test commit from Windows CL



}
