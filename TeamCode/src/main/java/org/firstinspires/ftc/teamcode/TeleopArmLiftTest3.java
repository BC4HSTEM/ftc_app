package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/* ARM LIFT TEST : using RUN_WITH_ENCODER
  ----------------------------------------------------------

  HD Hex 40:1 Specs -> RevRobotics40HdHexMotor
  @MotorType(ticksPerRev=2240, gearing=20, maxRPM=150, orientation=Rotation.CCW)

  https://ftcforum.usfirst.org/forum/ftc-technology/android-studio/51820-problem-with-using-encoders-run-to-position
  "Since you are using RunToPosition, you are using two elements of the built in motor control algorithm. That is "Closed loop velocity" and "Closed loop position".
  ...
  Closed loop velocity will regulate the speed at which the arm moves, adjusting to varying loads.
  Closed loop Position will attempt to regulate the approach velocity to make a smooth approach to the position."


*/

@TeleOp(name = "Arm Test: Run Using Encoder and Position")
public class TeleopArmLiftTest3 extends LinearOpMode {

    private DcMotor armLeft;
    // private DcMotor armRight;

    @Override
    public void runOpMode() throws InterruptedException
    {
        armLeft = hardwareMap.dcMotor.get("armLeft");

        // Set arm encoders to 0, set mode and Zero Power Behavior
        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Setup Telemetry, will not clear after cycle, setup reusable items for output
        telemetry.setAutoClear(false);
        Telemetry.Item armLeftPosition = telemetry.addData("Left Arm Position", armLeft.getCurrentPosition() );

        // ARM LIFT
        // int ARM_ENCODER_COUNT  = 2400;
        // int ARM_TURN_ANGLE = 240;
        // int MAX_ARM_ENCODER_COUNT = Math.round( ARM_ENCODER_COUNT * (ARM_TURN_ANGLE/360) );

        int MAX_ARM_ENCODER_COUNT = 330;
        String armCurrentDirection = "up";


        // Execute Code Before waitForStart is run and after Init button is pressed
        while( !isStarted() ){
          //print encoder counts to telemetry while we manually move the arm
          armLeftPosition.setValue(armLeft.getCurrentPosition());

          telemetry.update();
        }

        // Code after waitForStart is run when the start button is pressed


        while ( opModeIsActive() ) {


            /**
             * BEGIN ARM LIFT TEST 2
             * using RUN_USING_ENCODER

             * Gamepad 1 dpad up- arm lift up
             * Gamepad 1 dpad down - arm lift down
             *
             * !!! concerned about setting power to zero at bottom and accurately tracking the down position
             *
             * **/

            // Arm UP
            if (gamepad1.dpad_up){
                if(!armLeft.isBusy()) {
                    armCurrentDirection = "up";
                    armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armLeft.setTargetPosition(MAX_ARM_ENCODER_COUNT);
                    armLeft.setPower(1); // Positive power pushes arm up
                }

            // Arm DOWN
            } else if (gamepad1.dpad_down){
                if( !armLeft.isBusy() ) {
                    armCurrentDirection = "down";
                    armLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    armLeft.setPower(0.5); // Positive power should apply resistance to gravity but allow arm to drop
                }
            }


            // Arm Drop, ease braking power
            if (armCurrentDirection.equals("down") && armLeft.isBusy() ){

                if( armLeft.getCurrentPosition() <= 10 ) {
                    // Stop power to arm and reset encoders, arm drops, 10 example margin of error?
                    armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                } else if(armLeft.getCurrentPosition() <= Math.round(MAX_ARM_ENCODER_COUNT * 0.3337) ){
                    // Lower the resistance to gravity at 1/3 descent
                    armLeft.setPower(0.1);

                } else if(armLeft.getCurrentPosition() <= Math.round(MAX_ARM_ENCODER_COUNT * 0.6667) ){
                    // Lower the resistance to gravity at 2/3 descent
                    armLeft.setPower(0.25);

                }

            } else if (armCurrentDirection.equals("up") && armLeft.isBusy() ){

                // we need nearly 100% power to keep the arm up, test further
                if( armLeft.getCurrentPosition() >= 300 ) {
                    armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armLeft.setTargetPosition(MAX_ARM_ENCODER_COUNT);
                    armLeft.setPower(0.95);
                }

            }


            // END ARM LIFT


            idle();

            // Arm Lift Telemetry
            if( armLeft.isBusy() ){
                armLeftPosition.setValue(armLeft.getCurrentPosition());
                telemetry.update();
            }

        }
    }
}
