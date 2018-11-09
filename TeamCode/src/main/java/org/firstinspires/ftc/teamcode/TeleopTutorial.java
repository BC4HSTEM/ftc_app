package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/* ARM LIFT ALTERNATE : UNTESTED & NOT BUILT
  ----------------------------------------------------------
  
  HD Hex 40:1 Specs -> RevRobotics40HdHexMotor 
  @MotorType(ticksPerRev=2240, gearing=20, maxRPM=150, orientation=Rotation.CCW)
  
  !!! We should check all wiring. Motors don't seem to be responding according to correct encoder counts?
  
  Turn Arm motors off when the "down" position has been reached. 
  This removes power from the motors and must only be applied near the bottom of the drop
  
  A few resources and notes: 
  http://stemrobotics.cs.pdx.edu/node/4745
  
  https://ftcforum.usfirst.org/forum/ftc-technology/android-studio/6443-getting-motors-to-hold-their-position
  These guys are using negative power and negative encoder counts to step down??
  
  https://ftcforum.usfirst.org/forum/ftc-technology/android-studio/51820-problem-with-using-encoders-run-to-position
  "Since you are using RunToPosition, you are using two elements of the built in motor control algorithm. That is "Closed loop velocity" and "Closed loop position".
  ...
  Closed loop velocity will regulate the speed at which the arm moves, adjusting to varying loads.
  Closed loop Position will attempt to regulate the approach velocity to make a smooth approach to the position."

  
*/

@TeleOp(name = "Arm Lift Test: faux PID")
public class TeleopTutorial extends LinearOpMode {
    
    private DcMotor armLeft;
    // private DcMotor armRight;

    @Override
    public void runOpMode() throws InterruptedException
    {
        armLeft = hardwareMap.dcMotor.get("armLeft");
        
        // Set arm encoders to 0, set mode and Zero Power Behavior
        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Setup Telemetry, will not clear after cycle, setup reusable items for output
        telemetry.setAutoClear(false);
        Telemetry.Item armLeftPosition = telemetry.addData("Left Arm Position", armLeft.getCurrentPosition() );

        // ARM LIFT
        int ARM_ENCODER_COUNT  = 2400;
        int ARM_TURN_ANGLE = 240;
        int MAX_ARM_ENCODER_COUNT = Math.round( ARM_ENCODER_COUNT * (ARM_TURN_ANGLE/360) );
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
             * using RUN_WITHOUT_ENCODER

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
                    armLeft.setPower(1); // Positive power pushes arm up
                }

            // Arm DOWN
            } else if (gamepad1.dpad_down){
                if(!armLeft.isBusy()) {
                    armCurrentDirection = "down";
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

            } else if (armCurrentDirection.equals("down") && armLeft.isBusy() ){

                // we need nearly 100% power to keep the arm up, test further
                if( armLeft.getCurrentPosition() >= MAX_ARM_ENCODER_COUNT ) {
                    armLeft.setPower(0.9);
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
