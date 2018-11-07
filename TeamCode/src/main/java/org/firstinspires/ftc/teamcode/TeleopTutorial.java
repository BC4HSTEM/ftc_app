package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

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

@TeleOp(name = "Lift Arm Tests")
public class TeleopTutorial extends LinearOpMode {
    
    private DcMotor armLeft;
    private DcMotor armRight;

    @Override
    public void runOpMode() throws InterruptedException
    {
        armLeft = hardwareMap.dcMotor.get("armLeft");
        
        // Set arm encoders to 0
        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // Zero Power Behavior 
        armLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Setup Telemetry, will not clear after cycle, setup reusable items for output
        telemetry.setAutoClear(false);
        Telemetry.Item armLeftPosition = telemetry.addData("Left Arm Position", armLeft.getCurrentPosition());


        //code before waitForStart is run when Init button is pressed
        while(!isStarted){
          //print encoder counts to telemetry while we manually move the arm
          armLeftPosition.setValue(armLeft.getCurrentPosition());
          telemetry.update();
        }
        
        //code after waitForStart is run when the start button is pressed


        int armTarget = 0;
        double armSpeed = 0.0;
        String armCurrentDirection = "up";
        int maxArmEncoderHeight = 300;
        
        while (isOpModeActive()) {
          
          /**
           * BEGIN ARM LIFT TEST 1
           * using RUN_TO_POSITION
           
           * Gamepad 1 btn A - arm lift up
           * Gamepad 1 btn B - arm lift down
           * 
           * !!! concerned about reliability of encoder count as it doesn't seem to be tracking accurately  
          **/
          
          if (gamepad1.a){ // Arm UP

            armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armTarget = 200;
            armSpeed = 0.98; 
            armCurrentDirection = 'up'; 
            
            armLeft.setPower(armSpeed);
            armLeft.setTargetPosition(armTarget);
            
          } else if (gamepad1.b){ // Arm DOWN
            
            armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armTarget = 0;
            armSpeed = -0.1;  // From my research, negative is ignore, so I don't understand why this *seems* to work  
            armCurrentDirection = 'down'; 
            
            armLeft.setPower(armSpeed);
            armLeft.setTargetPosition(armTarget);
          }




            /**
             * BEGIN ARM LIFT TEST 2
             * using RUN_USING_ENCODER

             * Gamepad 1 btn A - arm lift up
             * Gamepad 1 btn B - arm lift down
             *
             * !!! concerned about setting power to zero at bottom and accurately tracking the down position
             **/
          
          if (gamepad1.dpad_up){ // Arm UP
            
            armLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            
            armTarget = Math.max(0, Math.min(  Math.round( armLeft.getCurrentPosition() + 50 ), maxArmEncoderHeight));
            armSpeed = 0.98; 
            armCurrentDirection = "up"; 
            
            armLeft.setPower(armSpeed);
            armLeft.setTargetPosition(armTarget);
            
          } else if (gamepad1.dpad_down){ // Arm DOWN
            
            armLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            
            armTarget = Math.min(0, Math.round( armLeft.getCurrentPosition() - 50 );
            armSpeed = 0.1;
            armCurrentDirection = "down"; 
            
            armLeft.setPower(armSpeed);
            armLeft.setTargetPosition(armTarget);
          }
          
                  
                  
                            
          /* Remove Power from the Arm Motor if motor is close to 0 position, arm should drop */
          if ( armCurrentDirection === "down" && armLeft.getTargetPosition() < 5 ){
            armSpeed = 0.0;  
            armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          }
          
          /** END ARM LIFT **/
          
          
          idle();
          
          // Arm Lift Telemetry
          if( armLeft.isBusy() ){
            armLeftPosition.setValue(armLeft.getCurrentPosition());
            telemetry.update();
          }
          
        }
    }
}
