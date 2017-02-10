package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.sql.Time;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//gamepad1 is user1's gamepad. They control driving and turning the sweeper on and off
//gamepad2 is user2's gamepad. They control shooting the ball

//DO NOT CHANGE THIS
@TeleOp(name="Karel", group="Pushbot")
//DO NOT UN COMMENT
//@Disabled
public class Karel extends OpMode{

    /* Declare OpMode members. */
    KarelHardware robot = new KarelHardware(); // use the class created to define the robot's hardware


    ElapsedTime runtime = new ElapsedTime(); //creates a new timer

    private final double TWEEN = .1; //the amount of seconds that we will move the arm



    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap); //initialize hardware map (what the things are called)


        // Send telemetry message to signify robot waiting;
        //telemetry is the little things at the bottom of the driver station
        //the first part is the title of the line, the second part is the little thing you want to say
        //("Position", currentPos) is would equate to >Position: (currentPos)[replaced with the
        telemetry.addData("Say", "Hello Driver");
        updateTelemetry(telemetry);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */



    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */



    @Override
    public void loop() {
        double left;
        double right;

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)

        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;

        //if left bumper is held, you can steer more precisely
        if(gamepad1.left_bumper){
            robot.leftMotor.setPower(left * .25); //sets the power to 10% of how much the person is pushing
            robot.rightMotor.setPower(right * .25);
        }
        else {
            robot.leftMotor.setPower(left * .75); //sets the power to 75% of how much the person is pushing
            robot.rightMotor.setPower(right * .75);
        }


        //throwing arm
        if(gamepad2.x) { //when the right bumper on the second controller is pressed
            robot.sweeper.setPower(-100);
        } else {
            robot.sweeper.setPower(0);
        }

        //for the sweeper
        if(gamepad2.a) {
            robot.launcher.setPower(100);
        } else if(gamepad2.b) {
            robot.launcher.setPower(-100);
        } else {
            robot.launcher.setPower(0);
        }


        //shows the amount that the person is moving the joysticks
        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);
        //tells the user if the robot is currently sweeping
        //telemetry.addData("sweeping", toggle);

        updateTelemetry(telemetry);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}


