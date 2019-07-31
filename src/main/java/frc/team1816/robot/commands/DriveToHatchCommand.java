package frc.team1816.robot.commands;

import com.edinarobotics.utils.math.Math1816;

import edu.wpi.first.wpilibj.command.Command;
import frc.team1816.robot.Components;
import frc.team1816.robot.Robot;
import frc.team1816.robot.subsystems.Drivetrain;

public class DriveToHatchCommand extends Command {
    
    private Drivetrain drivetrain;

    private static final double kP = 0.0015;
    private static final double ERROR_THRESHOLD = 0;
    private static final double ON_TARGET_THRESHOLD = 20;

    private double nominalPower;
    private double targetCenterX = 320.0;

    private double lateralError;
    private double width;
    private double height;
    private double xCoord;

    public DriveToHatchCommand(double power) {
        drivetrain = Components.getInstance().drivetrain;
        nominalPower = power;
        requires(drivetrain);
    }

    @Override
    protected void initialize() {
        width = Robot.stateInstance.getVisionWidth();
        height = Robot.stateInstance.getVisionWidth();
        xCoord = Robot.stateInstance.getVisionXCoord();
    }

    @Override
    protected void execute() {
      updateCoordData();
      lateralError = targetCenterX - xCoord;
      double leftPow = nominalPower;
      double rightPow = nominalPower;
      double control = Math.abs(lateralError * kP);

      StringBuilder sb = new StringBuilder("cam: (");
      sb.append(width).append("x").append(height).append(")\tcenter X: ").append(xCoord)
              .append("\tlatErr: ").append(lateralError).append("\tcontrol: ").append(control);

      System.out.println(sb.toString());

      if (xCoord == -1.0) {
          control = 0;
      }

      if (Math.abs(lateralError) >= ERROR_THRESHOLD) {
          if (lateralError < 0) { // target is right of center, so decrease right side (wrt cargo) vel
              leftPow = control; // drivetrain reversed, so apply control to other side
              leftPow = Math1816.coerceValue(1.0, 0.15, leftPow);
              rightPow = -leftPow;
          } else { // target is left of center, so decrease left side (wrt cargo) vel
              rightPow = control; // drivetrain reversed, so apply control to other side
              rightPow = Math1816.coerceValue(1.0, 0.15, rightPow);
              leftPow = -rightPow;
          }
      }

      System.out.println("L set pow: " + leftPow + "\tR set pow: " + rightPow);
      drivetrain.setDrivetrain(leftPow, rightPow);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        drivetrain.setDrivetrain(0, 0);
    }

    @Override
    protected void interrupted() {
        end();
    }

    private void updateCoordData() {
        xCoord = Robot.stateInstance.getVisionXCoord();
  }
}
