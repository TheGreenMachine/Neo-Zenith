package frc.team1816.robot.commands;

import com.edinarobotics.utils.math.Math1816;

import edu.wpi.first.wpilibj.command.Command;
import frc.team1816.robot.Components;
import frc.team1816.robot.Robot;
import frc.team1816.robot.subsystems.Arm;
import frc.team1816.robot.subsystems.Drivetrain;
import frc.team1816.robot.subsystems.Shooter;

public class DriveToHatchCommand extends Command {
    
    private Drivetrain drivetrain;
    private Arm arm;
    private Shooter shooter;

    private static final double kP = 0.0015;
    private static final double ERROR_THRESHOLD = 0;
    private static final double ON_TARGET_THRESHOLD = 20;

    private double nominalPower;
    private double targetCenterX = 320.0;

    private double lateralError;
    private double width;
    private double height;
    private double xCoord;
    private double yCoord = -1.0;
    private double yLastCoord = -1.0;
    private double targetHeight;
    private long time = System.currentTimeMillis();
    private long lastShootTime = time - 10000;

    public DriveToHatchCommand(double power) {
        drivetrain = Components.getInstance().drivetrain;
        arm = Components.getInstance().arm;
        shooter = Components.getInstance().shooter;
        nominalPower = power;
        requires(drivetrain);
    }

    @Override
    protected void initialize() {
        width = Robot.stateInstance.getVisionWidth();
        height = Robot.stateInstance.getVisionWidth();
        xCoord = Robot.stateInstance.getVisionXCoord();
        yCoord = Robot.stateInstance.getVisionYCoord();
        targetHeight = Robot.stateInstance.getTargetHeight();
        yLastCoord = yCoord;
    }

    @Override
    protected void execute() {
        time = System.currentTimeMillis();
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

        /**
         * distance ~ 1 / targetHeight
         * angle = atan(opposite / adjacent) = atan((yCoord - 320) / distance)
         */
        if (targetHeight > 0.0 && yCoord > 0.0) {
            double distanceInches = (600.0 / targetHeight) * 12.0;
            double targetHeightInches = (yCoord);
            double angle = Math.toDegrees(Math.atan2((yCoord - 320.0), distanceInches));
            System.out.printf("**** targetHeight: %6.1f, yCoord: %6.1f, distance: %6.1f, angle: %6.1f\n",
                    targetHeight, yCoord, distanceInches, angle);
        }

        if (yCoord > 0.0 && yLastCoord > 0.0 && (time - lastShootTime > 10000) && 100 < (yCoord - yLastCoord)) {
            shooter.shoot();
            lastShootTime = time;
            System.out.printf("  Shooting: yCoord: %6.0f; lastY: %6.0f, \n", yCoord, yLastCoord);
        }
        yLastCoord = yCoord;
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        drivetrain.setDrivetrain(0, 0);
        yLastCoord = -1.0;
        yCoord = -1.0;
    }

    @Override
    protected void interrupted() {
        end();
    }

    private void updateCoordData() {
        xCoord = Robot.stateInstance.getVisionXCoord();
        yCoord = Robot.stateInstance.getVisionYCoord();
        targetHeight = Robot.stateInstance.getTargetHeight();
  }
}
