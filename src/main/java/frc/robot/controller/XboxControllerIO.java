package frc.robot.controller;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxControllerIO implements CommandControllerIO {
    private final CommandXboxController controller;

    public XboxControllerIO(final int port){
        controller = new CommandXboxController(port);
    }

    @Override
    public double getDriveXAxis(){
        return controller.getLeftX();
    }
    @Override
    public double getDriveYAxis(){
        return controller.getLeftY();
    }
    @Override
    public double getTurnAxis(){
        return controller.getRightX();
    }

    @Override
    public Trigger lockGyroBtn(){
        return controller.a();
    }
    @Override
    public Trigger resetGyroBtn(){
        return controller.b();
    }
    @Override
    public Trigger stopXBtn(){
        return controller.x();
    }
}
