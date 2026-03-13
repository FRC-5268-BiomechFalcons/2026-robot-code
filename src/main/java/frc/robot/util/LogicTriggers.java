package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.Trigger;


public class LogicTriggers {

    // returns a trigger of input1 as True if input2 is False
    public static Trigger without(Trigger input1, Trigger input2) {
        return input1.and(input2.negate());
    }
}
