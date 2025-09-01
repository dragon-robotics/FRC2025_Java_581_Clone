package frc.robot.subsystems.intake;

public interface IntakeIO {
    default void setIntakeArmMotorVoltage(double voltage) {
        throw new UnsupportedOperationException("setIntakeArmMotorVoltage() not implemented");
    }

    default void setIntakeArmMotorPercentage(double percentage) {
        throw new UnsupportedOperationException("setIntakeArmMotorPercentage() not implemented");
    }

    default void setIntakeWheelMotorVoltage(double voltage) {
        throw new UnsupportedOperationException("setIntakeWheelMotorVoltage() not implemented");
    }

    default void setIntakeWheelMotorPercentage(double percentage) {
        throw new UnsupportedOperationException("setIntakeWheelMotorPercentage() not implemented");
    }
}
