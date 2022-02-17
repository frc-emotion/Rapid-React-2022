package com.team2568.frc2022.registers;

import com.team2568.frc2022.states.ClimbState;
import com.team2568.frc2022.states.DriveState;
import com.team2568.frc2022.states.IntakeState;
import com.team2568.frc2022.states.ShooterState;

public interface StringToValue<T> {
    public T convert(String s);

    public static StringToValue<String> kString = new StringToValue<String>() {
        @Override
        public String convert(String s) {
            return s;
        }
    };

    public static StringToValue<Integer> kInteger = new StringToValue<Integer>() {
        @Override
        public Integer convert(String s) {
            return Integer.valueOf(s);
        }
    };

    public static StringToValue<Double> kDouble = new StringToValue<Double>() {
        @Override
        public Double convert(String s) {
            return Double.valueOf(s);
        }
    };

    public static StringToValue<Boolean> kBoolean = new StringToValue<Boolean>() {
        @Override
        public Boolean convert(String s) {
            return Boolean.valueOf(s);
        }
    };

    public static StringToValue<ShooterState> kShooterState = new StringToValue<ShooterState>() {
        @Override
        public ShooterState convert(String s) {
            return ShooterState.valueOf(s);
        }
    };

    public static StringToValue<IntakeState> kIntakeState = new StringToValue<IntakeState>() {
        @Override
        public IntakeState convert(String s) {
            return IntakeState.valueOf(s);
        }
    };

    public static StringToValue<ClimbState> kClimbState = new StringToValue<ClimbState>() {
        @Override
        public ClimbState convert(String s) {
            return ClimbState.valueOf(s);
        }
    };

    public static StringToValue<DriveState> kDriveState = new StringToValue<DriveState>() {
        @Override
        public DriveState convert(String s) {
            return DriveState.valueOf(s);
        }
    };
}
