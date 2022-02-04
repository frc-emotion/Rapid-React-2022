package com.team2568.frc2022.commands;

public class JumpIfEqual extends Jump {
    private Compare<?> compare;

    public JumpIfEqual(Compare<?> compare, int counter) {
        super(counter);
        this.compare = compare;
    }

    @Override
    public int execute() {
        if (compare.isEqual()) {
            return super.execute();
        }
        return -1;
    }
}
