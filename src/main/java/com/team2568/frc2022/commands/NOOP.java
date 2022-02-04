package com.team2568.frc2022.commands;

public class NOOP implements Command {
    private int amount;
    private int current;

    public NOOP(int amount) {
        this.amount = amount;
        this.current = amount;
    }

    @Override
    public int execute() {
        if (current > 0) {
            current--;
            return -2;
        }
        current = amount;
        return -1;
    }
}
    