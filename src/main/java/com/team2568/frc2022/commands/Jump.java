package com.team2568.frc2022.commands;

public class Jump implements Command {
    private int counter;

    public Jump(int counter) {
        this.counter = counter;
    }

    @Override
    public int execute() {
        return counter;
    }
}
