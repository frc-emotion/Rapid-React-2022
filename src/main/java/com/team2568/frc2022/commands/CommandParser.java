package com.team2568.frc2022.commands;

import java.util.Map;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.team2568.frc2022.Registers;
import com.team2568.frc2022.registers.Register;

public class CommandParser {
    private Command command;

    public enum CommandOptions {
        noop, jump, set
    }

    @JsonCreator
    public CommandParser(@JsonProperty("command") String command, @JsonProperty("params") Map<String, String> params) {
        switch (CommandOptions.valueOf(command)) {
            case noop:
                this.command = new NOOP(Integer.valueOf(params.get("amount")));
                break;
            case jump:
                this.command = new Jump(Integer.valueOf(params.get("pc")));
                break;
            case set:
                Register<?> register = Registers.kRegisters.get(Integer.valueOf(params.get("register")));
                this.command = register.getSetter(params.get("value"));
                break;
        }
    }

    public Command getCommand() {
        return command;
    }
}