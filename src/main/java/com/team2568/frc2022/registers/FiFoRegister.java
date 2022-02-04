package com.team2568.frc2022.registers;

import java.util.ArrayList;

/**
 * Stores all set values and pops values when get is called. Throws an exception
 * if the stack is full and set is called.
 */
public class FiFoRegister<T> extends Register<T> {
    private ArrayList<T> stack = new ArrayList<T>();
    private int depth;

    public FiFoRegister(StringToValue<T> parser, int depth) {
        super(parser);
        this.depth = depth;
    }

    @Override
    public void set(T value) throws IndexOutOfBoundsException {
        if (stack.size() < depth) {
            stack.add(value);
        } else {
            throw new IndexOutOfBoundsException("FiFo of depth " + depth + " is full");
        }
    }

    @Override
    public T get() {
        if (stack.size() > 0) {
            return stack.remove(0);
        }
        return null;
    }
}
