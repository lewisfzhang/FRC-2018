package com.team254.lib.geometry;

import com.team254.lib.util.Interpolable;

public interface State<S> extends Interpolable<S> {
    public double distance(final S other);

    public boolean isEqual(final S other);

    public String toString();

    public String toCSV();
}
