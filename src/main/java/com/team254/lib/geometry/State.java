package com.team254.lib.geometry;

import com.team254.lib.util.Interpolable;

public interface State<S> extends Interpolable<S> {
    public double distance(final S other);

    public boolean equals(final Object other);

    public String toString();

    public String toCSV();
}
