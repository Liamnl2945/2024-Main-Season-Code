package lib.geometry;

import lib.util.Interpolable;
import lib.util.CSVWritable;


public interface State<S> extends Interpolable<S>, CSVWritable {
    double distance(final S other);

    boolean equals(final Object other);

    String toString();

    String toCSV();
}
