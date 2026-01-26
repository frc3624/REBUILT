package frc.robot.util;

import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;

public class ShotModel {
  private final NavigableMap<Double, Double> table = new TreeMap<>();

  public ShotModel() {
    // Put your real data here as you test.
    // distance (meters) -> shooter rpm
    table.put(1.0, 2400.0);
    table.put(1.5, 2650.0);
    table.put(2.0, 2850.0);
    table.put(2.5, 3000.0); // your baseline
    table.put(3.0, 3200.0);
    table.put(3.5, 3450.0);
    table.put(4.0, 3700.0);
  }

  public double rpmForDistance(double dMeters) {
    double minD = table.firstKey();
    double maxD = table.lastKey();
    dMeters = Math.max(minD, Math.min(maxD, dMeters));

    Map.Entry<Double, Double> lo = table.floorEntry(dMeters);
    Map.Entry<Double, Double> hi = table.ceilingEntry(dMeters);

    if (lo == null) return hi.getValue();
    if (hi == null) return lo.getValue();
    if (lo.getKey().equals(hi.getKey())) return lo.getValue();

    double t = (dMeters - lo.getKey()) / (hi.getKey() - lo.getKey());
    return lo.getValue() + t * (hi.getValue() - lo.getValue());
  }
}
