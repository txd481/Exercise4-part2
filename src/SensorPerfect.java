/**
 * 
 */


import lejos.robotics.RangeReadings;
import rp.robotics.localisation.GridPositionDistribution;
import rp.robotics.localisation.MyMap;
import rp.robotics.localisation.SensorModel;
import rp.robotics.mapping.Heading;

/**
 * Empty sensor model for testing.
 * 
 * @author Nick Hawes
 *
 */
public class SensorPerfect implements SensorModel {

	/*
	 * (non-Javadoc)
	 * 
	 * @see rp.robotics.localisation.SensorModel#updateAfterSensing(rp.robotics.
	 * localisation.GridPositionDistribution, rp.robotics.mapping.Heading,
	 * lejos.robotics.RangeReadings)
	 */
	@Override
	public GridPositionDistribution updateAfterSensing(
			GridPositionDistribution _dist, Heading _heading,
			RangeReadings _readings) {

		for (int y = 0; y < _dist.getGridHeight(); y++) {

			for (int x = 0; x < _dist.getGridWidth(); x++) {

				// for every point on the grid if the sensor distance is the same as linemap distance we keep the probabilities
				// if they are not the same we remove them
				// when sensor is out of range, we only keep the positions with distances greater than 142 because it is the minimum linemap distance which results in out of range sensor
				MyMap map = (MyMap) _dist.getGridMap();

				float heading = _heading.toDegrees(_heading);
				float prob;
				if (_readings.getRange(0) == map
						.rangeToObstacleFromGridPosition(x, y, heading)
						|| (_readings.getRange(0) == 255 && map
								.rangeToObstacleFromGridPosition(x, y, heading) > 142) ) {
					{
						
						prob = 1;
					}
				} else {
					prob = 0;
				}
				System.out.println(map.rangeToObstacleFromGridPosition(x, y,
						heading) + " " + _readings.getRange(0));

				float initialprob = _dist.getProbability(x, y);
// multipling the initial probability with our new probability either keeps them or removes them
				_dist.setProbability(x, y, prob * initialprob);
// we normalise to make the values add up to 1 and return
				_dist.normalise();

			}
		}

		return _dist;
	}
}
