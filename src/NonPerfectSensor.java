/**
 * 
 */


import lejos.robotics.RangeReadings;
import rp.robotics.localisation.GridPositionDistribution;
import rp.robotics.localisation.SensorModel;
import rp.robotics.mapping.Heading;

/**
 * Empty sensor model for testing.
 * 
 * @author Nick Hawes
 *
 */
public class NonPerfectSensor implements SensorModel {

	float heading;
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
		double a;
		double b;
		float heading = _heading.toDegrees(_heading);
		for (int y = 0; y < _dist.getGridHeight(); y++) {

			for (int x = 0; x < _dist.getGridWidth(); x++) {

				
				
				 a = _dist.getGridMap().rangeToObstacleFromGridPosition(x, y, heading);
				 b = _readings.getRange(0);
			
				
				 System.out.println(b);
				
				
				if (b == -1.0 || b == -1){ b = 150 ;}
		
				
				double pdif = (Math.abs(a-b) / (Math.abs (0.5*(a+b))) );
				// percentage difference between sensor reading and linemap distance is calculated returning a value between 0 and 2
				// that value is divided by 2 to make it between 0 and 1.
				// the greater the percentage difference the lower the probability 
				// so we use 1- this value to set the new probability
				// the actual probability is found by multiplying this new probability with the previous one
				int toX = x;
			
				int toY = y;
float initialprob = _dist.getProbability(toX, toY);

				_dist.setProbability(toX, toY, (float)((1- (pdif/2))*initialprob));
				
				
		
	}

}
		// normalise and return
		_dist.normalise();
		return _dist;
	}
		
}


