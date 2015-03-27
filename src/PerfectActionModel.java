

import rp.robotics.localisation.ActionModel;
import rp.robotics.localisation.GridPositionDistribution;
import rp.robotics.mapping.Heading;

/**
 * Example structure for an action model that should move the probabilities 1
 * cell in the requested direction. In the case where the move would take the
 * robot into an obstacle or off the map, this model assumes the robot stayed in
 * one place. This is the same as the model presented in Robot Programming
 * lecture on action models.
 * 
 * Note that this class doesn't actually do this, instead it shows you a
 * <b>possible</b> structure for your action model.
 * 
 * @author Nick Hawes
 * 
 */
public class PerfectActionModel implements ActionModel {

	@Override
	public GridPositionDistribution updateAfterMove(
			GridPositionDistribution _from, Heading _heading) {

		// Create the new distribution that will result from applying the action
		// model
		GridPositionDistribution to = new GridPositionDistribution(_from);

		// Move the probability in the correct direction for the action
		move(_from, to, _heading);
// return the new distribution
		return to;
	}

	/**
	 * Move probabilities from _from one cell in the plus x direction into _to
	 * 
	 * @param _from
	 * @param _to
	 *
	 * 
	 */

	private void move(GridPositionDistribution _from,
			GridPositionDistribution _to, Heading _heading) {

		int a = 0, b = 0, c = 0, d = 0;

		// a and b are x and y coordinates of location after move
		// c and d are x and y coordinates of the previous location

		// iterate through points updating as appropriate
		for (int y = 0; y < _to.getGridHeight(); y++) {

			for (int x = 0; x < _to.getGridWidth(); x++) {

				if (_heading == Heading.PLUS_X) {
					a = x + 1;
					b = y;
				} else if (_heading == Heading.PLUS_Y) {
					a = x;
					b = y + 1;
				
				} else if (_heading == Heading.MINUS_X) {
					a = x - 1;
					b = y;

				} else if (_heading == Heading.MINUS_Y) {
					a = x;
					b = y - 1;
				}

				// now assigning previous locations grid positions
				
				if (_heading == Heading.PLUS_X) {
					c = x - 1;
					d = y;
				} else if (_heading == Heading.PLUS_Y) {
					c = x;
					d = y - 1;
					
				} else if (_heading == Heading.MINUS_X) {
					c = x + 1;
					d = y;

				} else if (_heading == Heading.MINUS_Y) {
					c = x;
					d = y + 1;
				}

				// make sure to respect obstructed grid points
// before valid checks if a transition from previous position to the current position is possible
				boolean BeforeValid = (_to.getGridMap().isValidTransition(c, d,
						x, y));
				// after valid checks if a transition from current position to the next position is possible
				boolean AfterValid = (_to.getGridMap().isValidTransition(x, y,
						a, b));
// is obstructed checks whether or not the current position on a grid is inside an obstacle
				boolean isObstructed = (_to.getGridMap().isObstructed(x, y));
				if (!AfterValid) {
// if !aftervalid we should get the probability from previous location and accumulate it on the current position
					// position before move
					int fromX = c;
					int fromY = d;
					float fromProb = _from.getProbability(fromX, fromY);

					
					int toX = x;
					int toY = y;

					float toProb = _from.getProbability(x, y);

					_to.setProbability(toX, toY, fromProb + toProb);

				}
				if (!BeforeValid && AfterValid) {

					// if it is not possible for the robot to make a transition from previous position to current position
					// we give the current position 0 probability
					int toX = x;
					int toY = y;

					_to.setProbability(toX, toY, 0);

				}

				if (BeforeValid && AfterValid) {
// if it is a middle value,ie has no problem with transitions we simply get the probability from previous position and put it to the current position
					int fromX = c;
					int fromY = d;
					float fromProb = _from.getProbability(fromX, fromY);

					// position after move
					int toX = x;
					int toY = y;

					// set probability for position after move
					_to.setProbability(toX, toY, fromProb);

				}
				if (isObstructed) {
// if the point is obstructed we remove them from our probabilities table because robot cant get inside those positions.
					int toX = x;
					int toY = y;
					_to.setProbability(toX, toY, 0);

				}

			}
		}
		_to.normalise();
	}
}
