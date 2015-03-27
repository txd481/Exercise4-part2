package testt;

import java.awt.Point;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;

import lejos.geom.Line;
import lejos.geom.Rectangle;
import lejos.robotics.navigation.Pose;

import org.testng.Assert;
import org.testng.annotations.Test;

import rp.robotics.localisation.ActionModel;
import rp.robotics.localisation.GridPositionDistribution;
import rp.robotics.localisation.MarkovLocalisationSkeleton;
import rp.robotics.localisation.MyMap;
import rp.robotics.localisation.PerfectActionModel;
import rp.robotics.localisation.PerfectSensorModel;
import rp.robotics.localisation.SensorModel;
import rp.robotics.localisation.Zazy;
import rp.robotics.mapping.Heading;
import rp.robotics.mapping.IGridMap;
import rp.robotics.mapping.MapUtils;
import rp.robotics.mapping.NicksGridMap;
import rp.robotics.localisation.MyMap;

import rp.robotics.mapping.RPLineMap;
import rp.robotics.simulation.SimulatedRobot;

public class GridMapTest {

	// PLEASE GO TO THE LAST METHOD
	
	/***
	 * Create an instance of an object that implements IGridMap from a LineMap.
	 * You should copy this test file to your own project and replace
	 * NicksGridMap with you own implementation.
	 * 
	 * @param _lineMap
	 *            The underlying line map
	 * @param _gridXSize
	 *            How many grid positions along the x axis
	 * @param _gridYSize
	 *            How many grid positions along the y axis
	 * @param _xStart
	 *            The x coordinate where grid position (0,0) starts
	 * @param _yStart
	 *            The y coordinate where grid position (0,0) starts
	 * @param _cellSize
	 *            The distance between grid positions
	 * @return
	 */
	public static IGridMap createGridMap(RPLineMap _lineMap, int _gridXSize,
			int _gridYSize, float _xStart, float _yStart, float _cellSize) {
		return new NicksGridMap(_gridXSize, _gridYSize, _xStart, _yStart,
				_cellSize, _lineMap);
	}

	public static IGridMap createRectangularGridMap(int _xJunctions,
			int _yJunctions, float _pointSeparation) {

		int xInset = (int) (_pointSeparation / 2);
		int yInset = (int) (_pointSeparation / 2);

		float _width = _xJunctions * _pointSeparation;
		float _height = _yJunctions * _pointSeparation;

		ArrayList<Line> lines = new ArrayList<Line>();

		// these are the walls for the world outline
		lines.add(new Line(0f, 0f, _width, 0f));
		lines.add(new Line(_width, 0f, _width, _height));
		lines.add(new Line(_width, _height, 0f, _height));
		lines.add(new Line(0f, _height, 0f, 0f));

		Line[] lineArray = new Line[lines.size()];
		lines.toArray(lineArray);

		return createGridMap(new RPLineMap(lineArray, new Rectangle(0, 0,
				_width, _height)), _xJunctions, _yJunctions, xInset, yInset,
				_pointSeparation);
	}

	/**
	 * Creates a grid map to match the training map as of 6/3/2013.
	 * 
	 * @return
	 */
	public static IGridMap createTestMap() {
		float height = 238;
		float width = 366;

		// junction numbers
		int xJunctions = 12;
		int yJunctions = 7;

		// position of 0,0 junction wrt to top left of map
		int xInset = 24;
		int yInset = 24;

		// length of edges between junctions.
		int junctionSeparation = 30;

		ArrayList<Line> lines = new ArrayList<Line>();

		// these are the walls for the world outline
		lines.add(new Line(0f, 0f, width, 0f));
		lines.add(new Line(width, 0f, width, height));
		lines.add(new Line(width, height, 0f, height));
		lines.add(new Line(0f, height, 0f, 0f));

		lines.add(new Line(75.0f, 133f, 100f, 133f));
		lines.add(new Line(75.0f, 193.0f, 100f, 193.0f));
		lines.add(new Line(100f, 133f, 100f, 193.0f));
		lines.add(new Line(75.0f, 133f, 75.0f, 193.0f));

		lines.addAll(MapUtils.lineToBox(42f, 67f, 287f, 67f));
		lines.add(new Line(287f, 0, 287f, 67f));
		lines.add(new Line(257f, 0, 257f, 67f));

		lines.addAll(MapUtils.lineToBox(135f, 129f, 255f, 129f));
		lines.addAll(MapUtils.lineToBox(135f, 129f, 135f, height));

		lines.addAll(MapUtils.lineToBox(194f, 191f, 254f, 191f));
		lines.addAll(MapUtils.lineToBox(194f, 191f, 194f, height));

		lines.add(new Line(width - 42f, 99f, width, 99f));
		lines.add(new Line(width - 42f, 159f, width, 159f));
		lines.add(new Line(width - 42f, 99f, width - 42f, 159f));
		Line[] lineArray = new Line[lines.size()];
		lines.toArray(lineArray);

		return createGridMap(new RPLineMap(lineArray, new Rectangle(0, 0,
				width, height)), xJunctions, yJunctions, xInset, yInset,
				junctionSeparation);
	}

	@Test
	public void testMapTest() {

		IGridMap map = createTestMap();
		int width = map.getXSize();
		int height = map.getYSize();

		HashSet<Point> blocked = new HashSet<Point>();
		blocked.add(new Point(8, 0));
		blocked.add(new Point(8, 1));
		blocked.add(new Point(10, 3));
		blocked.add(new Point(11, 3));
		blocked.add(new Point(10, 4));
		blocked.add(new Point(11, 4));
		blocked.add(new Point(2, 4));
		blocked.add(new Point(2, 5));

		HashMap<Point, Point> invalid = new HashMap<Point, Point>();
		invalid.put(new Point(1, 1), new Point(1, 2));
		invalid.put(new Point(2, 1), new Point(2, 2));
		invalid.put(new Point(3, 1), new Point(3, 2));
		invalid.put(new Point(4, 1), new Point(4, 2));
		invalid.put(new Point(5, 1), new Point(5, 2));
		invalid.put(new Point(6, 1), new Point(6, 2));
		invalid.put(new Point(7, 1), new Point(7, 2));
		invalid.put(new Point(4, 3), new Point(4, 4));
		invalid.put(new Point(5, 3), new Point(5, 4));
		invalid.put(new Point(6, 3), new Point(6, 4));
		invalid.put(new Point(7, 3), new Point(7, 4));

		invalid.put(new Point(3, 4), new Point(4, 4));
		invalid.put(new Point(3, 5), new Point(4, 5));
		invalid.put(new Point(3, 6), new Point(4, 6));

		invalid.put(new Point(5, 6), new Point(6, 6));
		invalid.put(new Point(6, 5), new Point(6, 6));
		invalid.put(new Point(7, 5), new Point(7, 6));

		for (int y = 0; y < height; y++) {

			for (int x = 0; x < width; x++) {

				Point from = new Point(x, y);

				if (blocked.contains(from)) {

					// an in place transition should be fine
					Assert.assertFalse(map.isValidTransition(from.x, from.y,
							from.x, from.y));
				} else {
					if (x > 0) {

						Point to = new Point(x - 1, y);

						if (blocked.contains(to)
								|| !isManuallyApproved(from, to, invalid)) {
							Assert.assertFalse(map.isValidTransition(from.x,
									from.y, to.x, to.y));
						} else {
							Assert.assertTrue(map.isValidTransition(from.x,
									from.y, to.x, to.y));
						}
					}

					if (x < width - 1) {

						Point to = new Point(x + 1, y);

						if (blocked.contains(to)
								|| !isManuallyApproved(from, to, invalid)) {
							Assert.assertFalse(map.isValidTransition(from.x,
									from.y, to.x, to.y));
						} else {
							Assert.assertTrue(map.isValidTransition(from.x,
									from.y, to.x, to.y));
						}
					}

					if (y > 0) {

						Point to = new Point(x, y - 1);

						if (blocked.contains(to)
								|| !isManuallyApproved(from, to, invalid)) {
							Assert.assertFalse(map.isValidTransition(from.x,
									from.y, to.x, to.y));
						} else {
							Assert.assertTrue(map.isValidTransition(from.x,
									from.y, to.x, to.y));
						}
					}

					if (y < height - 1) {

						Point to = new Point(x, y + 1);

						if (blocked.contains(to)
								|| !isManuallyApproved(from, to, invalid)) {
							Assert.assertFalse(map.isValidTransition(from.x,
									from.y, to.x, to.y));
						} else {
							Assert.assertTrue(map.isValidTransition(from.x,
									from.y, to.x, to.y), "from " + from
									+ " to " + to);
						}
					}
				}
			}
		}

	}

	private boolean isManuallyApproved(Point _from, Point _to,
			HashMap<Point, Point> _invalid) {

		Point to = _invalid.get(_from);
		if (to != null && to.equals(_to)) {
			return false;
		}

		Point from = _invalid.get(_to);
		if (from != null && from.equals(_from)) {
			return false;
		}

		return true;
	}

	@Test
	public void rectangularMapTest() {
		int width = 10;
		int height = 10;

		IGridMap map = createRectangularGridMap(width, height, 30);

		for (int y = 0; y < height; y++) {

			for (int x = 0; x < width; x++) {

				int toX = x;
				int toY = y;

				// an in place transition should be fine
				Assert.assertTrue(map.isValidTransition(x, y, toX, toY));

				if (x > 0) {
					toX = x - 1;
					Assert.assertTrue(map.isValidTransition(x, y, toX, toY));
				}

				if (x < width - 1) {
					toX = x + 1;
					Assert.assertTrue(map.isValidTransition(x, y, toX, toY));
				}

				if (y > 0) {
					toY = y - 1;
					Assert.assertTrue(map.isValidTransition(x, y, toX, toY));
				}

				if (y < height - 1) {
					toY = y + 1;
					Assert.assertTrue(map.isValidTransition(x, y, toX, toY));
				}

			}
		}
	}

	@Test
	private void rangeToObstacleTest() {

		float sep = 30f;
		float target = sep / 2f;
		IGridMap map = createRectangularGridMap(1, 1, sep);
		Assert.assertEquals(
				map.rangeToObstacleFromGridPosition(0, 0,
						Heading.toDegrees(Heading.PLUS_X)), target, 0f);
		Assert.assertEquals(
				map.rangeToObstacleFromGridPosition(0, 0,
						Heading.toDegrees(Heading.PLUS_Y)), target, 0f);
		Assert.assertEquals(
				map.rangeToObstacleFromGridPosition(0, 0,
						Heading.toDegrees(Heading.MINUS_X)), target, 0f);
		Assert.assertEquals(
				map.rangeToObstacleFromGridPosition(0, 0,
						Heading.toDegrees(Heading.MINUS_Y)), target, 0f);

		map = createTestMap();

		// copied from above method
		float height = 238;
		float width = 366;

		// position of 0,0 junction wrt to top left of map
		int xInset = 24;
		int yInset = 24;

		Assert.assertEquals(
				map.rangeToObstacleFromGridPosition(0, 0,
						Heading.toDegrees(Heading.PLUS_X)), 233f, 0f);

		Assert.assertEquals(
				map.rangeToObstacleFromGridPosition(0, 0,
						Heading.toDegrees(Heading.PLUS_Y)), height - yInset, 0f);

		Assert.assertEquals(
				map.rangeToObstacleFromGridPosition(0, 0,
						Heading.toDegrees(Heading.MINUS_X)), xInset, 0f);

		Assert.assertEquals(
				map.rangeToObstacleFromGridPosition(0, 0,
						Heading.toDegrees(Heading.MINUS_Y)), yInset, 0f);

	}
	
@Test
	private void ActionTest()
	{
	RPLineMap lineMap = MapUtils.create2015Map1();

	// Grid map configuration

	// Grid junction numbers
	int xJunctions = 12;
	int yJunctions = 8;
	float junctionSeparation = 30;

	// position of grid map 0,0
	int xInset = 13;
	int yInset = 13;

	//IGridMap gridMap = GridMapViewer.createGridMap(lineMap, xJunctions,
	//		yJunctions, xInset, yInset, junctionSeparation);

	MyMap map = new MyMap (lineMap,xJunctions,yJunctions, xInset, yInset, junctionSeparation );
	System.out.println(map.getXSize());
	// the starting position of the robot for the simulation. This is not
	// known in the action model or position distribution
	int startGridX = 2;
	int startGridY = 1;

	// this converts the grid position into the underlying continuous
	// coordinate frame
	lejos.geom.Point startPoint = map.getCoordinatesOfGridPosition(startGridX,
			startGridY);

	// starting heading
	float startTheta = Heading.toDegrees(Heading.PLUS_X);

	Pose startPose = new Pose(startPoint.x, startPoint.y, startTheta);

	// This creates a simulated robot with single, forward pointing distance
	// sensor with similar properties to the Lego ultrasonic sensor but
	// without the noise
	SimulatedRobot robot = SimulatedRobot.createSingleNoiseFreeSensorRobot(
			startPose, lineMap);

	
	
	// This does the same as above but adds noise to the range readings
	// SimulatedRobot robot = SimulatedRobot.createSingleSensorRobot(
	// startPose, lineMap);

	ActionModel actionModel = new PerfectActionModel();
	//ActionModel actionModel = new Zazy();
	SensorModel sensorModel = new PerfectSensorModel();
	//SensorModel sensorModel = new Zazyy();
	
	MarkovLocalisationSkeleton ml = new MarkovLocalisationSkeleton(robot,
			lineMap, map, junctionSeparation);
	
	// visualising to see whats going on
	ml.visualise();
	Heading a = Heading.PLUS_X;
	Heading b = Heading.MINUS_X;
	Heading c = Heading.MINUS_Y;
	Heading d = Heading.PLUS_Y;
	// moving the robot around and setting the correct heading for the models
	ml.move(junctionSeparation, a, actionModel, sensorModel);
	ml.move(junctionSeparation, a, actionModel, sensorModel);
	ml.move(junctionSeparation, a, actionModel, sensorModel);
	ml.move(junctionSeparation, a, actionModel, sensorModel);
	ml.move(junctionSeparation, a, actionModel, sensorModel);
	ml.move(junctionSeparation, a, actionModel, sensorModel);
	ml.move(junctionSeparation, a, actionModel, sensorModel);
	ml.move(junctionSeparation, a, actionModel, sensorModel);


	robot.rotate(90);

	ml.move(junctionSeparation, d, actionModel, sensorModel);
	ml.move(junctionSeparation, d, actionModel, sensorModel);	
	ml.move(junctionSeparation, d, actionModel, sensorModel);	
	ml.move(junctionSeparation, d, actionModel, sensorModel);
	

	robot.rotate(-90);

	ml.move(junctionSeparation, a, actionModel, sensorModel);



	// max will store maximum probability value
	float max=0;
	 
	//  we get robots position in centimeters and convert it to grid positions
float px=	robot.getPose().getX();
float py= robot.getPose().getY();

int aa = Math.round((px-map.xStart)/map.cellSize);
int bb = Math.round((py-map.yStart)/map.cellSize);


	int x = ml.getDistribution().getGridWidth();
	int y = ml.getDistribution().getGridHeight();
	boolean localized = false;
	for (int i=0; i<x; i++){
		for (int j=0; j<y; j++){
			
		float prob =	ml.getDistribution().getProbability(i, j);
		
		
			
			if (prob > max) { max = prob; }
			
			// if the robots position has the maximum probability  , localized becomes true else false
			if (i == aa && j == bb) { localized = ml.getDistribution().getProbability(i, j) == max; }

		}
		
	}

// we assert the robot is lozalized
	Assert.assertEquals(localized,true);
	

}

	}
