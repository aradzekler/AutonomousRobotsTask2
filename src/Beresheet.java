public class Beresheet {

	public static final double MAIN_ENGINE_THRUST = 430;   // in Newtons.
	public static final double AUX_ENGINE_THRUST = 25;    // in Newtons.
	public static final double MAIN_ENGINE_FUEL_BURN = 1.0; // liter/sec.
	public static final double AUX_ENGINE_FUEL_BURN = 0.001; // liters/sec.
	public static final double AUX_ENGINE_NUM = 8;
	public static final double SPACECRAFT_WEIGHT = 164.0; // in kG.
	public static final double SPACECRAFT_DIAMETER = 2.288; // in meters.
	public static final double SPACECRAFT_RADIUS = 1.144; // in meters.
	final private double dt = 1; // delta-time (change in time in seconds).
	private double currAlt = 30000; // in km.
	private double batteryLife = 100.0; // battery in %.
	private double currAngle = 90;
	private int currTime = 0;
	private double angularSpeed = 0.0;
	private double angularAcceleration = 0.0;
	private double xSpeed = 1700.0; // in m/sec.
	private double ySpeed = -56.0; // in m/sec, negative number because we are going downwards and not upwards.
	private double xAccel = 0.0; // in m/sec.
	private double yAccel = 0.0; // in m/sec.
	private double fuelMass = 216.0; // in kG.
	double gravity = Moon.getNewtonMoonGravity(getWeight(), this.currAlt);


	//TODO: Add battery decay.

	// get current gravity (in relation to the moon).
	public static double getGravity(double mass, double height) {
		return Moon.getNewtonMoonGravity(mass, Moon.RADIUS + height);
	}

	// Update remaining fuel
	private void updateFuel() {
		this.fuelMass -= dt * (MAIN_ENGINE_FUEL_BURN + AUX_ENGINE_FUEL_BURN * AUX_ENGINE_NUM);
	}

	// Update current altitude.
	private void updateAltitude() {
		this.currAlt += Moon.getDisplacement(dt, this.xSpeed, this.yAccel);
	}

	// Updates current acceleration.
	private void updateAcceleration() {
		this.xSpeed += Moon.getCurrAcc(dt, this.xAccel);
		this.ySpeed += Moon.getCurrAcc(dt, this.yAccel);
	}

	// horizontal thrust (slowing down)
	private void xThrust() {
		this.xAccel -= getEngineThrust();
	}

	// vertical thrust (speeding up from negative y acceleration.
	private void yThrust() {
		this.yAccel += getEngineThrust();
	}

	// get spacecraft weight (spaceprobe weight + current fuel)
	public double getWeight() {
		return SPACECRAFT_WEIGHT + this.fuelMass;
	}

	// get current engine thrust.
	public double getEngineThrust() {
		double weight = getWeight();
		double force = MAIN_ENGINE_THRUST + AUX_ENGINE_THRUST * AUX_ENGINE_NUM;
		return Moon.getNewtonAcceleration(force, weight);
	}

	// updating our angle relative to the moon and normalizing it.
	private void updateAngle() {
		currAngle += Moon.getDisplacement(dt, angularSpeed, angularAcceleration);
		while (currAngle < -180) // Keep angle between -180 to 180 (we are in a 'flat' plane and not a circular one)
			currAngle += 360;
		while (currAngle > 180)
			currAngle -= 360;
	}

	// Update angular speed
	private void updateAngularSpeed() {
		angularSpeed += Moon.getNewtonAcceleration(dt, angularAcceleration);
	}

	// creates angular acceleration using newtonian physics formulas.
	public double getAngularAcceleration() {
		double torque = Moon.getTorque(SPACECRAFT_RADIUS, getEngineThrust());
		double moment = Moon.getNewtonMomentum(SPACECRAFT_RADIUS, angularAcceleration, getWeight());
		return Moon.getNewtonAcceleration(torque, moment);
	}

	// automatically rotate the craft back to original angle.
	private void rotateCraft() {
		if (currAngle > 0 && angularSpeed > -0.5) // if drifted too much, rotate left.
			angularAcceleration = -getAngularAcceleration();
		else if (currAngle < 0 && angularSpeed < 0.5) // else, rotate right.
			angularAcceleration = getAngularAcceleration();
		else
			angularAcceleration = 0;
		updateAngle();
		updateAngularSpeed();
	}


	// thrust schedule for flight.
	//TODO: Implement PID control here?
	private void activateEngines() {
		if (this.currAlt > 30000) {
			if ( this.ySpeed < -25 )
				yThrust();
		} else if (this.currAlt > 15000) {
			if (this.ySpeed < -100)
				yThrust();
		} else if (this.currAlt > 3000) {
			if (this.ySpeed < -50)
				yThrust();
		} else if (this.currAlt > 300) {
			if (this.ySpeed < -10)
				yThrust();
		} else if (this.currAlt > 30)  {
			if (this.ySpeed < -5)
				yThrust();
		}

	}

	private void engineTurnoff() {
		this.xAccel = 0; // no other forces working on our spacecraft except engines.
		double weight = getWeight();
		double gravity = Moon.getNewtonMoonGravity(weight, this.currAlt);
		this.yAccel = Moon.getNewtonAcceleration(gravity, weight); // calculate gravitational acceleration, i.e freefall.

	}

	// our attempt at landing.
	public void landingAction() {
		while (this.currAlt > 0 && this.fuelMass > 0) { // continue while can.
			currTime += dt; // simulation time loops of 1 sec.
			System.out.println(this.toString());
			engineTurnoff();
			activateEngines();
			rotateCraft();
			updateAltitude();
			updateAcceleration();
			updateFuel();

		}

		if (this.fuelMass < 1) {
			System.out.println("ERROR-CRASH: Fuel mass consumed before landing. Crashing imminent.");
			return;
		}

		if (this.currAlt < 1) { // touchdown.

			if (this.xSpeed < -5 || this.ySpeed > 5) { // if drift or fall speed is too high, consider it as crash.
				System.out.println("ERROR-CRASH: Spacecraft speed while touching down too high - CRASH");
				return;
			}
			if (currAngle < -3 || currAngle > 3) { // spacecraft angle too off.
				System.out.println("ERROR-CRASH: Spacecraft angle while touching down too high - CRASH");
				return;
			}
		}


	}


	@Override
	public String toString() {
		return String.format("%4ds\t%,9.2f\t%5.2fm/s\t%5.2fm/s²\t%7.2fm/s\t%5.2fm/s²\t%.2f\t%.2f\t%5.2f°\t%5.2f\t%6.3f",
				currTime,
				currAlt,
				ySpeed,
				yAccel,
				xSpeed,
				xAccel,
				fuelMass,
				getWeight(),
				currAngle,
				angularSpeed,
				angularAcceleration
		);
	}


}