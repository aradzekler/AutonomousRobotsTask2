public class Moon {
	// from: https://he.wikipedia.org/wiki/%D7%94%D7%99%D7%A8%D7%97
	public static final double RADIUS = 3475 * 1000; // meters
	public static final double ACC = 1.622; // m/s^2
	public static final double EQ_SPEED = 1700; // m/s
	public static final double MOON_MASS = 7.34767 * Math.pow(10, 22); // mass of the Moon = 7.34767309 × 10^22 kilograms
	public static final double GRAVITY = 6.674 * Math.pow(10, -11); // Newton's Gravity Constant.

	// https://www.calculatorsoup.com/calculators/physics/displacement_v_a_t.php
	// finds the distance traveled or displacement (s) of an object using its initial velocity (u),
	// acceleration (acc), and time (dt) traveled.
	public static double getDisplacement(double dt, double init_vel, double acc) {
		return init_vel * dt + acc * dt * dt / 2;
	}

	// simply multiple current acceleration with delta.
	public static double getCurrAcc(double dt, double acc) {
		return dt * acc;
	}


	/*
	// https://he.wikipedia.org/wiki/%D7%9B%D7%91%D7%99%D7%93%D7%94
	// get the current newtonian gravity.
	public static double getNewtonGravity(double largeMass, double smallMass, double distance) {
		return GRAVITY * largeMass * smallMass / Math.pow(distance, 2);
	}

	 */

	// https://he.wikipedia.org/wiki/%D7%9B%D7%91%D7%99%D7%93%D7%94
	// shows the gravity relative to the moon
	public static double getNewtonMoonGravity(double smallMass, double distance) {
		return -(GRAVITY * MOON_MASS * smallMass / Math.pow(distance, 2));
	}


	// dForce = mass * acceleration, so Acclereration = dFroce / mass
	public static double getNewtonAcceleration(double force, double mass) {
		return force / mass;
	}

	// Mass = dForce / acc
	public static double getNewtonMass(double force, double acc) {
		return force / acc;
	}

	// Momentum = mass * velocity
	public static double getNewtonMomentum(double force, double acc, double mass) {
		//return force / acc;
		return getNewtonMass(force, acc) * getNewtonAcceleration(force, mass);
	}


	// https://he.wikipedia.org/wiki/%D7%A0%D7%99%D7%95%D7%98%D7%95%D7%9F_(%D7%9E%D7%99%D7%93%D7%94)
	// Newton Force units, our mass times the current acceleration.
	public static double getNewtonForce(double mass, double acc) {
		return mass * acc;
	}

	// https://he.wikipedia.org/wiki/%D7%9E%D7%95%D7%9E%D7%A0%D7%98_%D7%9B%D7%95%D7%97
	// how much force we need to act on an object with radius 'r' in order to make it to spin.
	public static double getTorque(double force, double r) {
		return force * r;
	}


}
