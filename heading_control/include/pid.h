#ifndef PID_H
#define PID_H

#include <chrono>
#include <ratio>

class PID {
	public:
		PID() : kP(0.5), kI(0.5), kD(0.5), max_i_error(0.5) { // empty constructor; parameters initialized to 0.5
			last_update = clk.now();
		};

		PID(double p, double i, double d, double i_max) : kP(p), kI(i), kD(d), max_i_error(i_max) {
			last_update = clk.now();
		};

		enum terms_t { // allows for selection of which terms to return from update
			P = 0x1,
			I = 0x2,
			D = 0x4
		};

		// call as:  output = update(current_value, PID::terms::P | PID::terms::I);

		double update(double current, int terms) { // or'ing the desired terms from enum terms will make the function return only those terms
			std::chrono::steady_clock::time_point now = clk.now(); // get current time
			std::chrono::duration<double, std::milli> dt = now - last_update; // get time difference between now and the last time update ran
			last_update = now; // the last update time becomes now

			// proportional term calculation
			double last_p_error = p_error;
			p_error = goal - current; // proportional error, accounting for present error

			// integral term calculation
			i_error += p_error * dt.count(); // integral error accounts for past changes in error
			i_error = sgn(i_error) * std::fmin(std::abs(i_error), max_i_error); // fmin caps the i_error to max_i_error

			// derivative term calculation
			// accounts for future changes in error
			if(dt.count() != 0) d_error = (p_error - last_p_error) / dt.count(); // if dt isn't zero calculate d_error
			
			// treating t as a 3 bit integer, if the bit for any term is 1, it is included, otherwise it becomes 0
			return (((terms & terms_t::P) * kP * p_error) + ((terms & terms_t::I) * kI * i_error) + ((terms & terms_t::D) * kD * d_error));	
		};
		
		void set_kP(double new_kP) { 
			kP = new_kP;
			reset();
		};

		void set_kI(double new_kI) { 
			kI = new_kI;
			reset();
		};

		void set_kD(double new_kD) { 
			kD = new_kD;
			reset();
		};

		void target(double t) { 
			goal = t;
			reset();
		 };

		double get_target() { return goal; };
		
		void reset() { // sets errors to zero
			p_error = i_error = d_error = 0;
			last_update = clk.now();
		};

	private:
		// PID controller parameters
		double kP;
		double kI;
		double kD;
		
		// error values
		double p_error;
		double i_error;
		double d_error;

		// value for integral term capping
		double max_i_error;

		// target value
		double goal;
		
		// clock variables
		std::chrono::steady_clock clk;
		std::chrono::steady_clock::time_point last_update; // last time we called update()

		// for use in integral calculation
		double sgn(double x) {
			if(x == 0) return 0;
			else if(x < 0) return -1;
			else return 1;
		};
};

#endif
