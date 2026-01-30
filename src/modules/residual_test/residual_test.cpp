/****************************************************************************
 *
 *   Simple residual publisher for testing vehicle_torque_residual
 *   and vehicle_thrust_residual uORB topics.
 *
 ****************************************************************************/

#include <px4_platform_common/module.h>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>

#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_torque_residual.h>
#include <uORB/topics/vehicle_thrust_residual.h>

#include <cstdlib>   // atof
#include <cmath>     // sinf, M_PI
#include <unistd.h>  // usleep

extern "C" __EXPORT int residual_test_main(int argc, char *argv[]);

/**
 * Usage:
 *   residual_test             -> default fixed residuals
 *   residual_test 0.1 0.2     -> torque_z = 0.1, thrust_z = 0.2
 *
 * Run from PX4 shell:
 *   pxh> residual_test
 */
int residual_test_main(int argc, char *argv[])
{
	PX4_INFO("residual_test: starting");

	// Default residual values (in body frame)
	float torque_z_resid = 0.05f;  // [N·m] residual yaw torque
	float thrust_z_resid = 0.2f;   // [N]   residual thrust along body-z

	// Optional CLI args:
	//   argv[1] -> torque_z
	//   argv[2] -> thrust_z
	if (argc > 1) {
		torque_z_resid = static_cast<float>(atof(argv[1]));
	}

	if (argc > 2) {
		thrust_z_resid = static_cast<float>(atof(argv[2]));
	}

	PX4_INFO("residual_test: using torque_z = %.3f [N*m], thrust_z = %.3f [N]",
		 (double)torque_z_resid, (double)thrust_z_resid);

	uORB::Publication<vehicle_torque_residual_s> torque_pub{ORB_ID(vehicle_torque_residual)};
	uORB::Publication<vehicle_thrust_residual_s> thrust_pub{ORB_ID(vehicle_thrust_residual)};

	// 100 Hz loop, run for ~10 seconds
	const float publish_rate_hz = 100.0f;
	const int   loop_delay_us   = static_cast<int>(1e6f / publish_rate_hz);
	const int   max_iterations  = static_cast<int>(10.0f * publish_rate_hz);

	const hrt_abstime t0 = hrt_absolute_time();

	for (int i = 0; i < max_iterations; i++) {
		const hrt_abstime now = hrt_absolute_time();
		const float t = (now - t0) * 1e-6f; // [s]

		// You can use a constant value OR a small sinusoid
		const float torque_z  = torque_z_resid;   // or torque_z_resid * sinf(2.0f * M_PI * 0.5f * t);
		const float thrust_z  = thrust_z_resid;   // or thrust_z_resid * sinf(2.0f * M_PI * 0.5f * t);

		// ----------------- torque residual -----------------
		vehicle_torque_residual_s torque_msg{};
		torque_msg.timestamp = now;
		torque_msg.xyz[0] = 0.0f;        // roll residual
		torque_msg.xyz[1] = 0.0f;        // pitch residual
		torque_msg.xyz[2] = torque_z;    // yaw residual

		if (!torque_pub.publish(torque_msg)) {
			PX4_WARN("failed to publish vehicle_torque_residual");
		}

		// ----------------- thrust residual -----------------
		vehicle_thrust_residual_s thrust_msg{};
		thrust_msg.timestamp = now;
		thrust_msg.xyz[0] = 0.0f;        // body-x residual
		thrust_msg.xyz[1] = 0.0f;        // body-y residual
		thrust_msg.xyz[2] = thrust_z;    // body-z residual

		if (!thrust_pub.publish(thrust_msg)) {
			PX4_WARN("failed to publish vehicle_thrust_residual");
		}

		if (i % 100 == 0) {
			PX4_INFO("residual_test: t=%.2f s, tau_z=%.3f, T_z=%.3f",
				 (double)t, (double)torque_z, (double)thrust_z);
		}

		usleep(loop_delay_us);
	}

	PX4_INFO("residual_test: finished");
	return 0;
}

