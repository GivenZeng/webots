/*!
 * (c) 2006 - 2008 EPFL, Lausanne, Switzerland
 * Thomas Lochmatter
 * Adapted by Nicolas Heiniger for the e-puck
 */

#ifndef ODOMETRY
#define ODOMETRY

struct sOdometryTrack {
	struct {
		float wheel_distance;
		float wheel_conversion_left;
		float wheel_conversion_right;
	} configuration;
	struct {
		int pos_left_prev;
		int pos_right_prev;
	} state;
	struct {
		float x;
		float y;
		float theta;
	} result;
};

//! Initializes this module.
void odometry_track_init();

//! Initializes an sOdometryTrack structure using the given left and right position.
int odometry_track_start_pos(struct sOdometryTrack * ot, int pos_left, int pos_right);
//! Updates the position using the given left and right position.
void odometry_track_step_pos(struct sOdometryTrack * ot, int pos_left, int pos_right);

#endif
