#include "utils.h"
#include <math.h>
#include <string.h>

// Private variables
static volatile int sys_lock_cnt = 0;

void utils_step_towards(float *value, float goal, float step) {
    if (*value < goal) {
        if ((*value + step) < goal) {
            *value += step;
        } else {
            *value = goal;
        }
    } else if (*value > goal) {
        if ((*value - step) > goal) {
            *value -= step;
        } else {
            *value = goal;
        }
    }
}

float utils_calc_ratio(float low, float high, float val) {
	return (val - low) / (high - low);
}

/**
 * Make sure that 0 <= angle < 360
 *
 * @param angle
 * The angle to normalize.
 */
void utils_norm_angle(float *angle) {
	*angle = fmodf(*angle, 360.0f);

	if (*angle < 0.0f) {
		*angle += 360.0f;
	}
}

/**
 * Make sure that -pi <= angle < pi,
 *
 * TODO: Maybe use fmodf instead?
 *
 * @param angle
 * The angle to normalize in radians.
 * WARNING: Don't use too large angles.
 */
void utils_norm_angle_rad(float *angle) {
	while (*angle < -M_PI) {
		*angle += 2.0f * M_PI;
	}

	while (*angle >  M_PI) {
		*angle -= 2.0f * M_PI;
	}
}

int utils_truncate_number(float *number, float min, float max) {
	int did_trunc = 0;

	if (*number > max) {
		*number = max;
		did_trunc = 1;
	} else if (*number < min) {
		*number = min;
		did_trunc = 1;
	}

	return did_trunc;
}

int utils_truncate_number_int(int *number, int min, int max) {
	int did_trunc = 0;

	if (*number > max) {
		*number = max;
		did_trunc = 1;
	} else if (*number < min) {
		*number = min;
		did_trunc = 1;
	}

	return did_trunc;
}

int utils_truncate_number_abs(float *number, float max) {
	int did_trunc = 0;

	if (*number > max) {
		*number = max;
		did_trunc = 1;
	} else if (*number < -max) {
		*number = -max;
		did_trunc = 1;
	}

	return did_trunc;
}

int utils_truncate_number_abs_s32(s32 *number, s32 max){
	int did_trunc = 0;

	if (*number > max) {
		*number = max;
		did_trunc = 1;
	} else if (*number < -max) {
		*number = -max;
		did_trunc = 1;
	}

	return did_trunc;
}
  
float utils_map(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int utils_map_int(int x, int in_min, int in_max, int out_min, int out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * Truncate absolute values less than tres to zero. The value
 * tres will be mapped to 0 and the value max to max.
 */
void utils_deadband(float *value, float tres, float max) {
	if (fabsf(*value) < tres) {
		*value = 0.0f;
	} else {
		float k = max / (max - tres);
		if (*value > 0.0f) {
			*value = k * *value + max * (1.0f - k);
		} else {
			*value = -(k * -*value + max * (1.0f - k));
		}

	}
}

/**
 * Get the difference between two angles. Will always be between -180 and +180 degrees.
 * @param angle1
 * The first angle
 * @param angle2
 * The second angle
 * @return
 * The difference between the angles
 */
float utils_angle_difference(float angle1, float angle2) {
	// Faster in most cases
	float difference = angle1 - angle2;
	while (difference < -180.0f) difference += 2.0f * 180.0f;
	while (difference > 180.0f) difference -= 2.0f * 180.0f;
	return difference;
}

/**
 * Get the difference between two angles. Will always be between -pi and +pi radians.
 * @param angle1
 * The first angle in radians
 * @param angle2
 * The second angle in radians
 * @return
 * The difference between the angles in radians
 */
float utils_angle_difference_rad(float angle1, float angle2) {
	float difference = angle1 - angle2;
	while (difference < -M_PI) difference += 2.0f * M_PI;
	while (difference > M_PI) difference -= 2.0f * M_PI;
	return difference;
}

/**
 * Takes the average of a number of angles.
 *
 * @param angles
 * The angles in radians.
 *
 * @param angles_num
 * The number of angles.
 *
 * @param weights
 * The weight of the summarized angles
 *
 * @return
 * The average angle.
 */
float utils_avg_angles_rad_fast(float *angles, float *weights, int angles_num) {
	float s_sum = 0.0f;
	float c_sum = 0.0f;

	for (int i = 0; i < angles_num; i++) {
		float s, c;
		utils_fast_sincos_better(angles[i], &s, &c);
		s_sum += s * weights[i];
		c_sum += c * weights[i];
	}

	return utils_fast_atan2(s_sum, c_sum);
}

/**
 * Get the middle value of three values
 *
 * @param a
 * First value
 *
 * @param b
 * Second value
 *
 * @param c
 * Third value
 *
 * @return
 * The middle value
 */
float utils_middle_of_3(float a, float b, float c) {
	float middle;

	if ((a <= b) && (a <= c)) {
		middle = (b <= c) ? b : c;
	} else if ((b <= a) && (b <= c)) {
		middle = (a <= c) ? a : c;
	} else {
		middle = (a <= b) ? a : b;
	}
	return middle;
}

/**
 * Get the middle value of three values
 *
 * @param a
 * First value
 *
 * @param b
 * Second value
 *
 * @param c
 * Third value
 *
 * @return
 * The middle value
 */
int utils_middle_of_3_int(int a, int b, int c) {
	int middle;

	if ((a <= b) && (a <= c)) {
		middle = (b <= c) ? b : c;
	} else if ((b <= a) && (b <= c)) {
		middle = (a <= c) ? a : c;
	} else {
		middle = (a <= b) ? a : b;
	}
	return middle;
}

// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float utils_fast_inv_sqrt(float x) {
	union {
		float as_float;
		long as_int;
	} un;

	float xhalf = 0.5f*x;
	un.as_float = x;
	un.as_int = 0x5f3759df - (un.as_int >> 1);
	un.as_float = un.as_float * (1.5f - xhalf * un.as_float * un.as_float);
	return un.as_float;
}

/**
 * Fast atan2
 *
 * See http://www.dspguru.com/dsp/tricks/fixed-point-atan2-with-self-normalization
 *
 * @param y
 * y
 *
 * @param x
 * x
 *
 * @return
 * The angle in radians
 */
float utils_fast_atan2(float y, float x) {
	float abs_y = fabsf(y) + 1e-20f; // kludge to prevent 0/0 condition
	float angle;

	if (x >= 0) {
		float r = (x - abs_y) / (x + abs_y);
		float rsq = r * r;
		angle = ((0.1963f * rsq) - 0.9817f) * r + (M_PI / 4.0f);
	} else {
		float r = (x + abs_y) / (abs_y - x);
		float rsq = r * r;
		angle = ((0.1963f * rsq) - 0.9817f) * r + (3.0f * M_PI / 4.0f);
	}

	if (y < 0) {
		return(-angle);
	} else {
		return(angle);
	}
}

/**
 * Truncate the magnitude of a vector.
 *
 * @param x
 * The first component.
 *
 * @param y
 * The second component.
 *
 * @param max
 * The maximum magnitude.
 *
 * @return
 * True if saturation happened, false otherwise
 */
bool utils_saturate_vector_2d(float *x, float *y, float max) {
	bool retval = false;
	float mag = sqrtf(*x * *x + *y * *y);
	max = fabsf(max);

	if (mag < 1e-10f) {
		mag = 1e-10f;
	}

	if (mag > max) {
		const float f = max / mag;
		*x *= f;
		*y *= f;
		retval = true;
	}

	return retval;
}

/**
 * Fast sine and cosine implementation.
 *
 * See http://lab.polygonal.de/?p=205
 *
 * @param angle
 * The angle in radians
 * WARNING: Don't use too large angles.
 *
 * @param sin
 * A pointer to store the sine value.
 *
 * @param cos
 * A pointer to store the cosine value.
 */
void utils_fast_sincos(float angle, float *sin, float *cos) {
	//always wrap input angle to -PI..PI
	while (angle < -M_PI) {
		angle += 2.0f * M_PI;
	}

	while (angle >  M_PI) {
		angle -= 2.0f * M_PI;
	}

	// compute sine
	if (angle < 0.0f) {
		*sin = 1.27323954f * angle + 0.405284735f * angle * angle;
	} else {
		*sin = 1.27323954f * angle - 0.405284735f * angle * angle;
	}

	// compute cosine: sin(x + PI/2) = cos(x)
	angle += 0.5f * M_PI;

	if (angle >  M_PI) {
		angle -= 2.0f * M_PI;
	}

	if (angle < 0.0f) {
		*cos = 1.27323954f * angle + 0.405284735f * angle * angle;
	} else {
		*cos = 1.27323954f * angle - 0.405284735f * angle * angle;
	}
}

/**
 * Fast sine and cosine implementation.
 *
 * See http://lab.polygonal.de/?p=205
 *
 * @param angle
 * The angle in radians
 * WARNING: Don't use too large angles.
 *
 * @param sin
 * A pointer to store the sine value.
 *
 * @param cos
 * A pointer to store the cosine value.
 */
void utils_fast_sincos_better(float angle, float *sin, float *cos) {
	//always wrap input angle to -PI..PI
	while (angle < -M_PI) {
		angle += 2.0f * M_PI;
	}

	while (angle >  M_PI) {
		angle -= 2.0f * M_PI;
	}

	//compute sine
	if (angle < 0.0f) {
		*sin = 1.27323954f * angle + 0.405284735f * angle * angle;

		if (*sin < 0.0f) {
			*sin = 0.225f * (*sin * -*sin - *sin) + *sin;
		} else {
			*sin = 0.225f * (*sin * *sin - *sin) + *sin;
		}
	} else {
		*sin = 1.27323954f * angle - 0.405284735f * angle * angle;

		if (*sin < 0.0f) {
			*sin = 0.225f * (*sin * -*sin - *sin) + *sin;
		} else {
			*sin = 0.225f * (*sin * *sin - *sin) + *sin;
		}
	}

	// compute cosine: sin(x + PI/2) = cos(x)
	angle += 0.5f * M_PI;
	if (angle >  M_PI) {
		angle -= 2.0f * M_PI;
	}

	if (angle < 0.0f) {
		*cos = 1.27323954f * angle + 0.405284735f * angle * angle;

		if (*cos < 0.0f) {
			*cos = 0.225f * (*cos * -*cos - *cos) + *cos;
		} else {
			*cos = 0.225f * (*cos * *cos - *cos) + *cos;
		}
	} else {
		*cos = 1.27323954f * angle - 0.405284735f * angle * angle;

		if (*cos < 0.0f) {
			*cos = 0.225f * (*cos * -*cos - *cos) + *cos;
		} else {
			*cos = 0.225f * (*cos * *cos - *cos) + *cos;
		}
	}
}

/**
 * Calculate the values with the lowest magnitude.
 *
 * @param va
 * The first value.
 *
 * @param vb
 * The second value.
 *
 * @return
 * The value with the lowest magnitude.
 */
float utils_min_abs(float va, float vb) {
	float res;
	if (fabsf(va) < fabsf(vb)) {
		res = va;
	} else {
		res = vb;
	}

	return res;
}

/**
 * Calculate the values with the highest magnitude.
 *
 * @param va
 * The first value.
 *
 * @param vb
 * The second value.
 *
 * @return
 * The value with the highest magnitude.
 */
float utils_max_abs(float va, float vb) {
	float res;
	if (fabsf(va) > fabsf(vb)) {
		res = va;
	} else {
		res = vb;
	}

	return res;
}

/**
 * Create string representation of the binary content of a byte
 *
 * @param x
 * The byte.
 *
 * @param b
 * Array to store the string representation in.
 */
void utils_byte_to_binary(int x, char *b) {
	b[0] = '\0';

	int z;
	for (z = 128; z > 0; z >>= 1) {
		strcat(b, ((x & z) == z) ? "1" : "0");
	}
}

float utils_throttle_curve(float val, float curve_acc, float curve_brake, int mode) {
	float ret = 0.0f;
	
	if (val < -1.0f) {
		val = -1.0f;
	}

	if (val > 1.0f) {
		val = 1.0f;
	}
	
	float val_a = fabsf(val);

	float curve;
	if (val >= 0.0f) {
		curve = curve_acc;
	} else {
		curve = curve_brake;
	}

	// See
	// http://math.stackexchange.com/questions/297768/how-would-i-create-a-exponential-ramp-function-from-0-0-to-1-1-with-a-single-val
	if (mode == 0) { // Exponential
		if (curve >= 0.0f) {
			ret = 1.0f - powf(1.0f - val_a, 1.0f + curve);
		} else {
			ret = powf(val_a, 1.0f - curve);
		}
	} else if (mode == 1) { // Natural
		if (fabsf(curve) < 1e-10f) {
			ret = val_a;
		} else {
			if (curve >= 0.0f) {
				ret = 1.0f - ((expf(curve * (1.0f - val_a)) - 1.0f) / (expf(curve) - 1.0f));
			} else {
				ret = (expf(-curve * val_a) - 1.0f) / (expf(-curve) - 1.0f);
			}
		}
	} else if (mode == 2) { // Polynomial
		if (curve >= 0.0f) {
			ret = 1.0f - ((1.0f - val_a) / (1.0f + curve * val_a));
		} else {
			ret = val_a / (1.0f - curve * (1.0f - val_a));
		}
	} else { // Linear
		ret = val_a;
	}

	if (val < 0.0f) {
		ret = -ret;
	}

	return ret;
}

const float32_t sinTable_f32[FAST_MATH_TABLE_SIZE + 1] = {
   0.00000000f, 0.01227154f, 0.02454123f, 0.03680722f, 0.04906767f, 0.06132074f,
   0.07356456f, 0.08579731f, 0.09801714f, 0.11022221f, 0.12241068f, 0.13458071f,
   0.14673047f, 0.15885814f, 0.17096189f, 0.18303989f, 0.19509032f, 0.20711138f,
   0.21910124f, 0.23105811f, 0.24298018f, 0.25486566f, 0.26671276f, 0.27851969f,
   0.29028468f, 0.30200595f, 0.31368174f, 0.32531029f, 0.33688985f, 0.34841868f,
   0.35989504f, 0.37131719f, 0.38268343f, 0.39399204f, 0.40524131f, 0.41642956f,
   0.42755509f, 0.43861624f, 0.44961133f, 0.46053871f, 0.47139674f, 0.48218377f,
   0.49289819f, 0.50353838f, 0.51410274f, 0.52458968f, 0.53499762f, 0.54532499f,
   0.55557023f, 0.56573181f, 0.57580819f, 0.58579786f, 0.59569930f, 0.60551104f,
   0.61523159f, 0.62485949f, 0.63439328f, 0.64383154f, 0.65317284f, 0.66241578f,
   0.67155895f, 0.68060100f, 0.68954054f, 0.69837625f, 0.70710678f, 0.71573083f,
   0.72424708f, 0.73265427f, 0.74095113f, 0.74913639f, 0.75720885f, 0.76516727f,
   0.77301045f, 0.78073723f, 0.78834643f, 0.79583690f, 0.80320753f, 0.81045720f,
   0.81758481f, 0.82458930f, 0.83146961f, 0.83822471f, 0.84485357f, 0.85135519f,
   0.85772861f, 0.86397286f, 0.87008699f, 0.87607009f, 0.88192126f, 0.88763962f,
   0.89322430f, 0.89867447f, 0.90398929f, 0.90916798f, 0.91420976f, 0.91911385f,
   0.92387953f, 0.92850608f, 0.93299280f, 0.93733901f, 0.94154407f, 0.94560733f,
   0.94952818f, 0.95330604f, 0.95694034f, 0.96043052f, 0.96377607f, 0.96697647f,
   0.97003125f, 0.97293995f, 0.97570213f, 0.97831737f, 0.98078528f, 0.98310549f,
   0.98527764f, 0.98730142f, 0.98917651f, 0.99090264f, 0.99247953f, 0.99390697f,
   0.99518473f, 0.99631261f, 0.99729046f, 0.99811811f, 0.99879546f, 0.99932238f,
   0.99969882f, 0.99992470f, 1.00000000f, 0.99992470f, 0.99969882f, 0.99932238f,
   0.99879546f, 0.99811811f, 0.99729046f, 0.99631261f, 0.99518473f, 0.99390697f,
   0.99247953f, 0.99090264f, 0.98917651f, 0.98730142f, 0.98527764f, 0.98310549f,
   0.98078528f, 0.97831737f, 0.97570213f, 0.97293995f, 0.97003125f, 0.96697647f,
   0.96377607f, 0.96043052f, 0.95694034f, 0.95330604f, 0.94952818f, 0.94560733f,
   0.94154407f, 0.93733901f, 0.93299280f, 0.92850608f, 0.92387953f, 0.91911385f,
   0.91420976f, 0.90916798f, 0.90398929f, 0.89867447f, 0.89322430f, 0.88763962f,
   0.88192126f, 0.87607009f, 0.87008699f, 0.86397286f, 0.85772861f, 0.85135519f,
   0.84485357f, 0.83822471f, 0.83146961f, 0.82458930f, 0.81758481f, 0.81045720f,
   0.80320753f, 0.79583690f, 0.78834643f, 0.78073723f, 0.77301045f, 0.76516727f,
   0.75720885f, 0.74913639f, 0.74095113f, 0.73265427f, 0.72424708f, 0.71573083f,
   0.70710678f, 0.69837625f, 0.68954054f, 0.68060100f, 0.67155895f, 0.66241578f,
   0.65317284f, 0.64383154f, 0.63439328f, 0.62485949f, 0.61523159f, 0.60551104f,
   0.59569930f, 0.58579786f, 0.57580819f, 0.56573181f, 0.55557023f, 0.54532499f,
   0.53499762f, 0.52458968f, 0.51410274f, 0.50353838f, 0.49289819f, 0.48218377f,
   0.47139674f, 0.46053871f, 0.44961133f, 0.43861624f, 0.42755509f, 0.41642956f,
   0.40524131f, 0.39399204f, 0.38268343f, 0.37131719f, 0.35989504f, 0.34841868f,
   0.33688985f, 0.32531029f, 0.31368174f, 0.30200595f, 0.29028468f, 0.27851969f,
   0.26671276f, 0.25486566f, 0.24298018f, 0.23105811f, 0.21910124f, 0.20711138f,
   0.19509032f, 0.18303989f, 0.17096189f, 0.15885814f, 0.14673047f, 0.13458071f,
   0.12241068f, 0.11022221f, 0.09801714f, 0.08579731f, 0.07356456f, 0.06132074f,
   0.04906767f, 0.03680722f, 0.02454123f, 0.01227154f, 0.00000000f, -0.01227154f,
   -0.02454123f, -0.03680722f, -0.04906767f, -0.06132074f, -0.07356456f,
   -0.08579731f, -0.09801714f, -0.11022221f, -0.12241068f, -0.13458071f,
   -0.14673047f, -0.15885814f, -0.17096189f, -0.18303989f, -0.19509032f, 
   -0.20711138f, -0.21910124f, -0.23105811f, -0.24298018f, -0.25486566f, 
   -0.26671276f, -0.27851969f, -0.29028468f, -0.30200595f, -0.31368174f, 
   -0.32531029f, -0.33688985f, -0.34841868f, -0.35989504f, -0.37131719f, 
   -0.38268343f, -0.39399204f, -0.40524131f, -0.41642956f, -0.42755509f, 
   -0.43861624f, -0.44961133f, -0.46053871f, -0.47139674f, -0.48218377f, 
   -0.49289819f, -0.50353838f, -0.51410274f, -0.52458968f, -0.53499762f, 
   -0.54532499f, -0.55557023f, -0.56573181f, -0.57580819f, -0.58579786f, 
   -0.59569930f, -0.60551104f, -0.61523159f, -0.62485949f, -0.63439328f, 
   -0.64383154f, -0.65317284f, -0.66241578f, -0.67155895f, -0.68060100f, 
   -0.68954054f, -0.69837625f, -0.70710678f, -0.71573083f, -0.72424708f, 
   -0.73265427f, -0.74095113f, -0.74913639f, -0.75720885f, -0.76516727f, 
   -0.77301045f, -0.78073723f, -0.78834643f, -0.79583690f, -0.80320753f, 
   -0.81045720f, -0.81758481f, -0.82458930f, -0.83146961f, -0.83822471f, 
   -0.84485357f, -0.85135519f, -0.85772861f, -0.86397286f, -0.87008699f, 
   -0.87607009f, -0.88192126f, -0.88763962f, -0.89322430f, -0.89867447f, 
   -0.90398929f, -0.90916798f, -0.91420976f, -0.91911385f, -0.92387953f, 
   -0.92850608f, -0.93299280f, -0.93733901f, -0.94154407f, -0.94560733f, 
   -0.94952818f, -0.95330604f, -0.95694034f, -0.96043052f, -0.96377607f, 
   -0.96697647f, -0.97003125f, -0.97293995f, -0.97570213f, -0.97831737f, 
   -0.98078528f, -0.98310549f, -0.98527764f, -0.98730142f, -0.98917651f, 
   -0.99090264f, -0.99247953f, -0.99390697f, -0.99518473f, -0.99631261f, 
   -0.99729046f, -0.99811811f, -0.99879546f, -0.99932238f, -0.99969882f, 
   -0.99992470f, -1.00000000f, -0.99992470f, -0.99969882f, -0.99932238f, 
   -0.99879546f, -0.99811811f, -0.99729046f, -0.99631261f, -0.99518473f, 
   -0.99390697f, -0.99247953f, -0.99090264f, -0.98917651f, -0.98730142f, 
   -0.98527764f, -0.98310549f, -0.98078528f, -0.97831737f, -0.97570213f, 
   -0.97293995f, -0.97003125f, -0.96697647f, -0.96377607f, -0.96043052f, 
   -0.95694034f, -0.95330604f, -0.94952818f, -0.94560733f, -0.94154407f, 
   -0.93733901f, -0.93299280f, -0.92850608f, -0.92387953f, -0.91911385f, 
   -0.91420976f, -0.90916798f, -0.90398929f, -0.89867447f, -0.89322430f, 
   -0.88763962f, -0.88192126f, -0.87607009f, -0.87008699f, -0.86397286f, 
   -0.85772861f, -0.85135519f, -0.84485357f, -0.83822471f, -0.83146961f, 
   -0.82458930f, -0.81758481f, -0.81045720f, -0.80320753f, -0.79583690f, 
   -0.78834643f, -0.78073723f, -0.77301045f, -0.76516727f, -0.75720885f, 
   -0.74913639f, -0.74095113f, -0.73265427f, -0.72424708f, -0.71573083f, 
   -0.70710678f, -0.69837625f, -0.68954054f, -0.68060100f, -0.67155895f, 
   -0.66241578f, -0.65317284f, -0.64383154f, -0.63439328f, -0.62485949f, 
   -0.61523159f, -0.60551104f, -0.59569930f, -0.58579786f, -0.57580819f, 
   -0.56573181f, -0.55557023f, -0.54532499f, -0.53499762f, -0.52458968f, 
   -0.51410274f, -0.50353838f, -0.49289819f, -0.48218377f, -0.47139674f, 
   -0.46053871f, -0.44961133f, -0.43861624f, -0.42755509f, -0.41642956f, 
   -0.40524131f, -0.39399204f, -0.38268343f, -0.37131719f, -0.35989504f, 
   -0.34841868f, -0.33688985f, -0.32531029f, -0.31368174f, -0.30200595f, 
   -0.29028468f, -0.27851969f, -0.26671276f, -0.25486566f, -0.24298018f, 
   -0.23105811f, -0.21910124f, -0.20711138f, -0.19509032f, -0.18303989f, 
   -0.17096189f, -0.15885814f, -0.14673047f, -0.13458071f, -0.12241068f, 
   -0.11022221f, -0.09801714f, -0.08579731f, -0.07356456f, -0.06132074f, 
   -0.04906767f, -0.03680722f, -0.02454123f, -0.01227154f, -0.00000000f
};

float32_t utils_sin_f32(float x) {

  float32_t sinVal, fract, in;                           /* Temporary variables for input, output */
  uint16_t index;                                        /* Index variable */
  float32_t a, b;                                        /* Two nearest output values */
  int32_t n;
  float32_t findex;

  /* input x is in radians */
  /* Scale the input to [0 1] range from [0 2*PI] , divide input by 2*pi */
  in = x * 0.159154943092f;

  /* Calculation of floor value of input */
  n = (int32_t) in;

  /* Make negative values towards -infinity */
  if(x < 0.0f)
  {
    n--;
  }

  /* Map input value to [0 1] */
  in = in - (float32_t) n;

  /* Calculation of index of the table */
  findex = (float32_t) FAST_MATH_TABLE_SIZE * in;
  index = ((uint16_t)findex) & 0x1ff;

  /* fractional value calculation */
  fract = findex - (float32_t) index;

  /* Read two nearest values of input value from the sin table */
  a = sinTable_f32[index];
  b = sinTable_f32[index+1];

  /* Linear interpolation process */
  sinVal = (1.0f-fract)*a + fract*b;

  /* Return the output value */
  return (sinVal);
}


float32_t utils_cos_f32(float32_t x)
{
  float32_t cosVal, fract, in;                   /* Temporary variables for input, output */
  uint16_t index;                                /* Index variable */
  float32_t a, b;                                /* Two nearest output values */
  int32_t n;
  float32_t findex;

  /* input x is in radians */
  /* Scale the input to [0 1] range from [0 2*PI] , divide input by 2*pi, add 0.25 (pi/2) to read sine table */
  in = x * 0.159154943092f + 0.25f;

  /* Calculation of floor value of input */
  n = (int32_t) in;

  /* Make negative values towards -infinity */
  if(in < 0.0f)
  {
    n--;
  }

  /* Map input value to [0 1] */
  in = in - (float32_t) n;

  /* Calculation of index of the table */
  findex = (float32_t) FAST_MATH_TABLE_SIZE * in;
  index = ((uint16_t)findex) & 0x1ff;

  /* fractional value calculation */
  fract = findex - (float32_t) index;

  /* Read two nearest values of input value from the cos table */
  a = sinTable_f32[index];
  b = sinTable_f32[index+1];

  /* Linear interpolation process */
  cosVal = (1.0f-fract)*a + fract*b;

  /* Return the output value */
  return (cosVal);
}
