#pragma once

/**
 * @brief Compare two number and return the larger one
 * 
 * @param a First number to be compared
 * @param b Second number to be compared
 * @return float 
 */
float max(float a, float b){
	return a > b? a : b;
}

/**
 * @brief Compare two number and return the smaller one
 * 
 * @param a First number to be compared
 * @param b Second number to be compared
 * @return float 
 */
float min(float a, float b){
	return a < b? a : b;
}

/**
 * @brief Convert RGB into V
 * 
 * @param r R value
 * @param g G value
 * @param b B value
 * @return float 
 */
float rbg2v(long r, long g, long b){
	return max(r / 255, max(g / 255, b / 255)) * 100;
}

/**
 * @brief Convert RGB into Reflected light value (Grayscale)
 * 
 * @param r R value
 * @param g G value
 * @param b B value
 * @param method Method to be used for the conversion (0: Lightness method (worst), 1: Average method (similar to reflected light mode), 2: Luminosity method (similar to human vision))
 * @return float 
 */
float rgb2grayscale(long r, long g, long b, int method){
	if (method == 0){
		return (max(r / 255, max(g / 255, b / 255)) + min(r / 255, min(g / 255, b / 255))) / 2;
	} else if (method == 1){
		return (r + g + b) / 3;
	} else {
		return r * 0.3 / 255 + g * 0.59 / 255 + b * 0.11 / 255;
	}
}