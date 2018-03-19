// A SHOT on Goal. This version is tailored for a 12m kick distance.
// This version has the revised timers - please use this version.
// PLEASE USE THIS VERSION FOR THE ASSIGNMENT (BUT NOT ON x86!).
// This version is provided for convenience but is identical to the PS3 project code.
// Code specific to this version is indicated by comments starting //***SHOT

/* In the game of rugby a touchdown is converted by booting the ball over the 3m high goal cross bar from some point
on the pitch, but orthogonal to the try line where the touchdown occured.
This program calculates how fast and at what vertical angle you would have to kick the ball to get it over the cross bar from
a set distance (which is input).
It does this by cycling through a series of angles, and figuring out using standard trajectory equations
whether the ball would make it over the bar (ignoring factors such as air resistance, ball spin, air
density, and so on) until the first working combination is found. Using the kick speed and angle thus found, a series of x:y
coordinates of the ball's trajectory path are calculated and saved in an array. It is assumed that the ball is kicked on target
and goes through the uprights.
A 'display' function then plots these on a rugby-stylised graph for inspection (see screen layout later).
Units are (m) metres, (s) seconds, (deg) degrees, all float values.

Adrian Oram, March 2017 (not a rugby supporter, so don't ask!)
*/

#define yourName     "Mitch Peake && Joe Garlick"				// Please change these as appropriate!
#define yourTeamName "Bad Mother Ruckers"

#define _PS3		// Build for PS3 system otherwise x86 if commented out.
//#define _trace		// comment out to remove trace output used for testing
//#define _longTrace	// comment out to avoid lengthy trace of trial height calculations

#ifdef _PS3
#include <sys/sys_time.h>	// for PS3 timers  
#else
#include "hr_time.h"		//for x86 timers 
#include <fstream>			// for file I/O (tricky on PS3)
#endif // _PS3

#include <math.h>			// for COS, TAN, etc
#include <string>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <stdio.h>
#include <iomanip>
using namespace std;

// Function Prototypes
template <class T>
string ToString(T);
void getDistanceToKick(float*);
bool findSHOTonGoalSpeedAndAngle(float*, float*, float);
void generateFlightPath(float, float);
void showFlightPathResults(float, float, float);

// Timing stuff
#ifdef _PS3
long long get_time_in_ticks(void);		// home brew system clock reader (1 line of assembly).
long long start, stop;					// Use this with Time Base register method of timing
const double ticksToSeconds(79.8F);		// PS3 clocks at 3.2 GHz this is 24.9x that. 
#else
CStopWatch timer;				// x86
#endif // !_PS3

const int repeats = 100000;				// Repeat timings to help avoid exceptions, and record fastest times seen.
// On PS3 should be less of a problem as little else running.
long long SpeedAndAngleTime(1000000);	// Set high intially.
long long GenerateFlightPathTime(1000000);

// Some constants
const float g(9.81F);					// (m/s/s) gravity
const float Pi(3.14159265358979323846F);// This value stolen from M_PI defines in Math.h) - used to convert degrees to radians
const float dataEnd = -1.0F;			// End of data marker
const int x = 0;						// coordinate system
const int y = 1;
#define SPACE ' '
const double micro(1.0e6F);				// scaling factor to give microseconds
const double milli(1.0e3F);				// scaling factor to give milliseconds


// Kick metrics
//***SHOT const float minAngle(15.0F);	// (deg) pretty low!
//***SHOT const float maxAngle(45.0F);	// (deg) pretty steep. It becomes increasingly difficult to put enough energy into a ball at angles above 45 deg.
//***SHOT Limit range of angles to help speed things up, and help elsewhere?
const float minAngle(18.0F);	// (deg) ***SHOT We know the answer for kick distance 12.0m is 20 degrees.
const float maxAngle(23.0F);	// (deg)

int angleSelected;
int speedSelected;

const float minSpeed(5.0F);		// (m/s) pretty pathetic!
const float maxSpeed(32.0F);	// (m/s) who let Superman on the pitch!? Research indicates that 26 +/-1.7 m/s is optimal kick speed.
const float maxHeight(8.5F);	// (m)   trajectories above this height can't be displayed (out the park!)


const float xValueSquaredTimesGravity[104] = {
	-2.4525,
	-9.81,
	-22.0725,
	-39.24,
	-61.3125,
	-88.29,
	-120.173,
	-156.96,
	-198.652,
	-245.25,
	-296.753,
	-353.16,
	-414.473,
	-480.69,
	-551.813,
	-627.84,
	-708.773,
	-794.61,
	-885.353,
	-981,
	-1081.55,
	-1187.01,
	-1297.37,
	-1412.64,
	-1532.81,
	-1657.89,
	-1787.87,
	-1922.76,
	-2062.55,
	-2207.25,
	-2356.85,
	-2511.36,
	-2670.77,
	-2835.09,
	-3004.31,
	-3178.44,
	-3357.47,
	-3541.41,
	-3730.25,
	-3924,
	-4122.65,
	-4326.21,
	-4534.67,
	-4748.04,
	-4966.31,
	-5189.49,
	-5417.57,
	-5650.56,
	-5888.45,
	-6131.25,
	-6378.95,
	-6631.56,
	-6889.07,
	-7151.49,
	-7418.81,
	-7691.04,
	-7968.17,
	-8250.21,
	-8537.15,
	-8829,
	-9125.75,
	-9427.41,
	-9733.97,
	-10045.4,
	-10361.8,
	-10683.1,
	-11009.3,
	-11340.4,
	-11676.4,
	-12017.3,
	-12363.1,
	-12713.8,
	-13069.4,
	-13429.9,
	-13795.3,
	-14165.6,
	-14540.9,
	-14921,
	-15306.1,
	-15696,
	-16090.9,
	-16490.6,
	-16895.3,
	-17304.8,
	-17719.3,
	-18138.7,
	-18563,
	-18992.2,
	-19426.3,
	-19865.3,
	-20309.2,
	-20758,
	-21211.7,
	-21670.3,
	-22133.8,
	-22602.2,
	-23075.6,
	-23553.8,
	-24037,
	-24525,
	-25018,
	-25515.8,
	-26018.6,
	-26526.2
};
const float cosSpeedsinverse[11][55] = {
	{ 0.0221115, 0.0182739, 0.0153552, 0.0130837, 0.0112814, 0.00982731, 0.00863729, 0.00765102, 0.00682452, 0.00612506, 0.00552786, 0.00501394, 0.00456848, 0.00417986, 0.00383879, 0.00353783, 0.00327093, 0.00303312, 0.00282034, 0.00262919, 0.00245683, 0.00230088, 0.00215932, 0.00203044, 0.00191276, 0.00180502, 0.00170613, 0.00161515, 0.00153126, 0.00145374, 0.00138197, 0.00131538, 0.00125348, 0.00119586, 0.00114212, 0.00109192, 0.00104496, 0.00100097, 0.000959699, 0.000920927, 0.000884458, 0.000850114, 0.000817731, 0.000787165, 0.00075828, 0.000730957, 0.000705085, 0.000680562, 0.000657297, 0.000635204, 0.000614207, 0.000594234, 0.00057522, 0.000557104, 0.00053983 },
	{ 0.0222391, 0.0183794, 0.0154438, 0.0131592, 0.0113465, 0.00988404, 0.00868714, 0.00769518, 0.00686391, 0.00616041, 0.00555977, 0.00504288, 0.00459485, 0.00420398, 0.00386095, 0.00355825, 0.0032898, 0.00305063, 0.00283662, 0.00264436, 0.00247101, 0.00231416, 0.00217179, 0.00204216, 0.0019238, 0.00181544, 0.00171598, 0.00162448, 0.0015401, 0.00146214, 0.00138994, 0.00132297, 0.00126072, 0.00120276, 0.00114871, 0.00109823, 0.001051, 0.00100675, 0.000965238, 0.000926242, 0.000889563, 0.00085502, 0.000822451, 0.000791708, 0.000762657, 0.000735176, 0.000709154, 0.00068449, 0.00066109, 0.00063887, 0.000617752, 0.000597664, 0.00057854, 0.000560319, 0.000542946 },
	{ 0.0223712, 0.0184886, 0.0155356, 0.0132374, 0.0114139, 0.00994277, 0.00873876, 0.00774091, 0.0069047, 0.00619702, 0.00559281, 0.00507284, 0.00462216, 0.00422897, 0.00388389, 0.0035794, 0.00330935, 0.00306876, 0.00285347, 0.00266007, 0.00248569, 0.00232791, 0.00218469, 0.00205429, 0.00193523, 0.00182622, 0.00172618, 0.00163413, 0.00154925, 0.00147082, 0.0013982, 0.00133083, 0.00126821, 0.00120991, 0.00115554, 0.00110475, 0.00105724, 0.00101273, 0.000970974, 0.000931746, 0.000894849, 0.000860101, 0.000827338, 0.000796413, 0.000767189, 0.000739545, 0.000713368, 0.000688557, 0.000665019, 0.000642667, 0.000621423, 0.000601216, 0.000581978, 0.000563649, 0.000546173 },
	{ 0.022508, 0.0186017, 0.0156306, 0.0133183, 0.0114837, 0.0100036, 0.00879219, 0.00778823, 0.00694691, 0.0062349, 0.005627, 0.00510385, 0.00465041, 0.00425482, 0.00390764, 0.00360128, 0.00332959, 0.00308752, 0.00287092, 0.00267634, 0.00250089, 0.00234214, 0.00219805, 0.00206685, 0.00194706, 0.00183739, 0.00173673, 0.00164412, 0.00155873, 0.00147982, 0.00140675, 0.00133896, 0.00127596, 0.00121731, 0.0011626, 0.00111151, 0.00106371, 0.00101892, 0.00097691, 0.000937443, 0.00090032, 0.000865359, 0.000832396, 0.000801282, 0.000771879, 0.000744066, 0.00071773, 0.000692767, 0.000669084, 0.000646596, 0.000625222, 0.000604891, 0.000585536, 0.000567095, 0.000549512 },
	{ 0.0226495, 0.0187186, 0.0157288, 0.0134021, 0.0115559, 0.0100664, 0.00884746, 0.00783719, 0.00699058, 0.0062741, 0.00566237, 0.00513594, 0.00467965, 0.00428157, 0.0039322, 0.00362392, 0.00335052, 0.00310693, 0.00288897, 0.00269316, 0.00251661, 0.00235687, 0.00221186, 0.00207984, 0.0019593, 0.00184894, 0.00174765, 0.00165445,0.00156852, 0.00148912, 0.00141559, 0.00134738, 0.00128398, 0.00122496, 0.00116991, 0.00111849, 0.00107039, 0.00102533, 0.000983051, 0.000943336, 0.000905979, 0.000870799, 0.000837629, 0.000806318, 0.000776731, 0.000748743, 0.000722241, 0.000697122, 0.00067329, 0.00065066, 0.000629152, 0.000608694, 0.000589217, 0.00057066, 0.000552966 },
	{ 0.0227958, 0.0188395, 0.0158304, 0.0134886, 0.0116305, 0.0101315, 0.00890461, 0.00788782, 0.00703574, 0.00631462, 0.00569895, 0.00516911, 0.00470987, 0.00430922, 0.0039576, 0.00364733, 0.00337216, 0.003127, 0.00290763, 0.00271056, 0.00253287, 0.00237209, 0.00222615, 0.00209328, 0.00197195, 0.00186088, 0.00175893, 0.00166514,0.00157866, 0.00149874, 0.00142474, 0.00135609, 0.00129228, 0.00123287, 0.00117747, 0.00112572, 0.00107731, 0.00103195, 0.000989401, 0.000949429, 0.000911832, 0.000876424, 0.00084304, 0.000811527, 0.000781749, 0.00075358, 0.000726907, 0.000701625, 0.00067764, 0.000654863, 0.000633217, 0.000612626, 0.000593023, 0.000574346, 0.000556538 },
	{ 0.022947, 0.0189645, 0.0159354, 0.0135781, 0.0117077, 0.0101987, 0.00896368, 0.00794015, 0.00708242, 0.00635652, 0.00573676, 0.00520341, 0.00474112, 0.00433781, 0.00398386, 0.00367153, 0.00339453, 0.00314774, 0.00292692, 0.00272854, 0.00254967, 0.00238783, 0.00224092, 0.00210717, 0.00198504, 0.00187323, 0.0017706, 0.00167619, 0.00158913, 0.00150868, 0.00143419, 0.00136508, 0.00130085, 0.00124105, 0.00118528, 0.00113319, 0.00108445, 0.0010388, 0.000995965, 0.000955728, 0.000917881, 0.000882239, 0.000848633, 0.000816911, 0.000786935, 0.00075858, 0.000731729, 0.00070628, 0.000682135, 0.000659208, 0.000637418, 0.00061669, 0.000596957, 0.000578157, 0.00056023 },
	{ 0.0231033, 0.0190936, 0.016044, 0.0136706, 0.0117874, 0.0102681, 0.00902473, 0.00799422, 0.00713065, 0.00639981, 0.00577583, 0.00523885, 0.00477341, 0.00436735, 0.00401099, 0.00369653, 0.00341765, 0.00316918, 0.00294685, 0.00274712, 0.00256703, 0.00240409, 0.00225618, 0.00212152, 0.00199856, 0.00188598, 0.00178266, 0.0016876, 0.00159995, 0.00151896, 0.00144396, 0.00137438, 0.00130971, 0.0012495, 0.00119335, 0.0011409, 0.00109184, 0.00104587, 0.00100275, 0.000962237, 0.000924132, 0.000888247, 0.000854412, 0.000822474, 0.000792295, 0.000763746, 0.000736713, 0.00071109, 0.000686781, 0.000663697, 0.000641759, 0.00062089, 0.000601023, 0.000582094, 0.000564046 },
	{ 0.0232647, 0.0192271, 0.0161561, 0.0137661, 0.0118698, 0.0103399, 0.00908779, 0.00805008, 0.00718048, 0.00644453, 0.00581619, 0.00527545, 0.00480677, 0.00439787, 0.00403902, 0.00372236, 0.00344153, 0.00319132, 0.00296744, 0.00276632, 0.00258497, 0.00242089, 0.00227195, 0.00213634, 0.00201252, 0.00189916, 0.00179512, 0.0016994, 0.00161113, 0.00152957, 0.00145405, 0.00138398, 0.00131886, 0.00125823, 0.00120169, 0.00114888, 0.00109947, 0.00105318, 0.00100975, 0.000968961, 0.00093059, 0.000894454, 0.000860383, 0.000828222, 0.000797831, 0.000769082, 0.00074186, 0.000716059, 0.00069158, 0.000668335, 0.000646243, 0.000625228, 0.000605222, 0.000586161,0.000567987 },
	{ 0.0234315, 0.0193648, 0.0162718, 0.0138648, 0.0119548, 0.010414, 0.00915291, 0.00810777, 0.00723193, 0.00649071, 0.00585786, 0.00531326, 0.00484121, 0.00442939, 0.00406796, 0.00374903, 0.00346619, 0.00321419, 0.00298871, 0.00278614, 0.0026035, 0.00243824, 0.00228823, 0.00215165, 0.00202694, 0.00191277, 0.00180798, 0.00171157, 0.00162268, 0.00154053, 0.00146447, 0.0013939, 0.00132831, 0.00126725, 0.0012103, 0.00115711, 0.00110735, 0.00106073, 0.00101699, 0.000975904, 0.000937258, 0.000900863, 0.000866548, 0.000834157, 0.000803548, 0.000774594, 0.000747177, 0.00072119, 0.000696536, 0.000673124, 0.000650874, 0.000629709, 0.000609559, 0.000590362, 0.000572057 },
	{ 0.0236036, 0.0195071, 0.0163914, 0.0139666, 0.0120426, 0.0104905, 0.00922015, 0.00816733, 0.00728505, 0.00653839, 0.00590089, 0.00535228, 0.00487677, 0.00446192, 0.00409784, 0.00377657, 0.00349165, 0.0032378, 0.00301066, 0.00280661, 0.00262262, 0.00245615, 0.00230504, 0.00216745, 0.00204183, 0.00192682, 0.00182126, 0.00172415, 0.0016346, 0.00155185, 0.00147522, 0.00140414, 0.00133807, 0.00127656, 0.00121919, 0.00116561, 0.00111548, 0.00106852, 0.00102446, 0.000983073, 0.000944143, 0.000907481, 0.000872913, 0.000840284, 0.000809451, 0.000780284, 0.000752665, 0.000726487, 0.000701652, 0.000678069, 0.000655655, 0.000634334, 0.000614037, 0.000594698, 0.000576259 }
};

const float speeds[55] = {
	0.04,
	0.0330579,
	0.0277778,
	0.0236686,
	0.0204082,
	0.0177778,
	0.015625,
	0.0138408,
	0.0123457,
	0.0110803,
	0.01,
	0.00907029,
	0.00826446,
	0.00756144,
	0.00694444,
	0.0064,
	0.00591716,
	0.00548697,
	0.00510204,
	0.00475624,
	0.00444445,
	0.00416233,
	0.00390625,
	0.00367309,
	0.00346021,
	0.00326531,
	0.00308642,
	0.00292184,
	0.00277008,
	0.00262985,
	0.0025,
	0.00237954,
	0.00226757,
	0.00216333,
	0.00206612,
	0.00197531,
	0.00189036,
	0.00181077,
	0.00173611,
	0.00166597,
	0.0016,
	0.00153787,
	0.00147929,
	0.00142399,
	0.00137174,
	0.00132231,
	0.00127551,
	0.00123115,
	0.00118906,
	0.0011491,
	0.00111111,
	0.00107498,
	0.00104058,
	0.00100781,
	0.000976563
};

const float cosAnglesValues[10] = {
	0.951057,
	0.948324,
	0.945519,
	0.942641,
	0.939693,
	0.936672,
	0.93358,
	0.930418,
	0.927184,
	0.92388
};

const float cosX2Inverse[10] = {
	0.552786,
	0.555977,
	0.559281,
	0.5627,
	0.566237,
	0.569895,
	0.573676,
	0.577583,
	0.581619,
	0.585786
};

const float tanAnglesValues[10] = {
	0.32492,
	0.334595,
	0.344328,
	0.354119,
	0.36397,
	0.373885,
	0.383864,
	0.39391,
	0.404026,
	0.414214
};



float kickSpeed;				// (m/s) calculated
float kickAngle;				// (deg) calculated

// Pitch metrics
const float minDistanceToGoal(5.0F);	// (m) Probably too close! Might get charged down!
const float maxDistanceToGoal(50.0F);	// (m) This is almost half a standard rugby pitch length.
const float crossBarHeight(3.0F);		// (m) 3m is the standard rugby cross bar height.
const float margin(0.5F);				// (m) allow for a margin of error getting it over.
const float goalPostHeight(7.0F);		// (m) Upto 16m usually. Don't make higher, it won't fit on display!
float distanceToGoal;					// (m) Kicking distance, input by user.

// Final flight path of ball as a series of x,y coordinates
const float deltaD(0.5F);			// (m) need a coordinate every tick of this distance, plus increment used for speed and angle.
const float deltaY(0.25F);			// (m) increment in the height direction (vertical exaggeration of x2)
const float yScale = 1.0F / deltaY;	// vertical scaling factor
const float xScale = 1.0F / deltaD;	// horizontal scaling factor

const int maxDataPoints = (int)((maxDistanceToGoal + 2.0F) / deltaD);	// =104, calculate a data point for each 0.5 metre along
// +2.0 so that ball appears beyond the goal at maxDistance

float flightPath[104 + 1][2] = 
{
	{ 0, 0.0 },
	{ 0.5, 0.0 },
	{ 1, 0.0 },
	{ 1.5, 0.0 },
	{ 2, 0.0 },
	{ 2.5, 0.0 },
	{ 3, 0.0 },
	{ 3.5, 0.0 },
	{ 4, 0.0 },
	{ 4.5, 0.0 },
	{ 5, 0.0 },
	{ 5.5, 0.0 },
	{ 6, 0.0 },
	{ 6.5, 0.0 },
	{ 7, 0.0 },
	{ 7.5, 0.0 },
	{ 8, 0.0 },
	{ 8.5, 0.0 },
	{ 9, 0.0 },
	{ 9.5, 0.0 },
	{ 10, 0.0 },
	{ 10.5, 0.0 },
	{ 11, 0.0 },
	{ 11.5, 0.0 },
	{ 12, 0.0 },
	{ 12.5, 0.0 },
	{ 13, 0.0 },
	{ 13.5, 0.0 },
	{ 14, 0.0 },
	{ 14.5, 0.0 },
	{ 15, 0.0 },
	{ 15.5, 0.0 },
	{ 16, 0.0 },
	{ 16.5, 0.0 },
	{ 17, 0.0 },
	{ 17.5, 0.0 },
	{ 18, 0.0 },
	{ 18.5, 0.0 },
	{ 19, 0.0 },
	{ 19.5, 0.0 },
	{ 20, 0.0 },
	{ 20.5, 0.0 },
	{ 21, 0.0 },
	{ 21.5, 0.0 },
	{ 22, 0.0 },
	{ 22.5, 0.0 },
	{ 23, 0.0 },
	{ 23.5, 0.0 },
	{ 24, 0.0 },
	{ 24.5, 0.0 },
	{ 25, 0.0 },
	{ 25.5, 0.0 },
	{ 26, 0.0 },
	{ 26.5, 0.0 },
	{ 27, 0.0 },
	{ 27.5, 0.0 },
	{ 28, 0.0 },
	{ 28.5, 0.0 },
	{ 29, 0.0 },
	{ 29.5, 0.0 },
	{ 30, 0.0 },
	{ 30.5, 0.0 },
	{ 31, 0.0 },
	{ 31.5, 0.0 },
	{ 32, 0.0 },
	{ 32.5, 0.0 },
	{ 33, 0.0 },
	{ 33.5, 0.0 },
	{ 34, 0.0 },
	{ 34.5, 0.0 },
	{ 35, 0.0 },
	{ 35.5, 0.0 },
	{ 36, 0.0 },
	{ 36.5, 0.0 },
	{ 37, 0.0 },
	{ 37.5, 0.0 },
	{ 38, 0.0 },
	{ 38.5, 0.0 },
	{ 39, 0.0 },
	{ 39.5, 0.0 },
	{ 40, 0.0 },
	{ 40.5, 0.0 },
	{ 41, 0.0 },
	{ 41.5, 0.0 },
	{ 42, 0.0 },
	{ 42.5, 0.0 },
	{ 43, 0.0 },
	{ 43.5, 0.0 },
	{ 44, 0.0 },
	{ 44.5, 0.0 },
	{ 45, 0.0 },
	{ 45.5, 0.0 },
	{ 46, 0.0 },
	{ 46.5, 0.0 },
	{ 47, 0.0 },
	{ 47.5, 0.0 },
	{ 48, 0.0 },
	{ 48.5, 0.0 },
	{ 49, 0.0 },
	{ 49.5, 0.0 },
	{ 50, 0.0 },
	{ 50.5, 0.0 },
	{ 51, 0.0 },
	{ 51.5, 0.0 },
	{ -1.0, -1.0 }
};





// x,y coords (m,m) of ball flight. The sequence terminates with 'dataEnd' if fewer than maxDataPoints used.

//************************************* MAIN ***********************************************************************
int main(void)
{
	bool foundCombo(false);

	//getDistanceToKick(&distanceToGoal);	// comment this out if required
	distanceToGoal = 12.0F;  //***SHOT use this rather than entering it each run!

	cout << "\nYou entered " << distanceToGoal << " metres. Looking for solution for kick speed and angle...";
	fflush(stdout);	//PS3 console fix

	for (int reps(0); reps < repeats; ++reps)
	{
#ifdef _PS3
		start = get_time_in_ticks();		//*** PS3
#else
		timer.startTimer();					//*** x86
#endif
		//*********************************************************************************************************
		foundCombo = findSHOTonGoalSpeedAndAngle(&kickSpeed, &kickAngle, distanceToGoal);

		//*********************************************************************************************************
#ifdef _PS3
		stop = get_time_in_ticks();				//*** PS3 Time Base register returns a tick count
		if ((stop - start) < SpeedAndAngleTime) SpeedAndAngleTime = (stop - start);		//*** PS3 record fastest time
#else
		timer.stopTimer();					//*** x86 timers return time in Seconds, so scale to microseconds
		if (timer.getElapsedTime() < SpeedAndAngleTime) SpeedAndAngleTime = (timer.getElapsedTime() * micro);	//*** x86 record fastest time
#endif
	}

	if (foundCombo)
	{
		cout << " solution found. Generating ball's flight path for display...";
#ifdef _trace
		cout << "\nSpeed=" << kickSpeed << " Angle =" << kickAngle;
#endif // _trace

		for (int reps(0); reps < repeats; ++reps)
		{
#ifdef _PS3
			start = get_time_in_ticks();		//*** PS3
#else
			timer.startTimer();					//*** x86
#endif
			//*********************************************************************************************************
			generateFlightPath(kickSpeed, kickAngle);

			//*********************************************************************************************************
#ifdef _PS3
			stop = get_time_in_ticks();			//*** PS3 Time Base register returns a tick count
			if ((stop - start) < GenerateFlightPathTime) GenerateFlightPathTime = (stop - start); 
#else
			timer.stopTimer();					//*** x86 timers return time in Seconds, so scale to microseconds
			if (timer.getElapsedTime() < GenerateFlightPathTime) GenerateFlightPathTime = (timer.getElapsedTime() * micro);	//*** x86 record fastest time
#endif
		}
		showFlightPathResults(kickSpeed, kickAngle, distanceToGoal);
	}
	else cout << "no solution found.\n";

	cout << "Done...";
#ifndef _PS3
	system("PAUSE");	// hold console on PC, OK to return on PS3 as console display persists.
#endif

	return(0);
}
//************************************* END MAIN ********************************************************************

// Figure out the first working combination of Speed and Angle.
// Using the lowest kick speed to begin with, increment through the angles from low to high until one is found that gets 
// the ball over the bar. If none found increase the speed and repeat. Rationale: it's better to use as little 
// energy in the kick as possible! DeltaD is used as the increment for both angle and speed.

bool findSHOTonGoalSpeedAndAngle(float* speed, float* angle, float x)
{
	// *** reminders ***
	//const float minAngle(15.0)
	//const float maxAngle(45.0)
	//const float minSpeed(5.0)
	//const float maxSpeed(32.0)
	//const float crossBarHeight(3.0)

	float nextAngle(minAngle);	// Start with shallowest angle...
	bool foundCombo(false);		// Found combination of speed and angle that gets ball over bar?

	float gravityTimesX(-g * x*x);

	int angleCounter(0);
	float crossbarPlusMargin = crossBarHeight + margin;

	while (!(foundCombo || (nextAngle > maxAngle)))				// Think de Morgan's Theory, perhaps.
	{
		float nextSpeed(minSpeed);

		float tanAngleRads(tanAnglesValues[angleCounter]);

		float XTimesTanAngleRads(x * tanAngleRads);

		float InverseTwoTimesSquareOfCosAngleRadTimesGravity(cosX2Inverse[angleCounter] * gravityTimesX);
		
		int speedCounter(0);

		do {
			float InversenextSpeedSquared(speeds[speedCounter]);
			
			float height(InversenextSpeedSquared * InverseTwoTimesSquareOfCosAngleRadTimesGravity + XTimesTanAngleRads);

			//cout << nextAngle<< " - ";

			if (height > crossbarPlusMargin)	// Success! 
			{
				*speed = nextSpeed;			// Record the working combination...
				*angle = nextAngle;
				angleSelected = angleCounter;
				speedSelected = speedCounter;
				foundCombo = true;			// ... and stop looking.
			} else {
				nextSpeed += deltaD;		// Otherwise try next speed up (+0.5 m/s).#
			}
			speedCounter++;
		} while (!(foundCombo || (nextSpeed > maxSpeed)));
		angleCounter++;
		nextAngle += deltaD;	// no joy, try next angle up (+0.5 degrees).
	}

	return (foundCombo);
}

// With metrics found, calculate the flight path coords. Uses 'flightPath[104][2]' array as global.
void generateFlightPath(float speed, float angle)
{
	//  ...reminders...
	//const float deltaD (0.5F)
	//const int maxDataPoints = 104
	//const float maxHeight(8.5F);	// (m) trajectories above this height can't be displayed (out the park!)

	float yValue(0.001F);	// ball is sitting on a tee just above the ground begin with, of course!
	float xValue(0.0F);		// ...and hasn't moved yet.

	float tanAngleRads = tanAnglesValues[angleSelected];
	float inverseOfCos = cosSpeedsinverse[angleSelected][speedSelected];


	int i(0);
	for (; i < maxDataPoints && (yValue > 0.0) && (yValue <= maxHeight); ++i)	// If height goes negative or too high, STOP!
	{

		flightPath[i][y] = yValue;
		xValue += deltaD;			// do for each increment tick across the pitch


		// find the 'y' (height) for each 'x' distance using the angle and speed previously found (same equation as above)
		yValue = (xValueSquaredTimesGravity[i] * inverseOfCos) + (xValue * tanAngleRads);
	}
	 /*Finished generating required data points, now mark end-of-data with -1.0 (dataEnd)*/

	asm volatile (
		"	la	6,%[xVal]										\n" //	load address of xValue into r 6
		"	la	7,%[xVal]										\n" //	load address of yValue into r 7
		"	mr	18,%[maxPts]									\n" //	move 104 into r 18

		"mtctr 18												\n" // set the counter to decrementing 104 times
	"for:														\n" //  
		"														\n" //	load address of yValue into r 7
	"endif1:													\n" // 
		"bc 16,0,for											\n" // 
		"														\n" //
		"														\n"	//
		"														\n" // 
		"														\n" // 
		"mtcrf 0x20,103											\n" //

		: [xVal] "=m"  (xValue),									// output list
		  [yVal] "=m" (yValue),
		  [tAR] "=m"[tanAngleRads],
		  [iCos] "=m"[inverseOfCos]

 		: [maxPts] "i" (maxDataPoints),	   // input list
		  [yVal] "i" (yValue),
		  [MaxHe] "i" (maxHeight)
		 
		: "r10", "r11", "fr6", "fr7", "r8" ,									// clobber list

		); // end of inline ASM
}

//************************************ Supporting functions *******************************************************
//*****************************************************************************************************************

// Display  a stylised flightpath of ball etc. Memory is cheap so lots of replicated chars used for simplicity. 
// The string array below holds all the fixed text - the data points and other information are superimposed on it.
// The graph displays with roughly a 5x vertical exaggeration, the scale ticks have a 2x vertical exaggeration.
// PLEASE DON'T ALTER ANYTHING IN THIS FUNCTION.

void showFlightPathResults(float speed, float angle, float distanceToGoal)
{
	string YaxisTitle1 = "\n\n Height (m) above pitch\n in 0.25m increments\t\t\t\t\t\tSHOT ON GOAL CALCULATOR ";
	string YaxisTitle2 =                            "\n (Vert. exag.~= x5) \t\t\t\t\t\t~~~~~~~~~~~~~~~~~~~~~~~\n\n";
	string XaxisTitle1 = "\n\t\t\t\tDistance (m) to ";
	string XaxisTitle2 = "'s goal in 0.5m increments\n\n";
	string goalPostTitle = "+ = cross bar";

	const int leftMargin = 13;		// Number of chars from left side of screen to '0' position, from where ball is kicked.
	const int bottomMargin = 2;		// ditto from bottom to pitch level.
	const int resultsTextPos = 29;	// How far in to position results data.
	const int graphLines = 37;		// How many lines of text there are in the array below.
	const int pitchLevel = graphLines - bottomMargin - 1; // get stuff on to pitch directly.

	// This array = 4.4KB ish. There are 121 characters in each line + '\0' terminator.

	// This string array is treated as a 2-D array of chars, with row [0] at the top. The 'X's are replaced by
	// the performance data.

	string graphDisplay[] = {

		" 8.5 | Time to solution (us)  X                                                                                        \n",
		"     | Time generating  (us)  X                                                                                        \n",
		" 8.0 |      Kick angle (deg)  X            .         .         .         .         .         .         .         .     \n",
		"     |      Kick speed (m/s)  X                                                                                        \n",
		" 7.5 |         X  Iterations                                                                                           \n",
		"     |                                                                                                                 \n",
		" 7.0 |       .         .         .         .         .         .         .         .         .         .         .     \n",
		"     |                                                                                                                 \n",
		" 6.5 |                                                                                                                 \n",
		"     |                                                                                                                 \n",
		" 6.0 |       .         .         .         .         .         .         .         .         .         .         .     \n",
		"     |                                                                                                                 \n",
		" 5.5 |                                                                                                                 \n",
		"     |                                                                                                                 \n",
		" 5.0 |       .         .         .         .         .         .         .         .         .         .         .     \n",
		"     |                                                                                                                 \n",
		" 4.5 |                                                                                                                 \n",
		"     |                                                                                                                 \n",
		" 4.0 |       .         .         .         .         .         .         .         .         .         .         .     \n",
		"     |                                                                                                                 \n",
		" 3.5 |                                                                                                                 \n",
		"     |                                                                                                                 \n",
		" 3.0 |       .         .         .         .         .         .         .         .         .         .         .     \n",
		"     |                                                                                                                 \n",
		" 2.5 |                                                                                                                 \n",
		"     |                                                                                                                 \n",
		" 2.0 |       .         .         .         .         .         .         .         .         .         .         .     \n",
		"     |                                                                                                                 \n",
		" 1.5 |                                                                                                                 \n",
		"     |                                                                                                                 \n",
		" 1.0 |  _Q             .         .         .         .         .         .         .         .         .         .     \n",
		"     | | |\\_o                                                                                                          \n",
		" 0.5 | o |____#                                                                                                        \n",
		"     |  /                                                                                                              \n",
		" 0.0 | /    BLAT!!                                                                                                     \n",
		"     +-##----^+++++++++|+++++++++|+++++++++|+++++++++|+++++++++|+++++++++|+++++++++|+++++++++|+++++++++|+++++++++|+++++\n",
		"             0         5         10        15        20        25        30        35        40        45        50    \n"
	};// Nb. the double '\\' for the kicker's arm isn't a mistake. See actual output...

	const char goalPost = '|';
	const char crossBar = '+';
	const char ball = 'O';

	// Insert timing and other data...
	const int fieldWidth = 6;	//...convert data to a string and insert in display; SPACE replaces the end NULL of the string.
	graphDisplay[0].replace(resultsTextPos, fieldWidth, ToString((double)SpeedAndAngleTime / ticksToSeconds) + SPACE);		// Time for finding the solution
	graphDisplay[1].replace(resultsTextPos, fieldWidth, ToString((double)GenerateFlightPathTime / ticksToSeconds) + SPACE);	// Time for generating data points
	graphDisplay[2].replace(resultsTextPos, fieldWidth, ToString(angle) + SPACE);		// Kick angle found
	graphDisplay[3].replace(resultsTextPos, fieldWidth, ToString(speed) + SPACE);		// Kick speed found
	graphDisplay[4].replace(11, fieldWidth, ToString(repeats) + SPACE);		// # of times calcs repeated

	//***SHOT Check angle & speed figures and highlight disparities. ********************************************************************
	//***SHOT For 12.0 metres distance, we're expecting Speed=30.5m/s and angle=20.0 degrees.
	//***SHOT We can check for equalities with these floats as they're definite figures.
	const float actualAngle(20.0F);
	const float actualSpeed(30.5F);
	angle == actualAngle ? graphDisplay[2].replace(resultsTextPos + 8, 2, "OK   ")
		: graphDisplay[2].replace(resultsTextPos + 8, fieldWidth + 13, "X " + ToString(actualAngle) + " expected ");	//***SHOT check angle
	speed == actualSpeed ? graphDisplay[3].replace(resultsTextPos + 8, 2, "OK")
		: graphDisplay[3].replace(resultsTextPos + 8, fieldWidth + 13, "X " + ToString(actualSpeed) + " expected ");	//***SHOT check speed
	//***SHOT ***************************************************************************************************************************

	// Insert trajectory data from flightPath array into the array above. Scale the height values then convert to INTs for array indexing.
	//***SHOT Check each data point and display incorrect ones as 'x' and plot correct ones as 'ball'. Only corrects for the data points 
	//***SHOT actually produced, so may appear short.
	//***SHOT This table contains the correct data points generated from the flight data for distance = 12.0m:
	int actualPoints[][2] = { { 13, 34 }, { 14, 34 }, { 15, 33 }, { 16, 32 }, { 17, 32 }, { 18, 31 }, { 19, 30 }, { 20, 30 }, { 21, 29 },
	{ 22, 28 }, { 23, 28 }, { 24, 27 }, { 25, 27 }, { 26, 26 }, { 27, 25 }, { 28, 25 }, { 29, 24 }, { 30, 24 }, { 31, 23 }, { 32, 23 },
	{ 33, 22 }, { 34, 22 }, { 35, 21 }, { 36, 21 }, { 37, 20 }, { 38, 20 }, { 39, 20 }, { 40, 19 }, { 41, 19 }, { 42, 18 }, { 43, 18 },
	{ 44, 18 }, { 45, 17 }, { 46, 17 }, { 47, 17 }, { 48, 16 }, { 49, 16 }, { 50, 16 }, { 51, 15 }, { 52, 15 }, { 53, 15 }, { 54, 15 },
	{ 55, 14 }, { 56, 14 }, { 57, 14 }, { 58, 14 }, { 59, 14 }, { 60, 13 }, { 61, 13 }, { 62, 13 }, { 63, 13 }, { 64, 13 }, { 65, 13 },
	{ 66, 13 }, { 67, 13 }, { 68, 13 }, { 69, 12 }, { 70, 12 }, { 71, 12 }, { 72, 12 }, { 73, 12 }, { 74, 12 }, { 75, 12 }, { 76, 12 },
	{ 77, 12 }, { 78, 12 }, { 79, 12 }, { 80, 13 }, { 81, 13 }, { 82, 13 }, { 83, 13 }, { 84, 13 }, { 85, 13 }, { 86, 13 }, { 87, 13 },
	{ 88, 13 }, { 89, 14 }, { 90, 14 }, { 91, 14 }, { 92, 14 }, { 93, 14 }, { 94, 15 }, { 95, 15 }, { 96, 15 }, { 97, 15 }, { 98, 16 },
	{ 99, 16 }, { 100, 16 }, { 101, 17 }, { 102, 17 }, { 103, 17 }, { 104, 18 }, { 105, 18 }, { 106, 18 }, { 107, 19 }, { 108, 19 },
	{ 109, 20 }, { 110, 20 }, { 111, 21 }, { 112, 21 }, { 113, 21 }, { 114, 22 }, { 115, 22 }, { 116, 23 } };

	bool good(true);		//***SHOT start optimistically
	for (int i(0); i < maxDataPoints && flightPath[i][x] != dataEnd; i++)
	{
		int tempY = (pitchLevel - (int)(yScale*flightPath[i][y])); //***SHOT get Y coord 
		int tempX = (leftMargin + (int)(xScale*flightPath[i][x])); //***SHOT get X coord

		//***SHOT check Y value (no real need to check X value as they should be fixed increments)
		if (tempY == actualPoints[i][y])
			graphDisplay[tempY][tempX] = ball;	//***SHOT Y value is fine, plot it
		else {
			graphDisplay[tempY][tempX] = 'x';					//***SHOT Y value is incorrect, plot as 'x'...
			graphDisplay[actualPoints[i][y]][tempX] = ball;		//***SHOT ...and plot correct one.
			good=false;											//***SHOT Not good!
		};
	}//***SHOT produce a speech bubble...
	if (good) graphDisplay[28].replace(7, 11, "LOOKS GOOD!");
	else	  graphDisplay[28].replace(10, 4, "DOH!");
	//***SHOT *****************************************************************************************************************

	int GoalPostXpos = (int)(xScale * distanceToGoal) + leftMargin; // absolute X coord of goal posts.

	// Insert goal post at required distance into array above.
	for (int i(0); i < (int)(goalPostHeight * yScale); i++)
	{
		graphDisplay[pitchLevel - i][GoalPostXpos] = goalPost;	// insert from ground upwards (watch it go up in a memory window!)
	};

	// Place the cross bar...
	graphDisplay[pitchLevel + 1 - (int)(crossBarHeight * yScale)][GoalPostXpos] = crossBar;

	// Produce the finished graph on the console...
	cout << YaxisTitle1 << '(' << yourName << ')' << YaxisTitle2;
	for (int i(0); i<graphLines; ++i) cout << graphDisplay[i];	// display the big picture...

	// Output enough spaces to get goal post title to correct position, aligning the '+' with the post...
	for (int i(0); i < GoalPostXpos; i++) cout << SPACE; cout << goalPostTitle;

	// Last but not least...
	cout << XaxisTitle1 << yourTeamName << XaxisTitle2;

	//*************************************************************************************
#ifdef _trace	// Display/save generated data...

#ifndef _PS3		// can only access files easily on PC
	ofstream flightData;
	flightData.open("FlightPathData.txt", ios::app);
	flightData << "\n\nFlight path data for distance " << distanceToGoal << "m, angle= " << angle
		<< " degrees, and speed= " << speed << " m/s:\n";
	flightData << setw(3);
#endif // !_PS3
	cout << "\n\nFlight path data:\n";
	cout << setw(3);
	for (int i(0); i < maxDataPoints && flightPath[i][x] != dataEnd; ++i)
	{
		cout << '{' << flightPath[i][x] << ',' << flightPath[i][y] << "},";
#ifndef _PS3
		flightData << '{' << flightPath[i][x] << ',' << flightPath[i][y] << "},";
#endif // !_PS3
	}
	cout << "\n\nGraph points affected:\n";
#ifndef _PS3
	flightData << "\n\nGraph points affected:\n";
#endif // !_PS3
	for (int i(0); i < maxDataPoints && flightPath[i][x] != dataEnd; ++i)
	{
		cout << '{' << (leftMargin + (int)(xScale*flightPath[i][x])) << ',' << (pitchLevel - (int)(yScale*flightPath[i][y])) << "},";
#ifndef _PS3
		flightData << '{' << (leftMargin + (int)(xScale*flightPath[i][x])) << ',' << (pitchLevel - (int)(yScale*flightPath[i][y])) << "},";
#endif // !_PS3
	}
#ifndef _PS3
	flightData.close();
#endif // !_PS3
#endif //_trace
	//*************************************************************************************
}


//*****************************************************************************************************************
void getDistanceToKick(float* distance)
{
	cout << "\nDistance to goal in metres, 5.0 min, 50.0 max?:\n";
	cin >> *distance;
	while (*distance < minDistanceToGoal || *distance > maxDistanceToGoal)
	{
		cout << "\nOutside acceptable range. Try again:";  cin >> *distance;
	}
}

//*****************************************************************************************************************
template <class T>					// converts INTs and FLOATs to a string of chars
string ToString(T some_value)
{
	ostringstream textVersion;
	textVersion << some_value;
	return textVersion.str();
}

//*****************************************************************************************************************
#ifdef _PS3
long long get_time_in_ticks()		// Read the PS3's time base (TB low) register and return in R3
{									// Note this isn't in seconds as it depends on frequency of update.
	asm volatile (" mftb 3 \n"		// It does seem that dividing by 79.8 would give micro seconds.
		:::"r3");		// R3 is the default return parameter (uses clobber list only)
};
#endif
//************************************* END OF PROGRAM ************************************************************

