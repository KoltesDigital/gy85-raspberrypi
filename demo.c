#include <fcntl.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/param.h>

#include "adxl345/adxl345.h"
#include "hmc5883l/hmc5883l.h"
#include "itg3200/itg3200.h"

#define DELAY 30000

static int aFile = 0;
static int cFile = 0;
static int gFile = 0;

static int closeAndExit(int code)
{
    close(aFile);
    close(cFile);
    close(gFile);
    return code;
}

static int running;

static void ctrlCHandler(int signum)
{
	running = 0;
}

static void setupHandlers(void)
{
    struct sigaction sa =
    {
        .sa_handler = ctrlCHandler,
    };

    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
}

struct Coordinate
{
    short data;
	float smoothed;
	float min;
	float max;
    float normalized;
};

#define LERP_RATIO (0.000005f * DELAY)

static float lerp(float a, float b, float t)
{
	return a + (b - a) * t;
}

static void initCoordinate(struct Coordinate *coordinate)
{
	coordinate->smoothed = coordinate->min = coordinate->max = coordinate->normalized = coordinate->data;
}

static void updateCoordinate(struct Coordinate *coordinate)
{
	coordinate->smoothed = lerp(coordinate->smoothed, coordinate->data, LERP_RATIO);
	coordinate->min = MIN(coordinate->min, coordinate->smoothed);
 	coordinate->max = MAX(coordinate->max, coordinate->smoothed);
    coordinate->normalized = (coordinate->smoothed - coordinate->min) / (coordinate->max - coordinate->min) * 2.0f - 1.0f;
}

static struct Coordinate aX, aY, aZ;
static struct Coordinate cX, cY, cZ;
static struct Coordinate gX, gY, gZ, gT;

#define MAX_ADAPTERS 2

#define CHECK(S) if (S) return -1;

static int aConfigure(int aFile)
{
    CHECK(ADXL345_Init(aFile, ADXL345_ID, true));

    struct ADXL345_DataFormat confDataFormat = {
        .range = ADXL345_RANGE_2G,
    };
    CHECK(ADXL345_ConfigureDataFormat(aFile, &confDataFormat));

    struct ADXL345_Power confPowerControl = {
        .measurement = true,
    };
    CHECK(ADXL345_ConfigurePower(aFile, &confPowerControl));

    return 0;
}

static int cConfigure(int cFile)
{
    CHECK(HMC5883L_Init(cFile, HMC5883L_ID, true));

    struct HMC5883L conf = {
        .gain = HMC5883L_GAIN_1090,
        .measurementMode = HMC5883L_MEASUREMENTMODE_NORMAL,
        .outputRate = HMC5883L_OUTPUTRATE_30,
        .samples = HMC5883L_SAMPLES_2,
    };
    CHECK(HMC5883L_Configure(cFile, &conf));

    CHECK(HMC5883L_SetContinuousMeasurement(cFile));

    return 0;
}

static int gConfigure(int gFile)
{
    CHECK(ITG3200_Init(gFile, ITG3200_ID, true));
    
    struct ITG3200_Acquisition confAcquisition = {
        .lowPassFilter = ITG3200_LOWPASSFILTER_42,
        .sampleRateDivider = 0,
    };
    CHECK(ITG3200_ConfigureAcquisition(gFile, &confAcquisition));
    
    struct ITG3200_Power confPower = {
        .clockSource = ITG3200_CLOCKSOURCE_PLL_X,
    };
    CHECK(ITG3200_ConfigurePower(gFile, &confPower));
    
    return 0;
}

int main(void)
{
	int adapter;
    char filename[20];

    for (adapter = 0; adapter < MAX_ADAPTERS; ++adapter)
    {
        snprintf(filename, 20, "/dev/i2c-%d", adapter);
		if (!access(filename, R_OK | W_OK))
			break;
    }

	if (adapter == MAX_ADAPTERS)
	{
		fprintf(stderr, "No I2C adapter found\n");
		return -1;
	}

    aFile = open(filename, O_RDWR);
    if (aConfigure(aFile))
    {
        fprintf(stderr, "Failed to initialize accelerometer\n");
        return closeAndExit(-1);
    }

    cFile = open(filename, O_RDWR);
    if (cConfigure(cFile))
    {
        fprintf(stderr, "Failed to initialize compass\n");
        return closeAndExit(-1);
    }

    gFile = open(filename, O_RDWR);
    if (gConfigure(gFile))
    {
        fprintf(stderr, "Failed to initialize gyroscope\n");
        return closeAndExit(-1);
    }

    ADXL345_ReadData(aFile, &aX.data, &aY.data, &aZ.data);
    HMC5883L_ReadData(cFile, &cX.data, &cY.data, &cZ.data);
    ITG3200_ReadData(gFile, &gX.data, &gY.data, &gZ.data);
    ITG3200_ReadTemperature(gFile, &gT.data);

	initCoordinate(&aX);
	initCoordinate(&aY);
	initCoordinate(&aZ);
	initCoordinate(&cX);
	initCoordinate(&cY);
	initCoordinate(&cZ);
	initCoordinate(&gX);
	initCoordinate(&gY);
    initCoordinate(&gZ);
    initCoordinate(&gT);

	setupHandlers();

	initscr();
	noecho();
	curs_set(false);

	running = 1;
	while (running)
	{
        ADXL345_ReadData(aFile, &aX.data, &aY.data, &aZ.data);
        HMC5883L_ReadData(cFile, &cX.data, &cY.data, &cZ.data);
        ITG3200_ReadData(gFile, &gX.data, &gY.data, &gZ.data);
        ITG3200_ReadTemperature(gFile, &gT.data);

        updateCoordinate(&aX);
        updateCoordinate(&aY);
        updateCoordinate(&aZ);
        updateCoordinate(&cX);
        updateCoordinate(&cY);
        updateCoordinate(&cZ);
        updateCoordinate(&gX);
        updateCoordinate(&gY);
        updateCoordinate(&gZ);
        updateCoordinate(&gT);

		clear();

		mvprintw(0, 0, "%16s: %8d X %8d Y %8d Z\n", "Accelerometer", aX.data, aY.data, aZ.data);
		mvprintw(1, 0, "%16s: %8.1f X %8.1f Y %8.1f Z\n", "Smoothed", aX.smoothed, aY.smoothed, aZ.smoothed);
		mvprintw(2, 0, "%16s: %8.1f X %8.1f Y %8.1f Z\n", "Min", aX.min, aY.min, aZ.min);
        mvprintw(3, 0, "%16s: %8.1f X %8.1f Y %8.1f Z\n", "Max", aX.max, aY.max, aZ.max);
        mvprintw(4, 0, "%16s: %8.1f X %8.1f Y %8.1f Z\n", "Normalized", aX.normalized, aY.normalized, aZ.normalized);

		mvprintw(6, 0, "%16s: %8d X %8d Y %8d Z\n", "Compass", cX.data, cY.data, cZ.data);
		mvprintw(7, 0, "%16s: %8.1f X %8.1f Y %8.1f Z\n", "Smoothed", cX.smoothed, cY.smoothed, cZ.smoothed);
		mvprintw(8, 0, "%16s: %8.1f X %8.1f Y %8.1f Z\n", "Min", cX.min, cY.min, cZ.min);
        mvprintw(9, 0, "%16s: %8.1f X %8.1f Y %8.1f Z\n", "Max", cX.max, cY.max, cZ.max);
        mvprintw(10, 0, "%16s: %8.1f X %8.1f Y %8.1f Z\n", "Normalized", cX.normalized, cY.normalized, cZ.normalized);

		mvprintw(12, 0, "%16s: %8d X %8d Y %8d Z\n", "Gyroscope", gX.data, gY.data, gZ.data);
		mvprintw(13, 0, "%16s: %8.1f X %8.1f Y %8.1f Z\n", "Smoothed", gX.smoothed, gY.smoothed, gZ.smoothed);
		mvprintw(14, 0, "%16s: %8.1f X %8.1f Y %8.1f Z\n", "Min", gX.min, gY.min, gZ.min);
        mvprintw(15, 0, "%16s: %8.1f X %8.1f Y %8.1f Z\n", "Max", gX.max, gY.max, gZ.max);
        mvprintw(16, 0, "%16s: %8.1f X %8.1f Y %8.1f Z\n", "Normalized", gX.normalized, gY.normalized, gZ.normalized);

		mvprintw(18, 0, "%16s: %8d %8.1f °C\n", "Temperature", gT.data, ITG3200_ConvertTemperature(gT.data));
        mvprintw(19, 0, "%16s: %8.1f %8.1f °C\n", "Smoothed", gT.smoothed, ITG3200_ConvertTemperature(gT.smoothed));
        mvprintw(20, 0, "%16s: %8.1f %8.1f °C\n", "Min", gT.min, ITG3200_ConvertTemperature(gT.min));
        mvprintw(21, 0, "%16s: %8.1f %8.1f °C\n", "Max", gT.max, ITG3200_ConvertTemperature(gT.max));
        mvprintw(22, 0, "%16s: %8.1f\n", "Normalized", gT.normalized);

		refresh();

		usleep(DELAY);
	}

    endwin();
	
    close(aFile);
    close(cFile);
    close(gFile);

    return 0;
}
