// This file contains code which samples a signal on a PIC32MX440F Pinguino board, and converts it to a visualization on a ledcube.
// The conversion is done by a Constant Q Transform, very similar to a Fourier Transform, but more suitable to audio signals aimed at the human ear.
// This code is written by HolKann and Ratmir and has no copyright on it.

// As with any C code, the code is best understood bottom-up. These are the methods in reverse order:
// void setup()
//   Runs once at start-up
// void loop()
//   The main outer loop of the program. All code except the code in setup() or interrupt is called by this loop
// void calculateSPIData()
//   Given a chosen effect, and an amplitude for each frequency filter, this method calculates the bits to be passed to the SPI, which decides which leds are on/off.
// void adjustInputs()
//   Reads the input signals, excluding the audio signal. Things like brightness, amplification, kind of effect, max/min filter frequency etc. are read from the pins.

// void set*Filled*()
//   These are a bunch of methods simply denoting what depths should contain activated low leds. Part of the possible effects
// void Tmr4Interrupt()
//   The interrupt method, in which the signal is sampled and the screen refreshed.
// void adjustLayers()
//   Activates a layer of leds, which puts all leds in a certain layer on or off.
// void chooseEffect()
//   Chooses the current effect, based on nr of interrupts passed, and effect chosen by user.

// void preprocess_filters()
//   Calculates the size of the filters, as well as other constants needed for the CQT. Has to be called each time the max frequency or min frequency value is changed by the user.
// void cqt()
//   Calculates the CQT. Must be very efficient :)
// fixedpoint hamming(int m, int k)
//   Calculates a hamming window value for one sample.

// effectfuncs(int * values) such as backhigh_smooth, ..., sinewave
//   Calculate the heighth of each led column given the frequencies calculated by the CQT and the effect chosen by the user.
// void pushBack(u32 time, int *values)
//   For some effects, it is needed to give the leds periodically the impression of being pushed back. This method takes care of that.
// int applyDecay(int new, int old)
//   Helper method calculating some value that can be decayed.
// void setMainDepth(int *values, int mainDepth)
//   The main depth is the depth on which the other depths for an effect depend. This method calculates the led height of the main depth.

// fixedpoint approxCos(fixedpoint in)
//   Calculates a fixed-point approximation of the cos function
// int approxSin(int x)
//   Calculates a fixed-point approximation of the sin function
// int fp_abs(int in) 
//   Calculates an efficient abs function for integers
// int readPin(u32 pinNr)
//   Reads a given pin by taking the median of multiple samples
// void initializeOutput(pin, value)
//   Initializes pins to function as output pin. 
// void insertion_sort(int *array, int n) 
//   Sorts an array using simple insertion sort.

// void ISR_wrapper_vector_16(void) __attribute__ ((section(".vector_16")));
// void ISR_wrapper_vector_16(void) 
// void Tmr4Interrupt(void) __attribute__ ((interrupt));
// void init_timer4(void) 
//   Methods used to configure the interrupt.

// NOTE: read data from CDC by "sudo cat /dev/ttyACM0"
// NOTE: Do not use "millis()", as this seems to conflict with the interrupt. Instead we count the number of interrupts. This is about 14 each millisecond (because of 14 KHz interrupt frequency)
// NOTE: Automatic indentation command: "indent frequency_printer2.pde -kr -br -brf -brs -l10000"

#include <spi.c>
#include <interrupt.c>

//#define PIC32MX440F

#define RegEnable 5
// NOTE: MuxEnable is active if 0
#define MuxEnable 4
#define MUX0 1
#define MUX1 0
#define MUX2 3
#define MUX3 6
#define DATA 11
#define CLOCK 13
#define DECAY_PIN A1
#define INTENSITY_PIN A2
#define AMPLITUDE_PIN A3
#define ANALOG_PIN A6
#define HIGHFREQ_PIN A5
#define LOWFREQ_PIN A4
#define EFFECT_PIN A0
#define FLIPMONKEY 7
#define SATURATION_PIN 10
#define BUZZ_PIN 12

#define nrOfLayers 16
#define nrOfBands 16
#define maxDepth 5
#define ledOn 0

// PWM pins only are the PIN 11 and 12.

#define NRSAMPLES (1024)
#define LOWFREQDIV 8
#define fixedpoint int
#define PRECISION 10
// NOTE: Higher than 10 might give overflow for 32-bit numbers when multiplying...
#define SCALE (1<<PRECISION)
#define int2PI (1<<13)		// So in our book, a circle is a full 8192 units long. The sinus functions is based on this property!
#define ALPHA ((7<<PRECISION)/13)	// 0.53836*(1<<PRECISION)
#define BETA ((6<<PRECISION)/13)	// 1-0.53836*(1<<PRECISION)
#define MEM 3			// the number of memorized old sampling values, should always be odd
#define FREQS 16
#define FSAMPLE 14200
#define lowFreqBound 20
#define highFreqBound 7000
#define refreshRate 80
#define MAXCYCLE (FSAMPLE/16/refreshRate)
// so with a decay of 1, every 1<<DECAYPRECISION th frame the height is lowered by 1
#define DECAYPRECISION 10
#define REDUNDANCY 11		// the amount of samples to read one pin

#define NREFFECTS 27
#define MAXTOTALSAMPLES 2048

void ISR_wrapper_vector_16(void) __attribute__ ((section(".vector_16")));
// Put the ISR_wrapper in the good place

void ISR_wrapper_vector_16(void) {
    Tmr4Interrupt();
}

// ISR_wrapper will call the Tmr1Interrupt()

void Tmr4Interrupt(void) __attribute__ ((interrupt));
// Tmr1Interrupt is declared as an interrupt routine

// configure timer 4 
void init_timer4(void) {
    IntConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);	// interrupt mode (interrupt.c)
    T4CON = 0;			// reset timer 1 configuration
    TMR4 = 0;			// reset timer 1 counter register
    // adjust this to increase interrupt rate used for sampling
    PR4 = 0x0B00;		// define the preload register
    //0x1000 is 10.3 KHz
    //0x0B00 is 14.2 KHz
    //0x0A00 is 15.4 KHz
    //0x0900 is 17.3 KHz
    //0x0800 is 20.3 KHz
    IPC4SET = 0x5;		// select interrupt priority and sub-priority
    IFS0CLR = 0x10000;
    IEC0SET = 0x10000;		// enable timer 1 interrupt
    T4CONSET = 0x8000;		// start timer 1
}

u32 amplitude;			// amplification of signal
u32 intensity, previousIntensity;	// input intensity
u32 layerIntensity[nrOfLayers];	// intensity for each layer
u32 layercounter;		// counts the current layer being refreshed
int decay;			// decay of signal
unsigned int oldMinF, oldMaxF, F0, F16;	// minimum and maximum input frequencies
unsigned int effect, old_effect_read;	// input effect
unsigned int nrInterrupts, nrInterruptsSinceEffectChange, nrInterruptsSinceSignal;	// interruptcounters
int depth_on[maxDepth];		// signifying which depths should be "on" (aka all leds up to the top frequency led will be activated)
int effectMap[NREFFECTS];	// giving the order in which effects are played
typedef void (*effectFunc) (int *);	// type definition for effect method pointers
effectFunc effectFuncs[NREFFECTS];	// index for each effect method pointer
typedef void (*fillFunc) ();	// type definition for fill method pointers
fillFunc fillFuncs[NREFFECTS];	// index for each fill method pointer

char calculationFlag;		// denotes whether new frequencies should be calculated
u16 spiData[nrOfLayers][5];	// bitwise data for the spi
u16 spiData2[nrOfLayers][5];	// bitwise data for the spi
u16 *spiPointer = spiData;	// pointer to which data the spi should use

int leds[nrOfBands][maxDepth];	// on/off values for leds. The ith bit of each int denotes whether the ith led on a certain band and depth is turned on or off.

int latestSample = 0;		// most recent sample value
int signal[NRSAMPLES];		// current sample signal
int signal_lowfreq[NRSAMPLES];	// current sample signal, with a lower samplin frequency (see LOWFREQDIV)
int freqs[FREQS];		// frequencies for each band/filter
u32 lowfreq_endIndex = 0;	// index stating which frequency filters use low frequency samplin

unsigned int Freq[FREQS + 1];	// lower and upper frequency for each filter/band
unsigned int Div[FREQS];	// sample frequency divider for each filter
unsigned int NFreq[FREQS];	// number of samples needed for each filter
unsigned fixedpoint twoPiQ;	// 2*pi*Q value for the CQT (Constant Q Transform)

int band_it;			// iterator variable used in effect methods. For some reason, declaring this variable in each method significantly increases compiled code size, so it's declared here.

//##############Generic Methods###############

void insertion_sort(int *array, int n) {
    int c, d, t;
    for (c = 1; c < n; ++c) {
	d = c;
	while (d > 0 && array[d] < array[d - 1]) {
	    t = array[d];
	    array[d] = array[d - 1];
	    array[d - 1] = t;
	    --d;
	}
    }
}

void initializeOutput(pin, value) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, value);
}

int readPin(u32 pinNr) {
    int sampledata[REDUNDANCY];
    int i;
    for (i = 0; i < REDUNDANCY; ++i) {
	sampledata[i] = analogRead(pinNr);
    }
    insertion_sort(sampledata, REDUNDANCY);
    // return the median:
    return sampledata[REDUNDANCY / 2];
}

// fast abs(int) calculation
int fp_abs(int in) {
    fixedpoint mask = in >> 31;
    return (mask + in) ^ mask;
}

#define qN 11
#define qA PRECISION
#define qP 15
#define qR (2*qN-qP)
#define qS (qN+qP+1-qA)

int approxSin(int x) {
    // S(x) = x * ( (3<<p) - (x*x>>r) ) >> s
    // n : Q-pos for quarter circle             11, so full circle is 2^13 long
    // A : Q-pos for output                     10
    // p : Q-pos for parentheses intermediate   15
    // r = 2n-p                                  7
    // s = A-1-p-n                              17

    x = x << (30 - qN);		// resize to pi range
    // shift to full s32 range (Q13->Q30)

    if ((x ^ (x << 1)) < 0)	// test for quadrant 1 or 2
	x = (1 << 31) - x;

    x = x >> (30 - qN);
    return (x * ((3 << qP) - (x * x >> qR)) >> qS);
}

fixedpoint approxCos(fixedpoint in) {
    return approxSin((int2PI >> 2) - in);
}

// ----------------------Program Logic------------------

int applyDecay(int new, int old) {
    if (new >= old) {
	return (2 * new + old) / 3;
    } else {
	return (old * decay + new * (1024 - decay)) / 1024;
    }
}

void setMainDepth(int *values, int mainDepth) {
    for (band_it = 0; band_it < nrOfBands; ++band_it) {
	int newValue = (values[band_it] << DECAYPRECISION);
	leds[band_it][mainDepth] = applyDecay(newValue, leds[band_it][mainDepth]);
	if (leds[band_it][mainDepth] < 1) {
	    leds[band_it][mainDepth] = 1;
	} else if (leds[band_it][mainDepth] > (16 << DECAYPRECISION) - 1) {
	    leds[band_it][mainDepth] = (16 << DECAYPRECISION) - 1;
	}
    }
}

#define PERIOD 140000		// about 10s before pushing back
#define NRPHASES 4
#define rollbacktime 2500
int pushBackPhase = 0;

void pushBack(u32 time, int *values) {
    setAllUnfilled();
    int d;
    if (time / (rollbacktime / NRPHASES) >= pushBackPhase) {
	for (d = maxDepth - 1; d > pushBackPhase; --d) {
	    for (band_it = 0; band_it < nrOfBands; ++band_it) {
		leds[band_it][d] = leds[band_it][d - 1];
	    }
	}
	for (d = 0; d <= pushBackPhase && d < maxDepth; ++d) {
	    for (band_it = 0; band_it < nrOfBands; ++band_it) {
		leds[band_it][d] = -1;
	    }
	}
	++pushBackPhase;
    }
}

// %%%%%%%%%%%%%%%%%%%Effects%%%%%%%%%%%%%%%%%%%

// takes an int vector of length 16, and makes values[n] leds burn in column n
void backHigh_smooth(int *values) {
    setMainDepth(values, maxDepth - 1);

    int d;
    for (d = maxDepth - 2; d >= 0; --d) {
	for (band_it = 0; band_it < nrOfBands; ++band_it) {
	    leds[band_it][d] = leds[band_it][d + 1] * 800 / 1024 + 1;
	    if (leds[band_it][d] < 1) {
		leds[band_it][d] = 1;
	    } else if (leds[band_it][d] > (16 << DECAYPRECISION) - 1) {
		leds[band_it][d] = (16 << DECAYPRECISION) - 1;
	    }
	}
    }
}

// takes an int vector of length 16, and makes values[n] leds burn in column n
void midHigh_smooth(int *values) {
    int mainBand = maxDepth / 2;
    setMainDepth(values, mainBand);

    // sideUpdate should be less than 2014 and more than 0.
    // Increasing sideUpdate increases the speed with which the sides follow the middle
    int sideUpdate = 300;
    int d;
    for (band_it = 0; band_it < nrOfBands; ++band_it) {
	for (d = mainBand - 1; d >= 0; --d) {
	    leds[band_it][d] = (leds[band_it][d] * (1024 - sideUpdate) + sideUpdate * leds[band_it][d + 1]) / 1024 + 1;
	    leds[band_it][maxDepth - d - 1] = (leds[band_it][maxDepth - d - 1] * (1024 - sideUpdate) + sideUpdate * leds[band_it][maxDepth - d - 2]) / 1024 + 1;
	    if (leds[band_it][d] < 1) {
		leds[band_it][d] = 1;
	    } else if (leds[band_it][d] > (16 << DECAYPRECISION) - 1) {
		leds[band_it][d] = (16 << DECAYPRECISION) - 1;
	    }
	}
    }
}

void midHigh_with_fall(int *values) {
    int d;
    u32 time = nrInterrupts % PERIOD;
    if (time < rollbacktime) {
	pushBack(time, values);
    } else {
	pushBackPhase = 0;
	midHigh_smooth(values);
    }
}

void onlyBack(int *values) {
    setMainDepth(values, maxDepth - 1);

    int d;
    for (d = maxDepth - 2; d >= 0; --d) {
	for (band_it = 0; band_it < nrOfBands; ++band_it) {
	    leds[band_it][d] = 1;
	}
    }
}

void pushUp_smooth(int *values) {
    setMainDepth(values, maxDepth - 1);

    int d;
    for (d = maxDepth - 2; d >= 0; --d) {
	for (band_it = 0; band_it < nrOfBands; ++band_it) {
	    leds[band_it][d] = leds[band_it][d + 1];
	}
    }
}

void pushUp_three(int *values) {
    setMainDepth(values, maxDepth - 2);

    int d;
    for (d = maxDepth - 3; d >= 1; --d) {
	for (band_it = 0; band_it < nrOfBands; ++band_it) {
	    leds[band_it][d] = leds[band_it][d + 1];
	}
    }
    for (band_it = 0; band_it < nrOfBands; ++band_it) {
	leds[band_it][0] = -1;
	leds[band_it][maxDepth - 1] = -1;
    }
}

void pushUp_threeFront(int *values) {
    setMainDepth(values, 0);

    int d;
    for (d = 1; d < 3; ++d) {
	for (band_it = 0; band_it < nrOfBands; ++band_it) {
	    leds[band_it][d] = leds[band_it][d - 1];
	}
    }
    for (band_it = 0; band_it < nrOfBands; ++band_it) {
	leds[band_it][3] = -1;
	leds[band_it][4] = -1;
    }
}

void rollBack_smooth(int *values) {
    setMainDepth(values, 0);

    int w = 500;
    int d;
    for (d = maxDepth - 1; d > 0; d -= 1) {
	for (band_it = 0; band_it < nrOfBands; ++band_it) {
	    leds[band_it][d] = (leds[band_it][d - 1] * (1024 - w) + w * leds[band_it][d]) / 1024;
	    if (leds[band_it][d] < 1) {
		leds[band_it][d] = 1;
	    } else if (leds[band_it][d] > (16 << DECAYPRECISION) - 1) {
		leds[band_it][d] = (16 << DECAYPRECISION) - 1;
	    }
	}
    }
}

void rollBack_with_fall(int *values) {
    int d;
    u32 time = nrInterrupts % PERIOD;
    if (time < rollbacktime) {
	pushBack(time, values);
    } else {
	pushBackPhase = 0;
	rollBack_smooth(values);
    }
}

void sinewave(int *values) {
    setMainDepth(values, 0);

    for (band_it = 0; band_it < nrOfBands; ++band_it) {
	leds[band_it][1] = leds[band_it][0];
	leds[band_it][2] = -1;
	leds[band_it][3] = -1;
    }

    int d = maxDepth - 1;
    for (band_it = 0; band_it < nrOfBands; ++band_it) {
	leds[band_it][d] = 8 << DECAYPRECISION;
	leds[band_it][d] += ((2 * approxSin(nrInterrupts / 5 + band_it * 512)) << DECAYPRECISION) / SCALE;
	leds[band_it][d] += ((1 * approxSin(int2PI / 2 - nrInterrupts / 2 + band_it * 856)) << DECAYPRECISION) / SCALE;
	leds[band_it][d] += ((3 * approxSin(int2PI / 4 + nrInterrupts / 7 + band_it * 600) / 2) << DECAYPRECISION) / SCALE;
	leds[band_it][d] += ((2 * approxSin(int2PI / 3 - nrInterrupts / 5 + band_it * 700)) << DECAYPRECISION) / SCALE;

	if (leds[band_it][d] < 1) {
	    leds[band_it][d] = 1;
	} else if (leds[band_it][d] > (16 << DECAYPRECISION) - 1) {
	    leds[band_it][d] = (16 << DECAYPRECISION) - 1;
	}
    }
}

//-------------------------------

fixedpoint hamming(int m, int k) {
    return ALPHA - (BETA * approxCos(int2PI * m / NFreq[k]) >> PRECISION);
}

void cqt() {
    unsigned int k, i, indx;
    int windowed, angle;
    float real_f, imag_f;
    fixedpoint real, imag;
    for (k = 0; k < lowfreq_endIndex; ++k) {
	indx = nrInterrupts % NRSAMPLES - 1 + 8 * NRSAMPLES;
	real = ALPHA - (BETA * signal_lowfreq[indx % NRSAMPLES] >> PRECISION);
	imag = 0;
	for (i = 1; i < NFreq[k]; ++i) {
	    windowed = hamming(i, k) * signal_lowfreq[(indx - i * Div[k]) % NRSAMPLES];
	    angle = twoPiQ * i / NFreq[k];
	    real += windowed * approxCos(angle) >> PRECISION;
	    imag += windowed * approxSin(angle) >> PRECISION;
	}

	real_f = real / (float) SCALE;
	imag_f = imag / (float) SCALE;
	freqs[k] = logf(powf(real_f * real_f + imag_f * imag_f, 0.5) / NFreq[k] + 0.1) * amplitude / 32;
    }
    for (; k < FREQS; ++k) {
	indx = nrInterrupts % NRSAMPLES - 1 + 8 * NRSAMPLES;
	real = ALPHA - (BETA * signal[indx % NRSAMPLES] >> PRECISION);
	imag = 0;
	for (i = 1; i < NFreq[k]; ++i) {
	    windowed = hamming(i, k) * signal[(indx - i * Div[k]) % NRSAMPLES];
	    angle = twoPiQ * i / NFreq[k];
	    real += windowed * approxCos(angle) >> PRECISION;
	    imag += windowed * approxSin(angle) >> PRECISION;
	}
	real_f = real / (float) SCALE;
	imag_f = imag / (float) SCALE;
	freqs[k] = logf(powf(real_f * real_f + imag_f * imag_f, 0.5) / NFreq[k] + 0.1) * amplitude / 32;
    }
}

void preprocess_filters() {
    //CDC.printf("calculating twoPiQ...\n");
    float nn = powf(2, log(F16 / (double) F0) / log(2) / 16.0);
    twoPiQ = int2PI * (nn + 1) / (nn - 1) / 2;

    int i;
    //printf("calculating the border frequencies...\n");
    for (i = 0; i < FREQS + 1; ++i) {
	Freq[i] = (F0 * powf(nn, i) + F16 / powf(nn, FREQS - i)) / 2;
    }

    //CDC.printf("calculating the index until which the signal_lowfreq samples should be used\n");
    lowfreq_endIndex = 0;
    while (FSAMPLE / (Freq[lowfreq_endIndex + 1] - Freq[lowfreq_endIndex]) >= NRSAMPLES && lowfreq_endIndex < FREQS) {
	++lowfreq_endIndex;
    }

    int samplesLeft = MAXTOTALSAMPLES;

    //CDC.printf("calculating sample frequency dividers...\n");
    for (i = 16; i > lowfreq_endIndex; --i) {
	Div[i - 1] = 1 + FSAMPLE / ((Freq[i] - Freq[i - 1]) * samplesLeft / i);
	NFreq[i - 1] = FSAMPLE / (Freq[i] - Freq[i - 1]) / Div[i - 1];
	samplesLeft -= NFreq[i - 1];
    }
    for (; i > 0; --i) {
	Div[i - 1] = 1 + FSAMPLE / LOWFREQDIV / ((Freq[i] - Freq[i - 1]) * samplesLeft / i);
	NFreq[i - 1] = FSAMPLE / LOWFREQDIV / (Freq[i] - Freq[i - 1]) / Div[i - 1];
	samplesLeft -= NFreq[i - 1];
    }
}

//-------------------------------
void chooseEffect() {
    u32 time = nrInterrupts - nrInterruptsSinceEffectChange;
    if (time < rollbacktime) {
	pushBack(time, freqs);
    } else {
	effectFuncs[effect] (freqs);
    }
}

void adjustLayers() {
    layercounter = (layercounter + 1) % (nrOfLayers * MAXCYCLE);
    int phase = layercounter % MAXCYCLE;
    if (phase == 0) {
	// deactivating layer
	digitalWrite(MuxEnable, 1);
	if (layercounter == 0) {
	    calculationFlag = 1;
	}
    }
    unsigned int l = layercounter / MAXCYCLE;	// this is the current layer
    if ((MAXCYCLE - phase) - layerIntensity[l] == 0) {
	// pushing data into shift registers
	int i;
	for (i = 0; i < maxDepth; ++i) {
	    SPI.transfer(spiPointer[l * 5 + i]);
	}
	// pushing data to output of shift registers 
	digitalWrite(RegEnable, 1);
	digitalWrite(RegEnable, 0);

	// pushing the layer to be activated
	digitalWrite(MUX0, l & 1);
	digitalWrite(MUX1, l & 2);
	digitalWrite(MUX2, l & 4);
	digitalWrite(MUX3, l & 8);	// 00...001000 

	// activating layer
	digitalWrite(MuxEnable, 0);
    }
}

#define SATURATION_LEVEL 470
void Tmr4Interrupt(void) {
    if (IFS0bits.T4IF) {	// Timer Interrupt flag
	IFS0CLR = 0x10000;	// Clear the timer interrupt flag

	++nrInterrupts;
	latestSample = analogRead(ANALOG_PIN) - 511;
	signal[nrInterrupts % NRSAMPLES] = latestSample;
	signal_lowfreq[(nrInterrupts / LOWFREQDIV) % NRSAMPLES] = latestSample;

	adjustLayers();
    }
}

void setAllFilled() {
    depth_on[0] = 1;
    depth_on[1] = 1;
    depth_on[2] = 1;
    depth_on[3] = 1;
    depth_on[4] = 1;
}

void setAllUnfilled() {
    depth_on[0] = 0;
    depth_on[1] = 0;
    depth_on[2] = 0;
    depth_on[3] = 0;
    depth_on[4] = 0;
}

void setFrontAndBackFilled() {
    depth_on[0] = 1;
    depth_on[1] = 0;
    depth_on[2] = 0;
    depth_on[3] = 0;
    depth_on[4] = 1;
}

void setMidFilled() {
    depth_on[0] = 0;
    depth_on[1] = 0;
    depth_on[2] = 1;
    depth_on[3] = 0;
    depth_on[4] = 0;
}

void setThreeMidFilled() {
    depth_on[0] = 0;
    depth_on[1] = 1;
    depth_on[2] = 1;
    depth_on[3] = 1;
    depth_on[4] = 0;
}

void setThreeFrontFilled() {
    depth_on[0] = 1;
    depth_on[1] = 1;
    depth_on[2] = 1;
    depth_on[3] = 0;
    depth_on[4] = 0;
}

void setTwoMidFilled() {
    depth_on[0] = 0;
    depth_on[1] = 1;
    depth_on[2] = 0;
    depth_on[3] = 1;
    depth_on[4] = 0;
}

void setTwoFrontFilled() {
    depth_on[0] = 1;
    depth_on[1] = 0;
    depth_on[2] = 1;
    depth_on[3] = 0;
    depth_on[4] = 0;
}

void setFilled11001() {
    depth_on[0] = 1;
    depth_on[1] = 1;
    depth_on[2] = 0;
    depth_on[3] = 0;
    depth_on[4] = 1;
}

#define EFFECT_PERIOD 70000

void adjustInputs() {
    effect = readPin(EFFECT_PIN) * NREFFECTS / 1024;
    if (effect == 0) {
	effect = effectMap[((nrInterrupts / EFFECT_PERIOD) % (NREFFECTS - 4))];	// Increase effect looping speed by increasing the factor
    } else if (effect == NREFFECTS - 1) {
	effect = effectMap[((nrInterrupts / EFFECT_PERIOD) % (NREFFECTS - 4))];	// Increase effect looping speed by increasing the factor
	if (old_effect_read != effect) {
	    nrInterruptsSinceEffectChange = nrInterrupts;
	    pushBackPhase = 0;
	}
    }

    old_effect_read = effect;
    fillFuncs[effect] ();

    //CDC.printf("intensity %i\n",readPin(INTENSITY_PIN));
    //CDC.printf("decay %i\n",readPin(DECAY_PIN));

    amplitude = readPin(AMPLITUDE_PIN);
    decay = 147 * logf(readPin(DECAY_PIN) + 1);	// can be 0 or [101..1018]

    u32 tmp = readPin(INTENSITY_PIN) * (MAXCYCLE + 2) / 1024;
    int data_in = 1;
    int i;
    u32 idx = nrInterrupts;
    for (i = 1; i <= 10; ++i) {
	if (fp_abs(signal[(idx - i) % NRSAMPLES]) < 2) {
	    data_in = 0;
	    break;
	}
    }
    if (data_in == 0 && previousIntensity == tmp) {
	previousIntensity = tmp;
	if (nrInterrupts - nrInterruptsSinceSignal > 857000) {	// 857000 is about 60 seconds
	    intensity = 0;
	} else {
	    intensity = tmp;
	}
    } else {
	previousIntensity = tmp;
	intensity = tmp;
	nrInterruptsSinceSignal = nrInterrupts;
    }
    if (intensity <= MAXCYCLE) {
	for (i = 0; i < nrOfLayers; ++i) {
	    layerIntensity[i] = intensity;
	}
    } else {
	for (i = 0; i < 8; ++i) {
	    layerIntensity[i] = i / 2 + 1;
	}
	for (i = 8; i < 15; ++i) {
	    layerIntensity[i] = i - 3;
	}
	layerIntensity[15] = 11;
    }

    unsigned int minF = readPin(LOWFREQ_PIN);
    unsigned int maxF = readPin(HIGHFREQ_PIN);	// TODO: make more generic

    if (fp_abs(minF - oldMinF) > 1 || fp_abs(maxF - oldMaxF) > 1) {
	oldMinF = minF;
	oldMaxF = maxF;

	float base = 1.0091;
	minF = powf(base, minF);
	maxF = powf(base, maxF);
	if (minF < lowFreqBound) {
	    minF = lowFreqBound;
	}
	if (maxF > highFreqBound) {
	    maxF = highFreqBound;
	}
	if (2 * minF > maxF) {	// at least one octave will always be displayed.
	    if (2 * minF > highFreqBound) {
		maxF = highFreqBound;
		minF = highFreqBound / 2;
	    } else {
		maxF = 2 * minF;
	    }
	}
	if (maxF - minF < 60) {	// low frequencies must differ at least 60, so that the frequency width of each bucket is at least 2Hz. This prevents the number of samples needed to be larger than MAXSAMPLESIZE
	    maxF = minF + 60;
	}
	F0 = minF;
	F16 = maxF;
	preprocess_filters();
    }
}

void calculateSPIData() {
    unsigned int l;		// layer
    int i, j, nthLed, nthLed_mod, nthLed_div;
    u16 *tmpPointer;		// use of pointers to arrays to avoid interrupt reading half-written arrays
    if (spiPointer == spiData) {
	tmpPointer = spiData2;
    } else {
	tmpPointer = spiData;
    }
    u16 tmp;
    for (i = 0; i < maxDepth; ++i) {
	for (l = 0; l < nrOfLayers; ++l) {
	    tmp = 0;
	    for (j = 0; j < nrOfBands; ++j) {
		nthLed = i * nrOfBands + j;
		tmp <<= 1;
		nthLed_mod = nthLed % maxDepth;
		nthLed_div = nthLed / maxDepth;
		if (depth_on[nthLed_mod] == 0) {
		    tmp |= (leds[nthLed_div][nthLed_mod] >> DECAYPRECISION) != l;
		} else {
		    tmp |= (leds[nthLed_div][nthLed_mod] >> DECAYPRECISION) < l;
		}
	    }
	    tmpPointer[l * 5 + i] = tmp;
	}
    }
    spiPointer = tmpPointer;
}

void loop() {

    if (fp_abs(latestSample) >= SATURATION_LEVEL) {	// checking for saturation (old: 475)
	digitalWrite(SATURATION_PIN, 1);
    } else {
	digitalWrite(SATURATION_PIN, 0);
    }
    adjustInputs();

    if (calculationFlag != 0) {

	calculationFlag = 0;
	cqt();
	chooseEffect();
	calculateSPIData();
    }
}

void setup() {
    int i, j, k;

    // ODCG=0x00000100; zet pin 11 op open drain
    // ODCG 0x000000ff manipuleert pin 13
    ODCG = 0x00000100;
    SPI.init();
    // Giving a higher number (eg 0xffffffff) will speed up the SPI to 20MHz
    // However, the shift registers don't seem to be able to cope with this :(
    // 0x00400000 is already to fast, so we keep it on 0x00380000, which is about 4MHz
    SPI_clock(0x00390000);
    SPI2CON = SPI2CON | (1 << 10);	// Setting the SPI to use 16-bit words instead of 8.
    // use SPI2CON=SPI2CON | (1<<11); to get to 32 bit
    // Adjusting this also requires to adjust the SPI_write 
    // signature in spi.c to "unsigned char SPI_write(unsigned int data_out)".

    AD1CON3 = 0x8100;		// speeding up samplin rate from 12us to 8us

    initializeOutput(RegEnable, 0);
    initializeOutput(MUX0, 0);
    initializeOutput(MUX1, 0);
    initializeOutput(MUX2, 0);
    initializeOutput(MUX3, 0);
    initializeOutput(MuxEnable, 1);
    initializeOutput(FLIPMONKEY, 0);
    initializeOutput(SATURATION_PIN, 0);
    initializeOutput(BUZZ_PIN, 0);

    u32 layer, band, depth;
    for (band = 0; band < nrOfBands; ++band) {
	for (depth = 0; depth < maxDepth; ++depth) {
	    leds[band][depth] = 0;
	}
    }
    for (depth = 0; depth < maxDepth; ++depth) {
	depth_on[depth] = 0;
    }

    layercounter = 0;

    oldMinF = 100;
    oldMaxF = 1000;

    effect = 0;
    nrInterrupts = 0;
    nrInterruptsSinceEffectChange = 0;
    nrInterruptsSinceSignal = 0;

    for (k = 0; k < FREQS; ++k) {
	Div[k] = 1;		// TODO: initialize everything decently to avoid division by 0 errors and other problems
    }

    effectMap[0] = 6;
    effectMap[1] = 19;
    effectMap[2] = 2;
    effectMap[3] = 13;
    effectMap[4] = 8;
    effectMap[5] = 10;
    effectMap[6] = 15;
    effectMap[7] = 16;
    effectMap[8] = 11;
    effectMap[9] = 4;
    effectMap[10] = 21;
    effectMap[11] = 14;
    effectMap[12] = 3;
    effectMap[13] = 17;
    effectMap[14] = 7;
    effectMap[15] = 5;
    effectMap[16] = 1;
    effectMap[17] = 12;
    effectMap[18] = 20;
    effectMap[19] = 18;
    effectMap[20] = 9;
    effectMap[21] = 22;
    effectMap[22] = 25;
    effectMap[23] = 0;
    effectMap[24] = 0;
    effectMap[25] = 0;
    effectMap[26] = 0;


    effectFuncs[0] = 0;
    fillFuncs[0] = 0;
    effectFuncs[1] = &pushUp_smooth;
    fillFuncs[1] = &setAllFilled;
    effectFuncs[2] = &pushUp_smooth;
    fillFuncs[2] = &setFrontAndBackFilled;
    effectFuncs[3] = &pushUp_smooth;
    fillFuncs[3] = &setAllUnfilled;
    effectFuncs[4] = &pushUp_three;
    fillFuncs[4] = &setThreeMidFilled;
    effectFuncs[5] = &pushUp_three;
    fillFuncs[5] = &setTwoMidFilled;
    effectFuncs[6] = &pushUp_three;
    fillFuncs[6] = &setAllUnfilled;
    effectFuncs[7] = &pushUp_threeFront;
    fillFuncs[7] = &setThreeFrontFilled;
    effectFuncs[8] = &pushUp_threeFront;
    fillFuncs[8] = &setTwoFrontFilled;
    effectFuncs[9] = &pushUp_threeFront;
    fillFuncs[9] = &setAllUnfilled;
    effectFuncs[10] = &rollBack_smooth;
    fillFuncs[10] = &setAllFilled;
    effectFuncs[11] = &rollBack_smooth;
    fillFuncs[11] = &setFrontAndBackFilled;
    effectFuncs[12] = &rollBack_smooth;
    fillFuncs[12] = &setAllUnfilled;
    effectFuncs[13] = &backHigh_smooth;
    fillFuncs[13] = &setAllFilled;
    effectFuncs[14] = &backHigh_smooth;
    fillFuncs[14] = &setFrontAndBackFilled;
    effectFuncs[15] = &backHigh_smooth;
    fillFuncs[15] = &setAllUnfilled;
    effectFuncs[16] = &midHigh_smooth;
    fillFuncs[16] = &setAllFilled;
    effectFuncs[17] = &midHigh_smooth;
    fillFuncs[17] = &setFrontAndBackFilled;
    effectFuncs[18] = &midHigh_smooth;
    fillFuncs[18] = &setAllUnfilled;
    effectFuncs[19] = &midHigh_smooth;
    fillFuncs[19] = &setMidFilled;
    effectFuncs[20] = &onlyBack;
    fillFuncs[20] = &setFrontAndBackFilled;
    effectFuncs[21] = &onlyBack;
    fillFuncs[21] = &setAllUnfilled;
    effectFuncs[22] = &pushUp_smooth;
    fillFuncs[22] = &setMidFilled;
    effectFuncs[23] = &rollBack_with_fall;
    fillFuncs[23] = &setAllUnfilled;
    effectFuncs[24] = &midHigh_with_fall;
    fillFuncs[24] = &setAllUnfilled;
    effectFuncs[25] = &sinewave;
    fillFuncs[25] = &setFilled11001;
    effectFuncs[26] = 0;
    fillFuncs[26] = 0;

    init_timer4();
}

Wavelet:    11025,10 scales=     5--10--21--43--86--172--344--689--1378--2756--5512
CQT:        11025,16 bands=      20--28--40--57--81--115--164--233--471--670--952--1352--1922--2730--3879--5512