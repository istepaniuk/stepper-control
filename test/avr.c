#define CW  0
#define CCW 1
#define HALFSTEPS

static int OCR1A = 0;

static int TCCR1B;

static int CS10;

static int CS11;

static int CS12;

//#define FULLSTEPS

#define SM_PORT         PORTC
#define SM_DRIVE        DDRC
#define A1    PC7 //!< Stepper motor winding A positive pole.
#define A2    PC6 //!< Stepper motor winding A negative pole.
#define B1    PC5 //!< Stepper motor winding B positive pole.
#define B2    PC4 //!< Stepper motor winding B negative pole.

void sm_driver_Init_IO(void);
unsigned char sm_driver_StepCounter(signed char inc);
void sm_driver_StepOutput(unsigned char pos);

//! Position of stepper motor.
extern int stepPosition;

#ifndef SPEED_CNTR_H
#define SPEED_CNTR_H

/*! \brief Holding data used by timer interrupt for speed ramp calculation.
 *
 *  Contains data used by timer interrupt to calculate speed profile.
 *  Data is written to it by move(), when stepper motor is moving (timer
 *  interrupt running) data is read/updated when calculating a new step_delay
 */
typedef struct {
    //! What part of the speed ramp we are in.
    unsigned char run_state
            : 3;
    //! Direction stepper motor should move.
    unsigned char dir
            : 1;
    //! Peroid of next timer delay. At start this value set the accelration rate.
    unsigned int step_delay;
    //! What step_pos to start decelaration
    unsigned int decel_start;
    //! Sets deceleration rate.
    signed int decel_val;
    //! Minimum time delay (max speed)
    signed int min_delay;
    //! Counter used when accelerateing/decelerateing to calculate step_delay.
    signed int accel_count;
} speedRampData;

/*! \Brief Frequency of timer1 in [Hz].
 *
 * Modify this according to frequency used. Because of the prescaler setting,
 * the timer1 frequency is the clock frequency divided by 8.
 */
// Timer/Counter 1 running on 3,686MHz / 8 = 460,75kHz (2,17uS). (T1-FREQ 460750)
#define T1_FREQ 1382400

//! Number of (full)steps per round on stepper motor in use.
#define FSPR 200

#ifdef HALFSTEPS
#define SPR (FSPR*2)
#pragma message("[speed_cntr.c] *** Using Halfsteps ***")
#endif
#ifdef FULLSTEPS
#define SPR FSPR
#pragma message("[speed_cntr.c] *** Using Fullsteps ***")
#endif
#ifndef HALFSTEPS
#ifndef FULLSTEPS

#endif
#endif

// Maths constants. To simplify maths when calculating in speed_cntr_Move().
#define ALPHA (2*3.14159/SPR)                    // 2*pi/spr
#define A_T_x100 ((long)(ALPHA*T1_FREQ*100))     // (ALPHA / T1_FREQ)*100
#define T1_FREQ_148 ((int)((T1_FREQ*0.676)/100)) // divided by 100 and scaled by 0.676
#define A_SQ (long)(ALPHA*2*10000000000)         // ALPHA*2*10000000000
#define A_x20000 (int)(ALPHA*20000)              // ALPHA*20000

// Speed ramp states
#define STOP  0
#define ACCEL 1
#define DECEL 2
#define RUN   3

void speed_cntr_Move(signed int step, unsigned int accel, unsigned int decel, unsigned int speed);
void speed_cntr_Init_Timer1(void);
static unsigned long sqrt(unsigned long v);
unsigned int min(unsigned int x, unsigned int y);

//! Global status flags
extern struct GLOBAL_FLAGS status;

#endif

#define F_CPU 11059200

#define TRUE 1
#define FALSE 0

/*! \brief Status flags
 */
struct GLOBAL_FLAGS {
    //! True when stepper motor is running.
    unsigned char running
            :1;
    //! True when uart has received a string (ended with '/r').
    unsigned char cmd
            :1;
    //! Dummy bits to fill up a byte.
    unsigned char dummy
            :6;
};

//! Cointains data for timer interrupt.
speedRampData srd;

/*! \brief Move the stepper motor a given number of steps.
 *
 *  Makes the stepper motor move the given number of steps.
 *  It accelrate with given accelration up to maximum speed and decelerate
 *  with given deceleration so it stops at the given step.
 *  If accel/decel is to small and steps to move is to few, speed might not
 *  reach the max speed limit before deceleration starts.
 *
 *  \param step  Number of steps to move (pos - CW, neg - CCW).
 *  \param accel  Accelration to use, in 0.01*rad/sec^2.
 *  \param decel  Decelration to use, in 0.01*rad/sec^2.
 *  \param speed  Max speed, in 0.01*rad/sec.
 */
void speed_cntr_Move(signed int step, unsigned int accel, unsigned int decel, unsigned int speed)
{
    //! Number of steps before we hit max speed.
    unsigned int max_s_lim;
    //! Number of steps before we must start deceleration (if accel does not hit max speed).
    unsigned int accel_lim;

    // Set direction from sign on step value.
    if (step < 0) {
        srd.dir = CCW;
        step = -step;
    }
    else {
        srd.dir = CW;
    }

    // If moving only 1 step.
    if (step == 1) {
        // Move one step...
        srd.accel_count = -1;
        // ...in DECEL state.
        srd.run_state = DECEL;
        // Just a short delay so main() can act on 'running'.
        srd.step_delay = 1000;
        status.running = TRUE;
        OCR1A = 10;
        // Run Timer/Counter 1 with prescaler = 8.
        TCCR1B |= ((0 << CS12) | (1 << CS11) | (0 << CS10));
    }
        // Only move if number of steps to move is not zero.
    else if (step != 0) {
        // Refer to documentation for detailed information about these calculations.

        // Set max speed limit, by calc min_delay to use in timer.
        // min_delay = (alpha / tt)/ w
        srd.min_delay = A_T_x100 / speed;

        // Set accelration by calc the first (c0) step delay .
        // step_delay = 1/tt * sqrt(2*alpha/accel)
        // step_delay = ( tfreq*0.676/100 )*100 * sqrt( (2*alpha*10000000000) / (accel*100) )/10000
        srd.step_delay = (T1_FREQ_148 * sqrt(A_SQ / accel)) / 100;

        // Find out after how many steps does the speed hit the max speed limit.
        // max_s_lim = speed^2 / (2*alpha*accel)
        max_s_lim = (long) speed * speed / (long) (((long) A_x20000 * accel) / 100);
        // If we hit max speed limit before 0,5 step it will round to 0.
        // But in practice we need to move atleast 1 step to get any speed at all.
        if (max_s_lim == 0) {
            max_s_lim = 1;
        }

        // Find out after how many steps we must start deceleration.
        // n1 = (n1+n2)decel / (accel + decel)
        accel_lim = ((long) step * decel) / (accel + decel);
        // We must accelrate at least 1 step before we can start deceleration.
        if (accel_lim == 0) {
            accel_lim = 1;
        }

        // Use the limit we hit first to calc decel.
        if (accel_lim <= max_s_lim) {
            srd.decel_val = accel_lim - step;
        }
        else {
            srd.decel_val = -((long) max_s_lim * accel) / decel;
        }
        // We must decelrate at least 1 step to stop.
        if (srd.decel_val == 0) {
            srd.decel_val = -1;
        }

        // Find step to start decleration.
        srd.decel_start = step + srd.decel_val;

        // If the maximum speed is so low that we dont need to go via accelration state.
        if (srd.step_delay <= srd.min_delay) {
            srd.step_delay = srd.min_delay;
            srd.run_state = RUN;
        }
        else {
            srd.run_state = ACCEL;
        }

        // Reset counter.
        srd.accel_count = 0;
        status.running = TRUE;
        //OCR1A = 10;
        // Set Timer/Counter to divide clock by 8
        //TCCR1B |= ((0 << CS12) | (1 << CS11) | (0 << CS10));
    }
}

/*! \brief Init of Timer/Counter1.
 *
 *  Set up Timer/Counter1 to use mode 1 CTC and
 *  enable Output Compare A Match Interrupt.
 */
void speed_cntr_Init_Timer1(void)
{
    // Tells what part of speed ramp we are in.
    srd.run_state = STOP;
    // Timer/Counter 1 in mode 4 CTC (Not running).
    //TCCR1B = (1 << WGM12);
    // Timer/Counter 1 Output Compare A Match Interrupt enable.
    //TIMSK = (1 << OCIE1A);
}

/*! \brief Timer/Counter1 Output Compare A Match Interrupt.
 *
 *  Timer/Counter1 Output Compare A Match Interrupt.
 *  Increments/decrements the position of the stepper motor
 *  exept after last position, when it stops.
 *  The \ref step_delay defines the period of this interrupt
 *  and controls the speed of the stepper motor.
 *  A new step delay is calculated to follow wanted speed profile
 *  on basis of accel/decel parameters.
 */
//#pragma vector=TIMER1_COMPA_vect
//__interrupt

void speed_cntr_TIMER1_COMPA_interrupt(void)
{
    // Holds next delay period.
    unsigned int new_step_delay;
    // Remember the last step delay used when accelrating.
    static int last_accel_delay;
    // Counting steps when moving.
    static unsigned int step_count = 0;
    // Keep track of remainder from new_step-delay calculation to incrase accurancy
    static unsigned int rest = 0;

    OCR1A = srd.step_delay;

    switch (srd.run_state) {
        case STOP:
            step_count = 0;
            rest = 0;
            // Stop Timer/Counter 1.
            TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));
            status.running = FALSE;
            break;

        case ACCEL:
            sm_driver_StepCounter(srd.dir);
            step_count++;
            srd.accel_count++;
            new_step_delay = srd.step_delay - (((2 * (long) srd.step_delay) + rest) / (4 * srd.accel_count + 1));
            rest = ((2 * (long) srd.step_delay) + rest) % (4 * srd.accel_count + 1);
            // Chech if we should start decelration.
            if (step_count >= srd.decel_start) {
                srd.accel_count = srd.decel_val;
                srd.run_state = DECEL;
            }
                // Chech if we hitted max speed.
            else if (new_step_delay <= srd.min_delay) {
                last_accel_delay = new_step_delay;
                new_step_delay = srd.min_delay;
                rest = 0;
                srd.run_state = RUN;
            }
            break;

        case RUN:
            sm_driver_StepCounter(srd.dir);
            step_count++;
            new_step_delay = srd.min_delay;
            // Chech if we should start decelration.
            if (step_count >= srd.decel_start) {
                srd.accel_count = srd.decel_val;
                // Start decelration with same delay as accel ended with.
                new_step_delay = last_accel_delay;
                srd.run_state = DECEL;
            }
            break;

        case DECEL:
            sm_driver_StepCounter(srd.dir);
            step_count++;
            srd.accel_count++;
            new_step_delay = srd.step_delay - (((2 * (long) srd.step_delay) + rest) / (4 * srd.accel_count + 1));
            rest = ((2 * (long) srd.step_delay) + rest) % (4 * srd.accel_count + 1);
            // Check if we at last step
            if (srd.accel_count >= 0) {
                srd.run_state = STOP;
            }
            break;
    }
    srd.step_delay = new_step_delay;
}


static unsigned long sqrt(unsigned long x)
{
    register unsigned long xr;  // result register
    register unsigned long q2;  // scan-bit register
    register unsigned char f;   // flag (one bit)

    xr = 0;                     // clear result
    q2 = 0x40000000L;           // higest possible result bit
    do {
        if ((xr + q2) <= x) {
            x -= xr + q2;
            f = 1;                  // set flag
        }
        else {
            f = 0;                  // clear flag
        }
        xr >>= 1;
        if (f) {
            xr += q2;               // test flag
        }
    }
    while (q2 >>= 2);          // shift twice
    if (xr < x) {
        return xr + 1;             // add for rounding
    }
    else {
        return xr;
    }
}


unsigned int min(unsigned int x, unsigned int y)
{
    if (x < y) {
        return x;
    }
    else {
        return y;
    }
}
