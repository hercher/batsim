#include <stdlib.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include "config.h"

/* do not change */
#define F_PWM 30			//pwm frequency
#define PWM_STEPS 256		//pwm steps

#define T_PWM (F_CPU/(F_PWM*PWM_STEPS))

#if (T_PWM<(93+5))
    #error "T_PWM too low"
#endif

struct successor {
	uint8_t prob;
	struct state *next_state;
};

struct state {
	char *name;
	uint8_t pwm;
	uint8_t count;
	struct successor *successors;
};

/*
 * this is a (circular) markov chain
 * modeling the pwm setting per state
 * and the probability for state transitions
 *
 * each bat moves independently through this chain
 */
static struct state batstate[] = {
	/*OFF*/
	{"OFF", 	0,   	2, 	(struct successor[]){{90, &batstate[0]}, {10, &batstate[1]}}},
	/* UP */
	{"UP0", 	1,   	2, 	(struct successor[]){{90, &batstate[2]}, {10, &batstate[15]}}},
	{"UP1", 	4,   	2, 	(struct successor[]){{90, &batstate[3]}, {10, &batstate[14]}}},
	{"UP2", 	8,   	2, 	(struct successor[]){{90, &batstate[4]}, {10, &batstate[13]}}},
	{"UP3", 	16,  	2, 	(struct successor[]){{90, &batstate[5]}, {10, &batstate[12]}}},
	{"UP4", 	32,  	2, 	(struct successor[]){{90, &batstate[6]}, {10, &batstate[11]}}},
	{"UP5", 	64,  	2, 	(struct successor[]){{90, &batstate[7]}, {10, &batstate[10]}}},
	{"UP6", 	128, 	2, 	(struct successor[]){{90, &batstate[8]}, {10, &batstate[9] }}},
	/* FULL ON */
	{"ON", 		255, 	2,	(struct successor[]){{50, &batstate[8]}, {50, &batstate[9]}}},
	/* DOWN */
	{"DOWN0", 	128,   	2, 	(struct successor[]){{90, &batstate[10]}, {10, &batstate[7]}}},
	{"DOWN1", 	64,   	2, 	(struct successor[]){{90, &batstate[11]}, {10, &batstate[6]}}},
	{"DOWN2", 	32,   	2, 	(struct successor[]){{90, &batstate[12]}, {10, &batstate[5]}}},
	{"DOWN3", 	16,  	2, 	(struct successor[]){{90, &batstate[13]}, {10, &batstate[4]}}},
	{"DOWN4", 	8,  	2, 	(struct successor[]){{90, &batstate[14]}, {10, &batstate[3]}}},
	{"DOWN5", 	4,  	2, 	(struct successor[]){{90, &batstate[15]}, {10, &batstate[2]}}},
	{"DOWN6", 	1, 		2, 	(struct successor[]){{90, &batstate[0] }, {10, &batstate[1]}}},
};

static struct state *cur_state[BATS];

static void init_current_state(void) {
	uint8_t i;

	for(i = 0; i < BATS; i++) {
		cur_state[i] = &batstate[0];
	}
}

static  struct state *get_next_state(struct state *s) {
	uint8_t p;
	uint8_t sum = 0;
	uint8_t i;

	p = rand() % 100;
	for(i = 0; i < s->count; i++) {
		sum += s->successors[i].prob;
		if (sum >= p)
			return s->successors[i].next_state;
	}
	return NULL;
}

/*
 * this soft puls width modulation is based on
 * http://www.mikrocontroller.net/articles/Soft-PWM
 *
 * we have 256 steps resolution but use only
 * 8 distinct values to better match the
 * light sensitivity of the human eye
 */
static volatile uint8_t pwm_setting[8];

static void reset_pwm_settings(void) {
	uint8_t i;

	for(i = 0; i < 8; i++) {
		pwm_setting[i] = 0;
	}
}

ISR(TIMER1_COMPA_vect) {
    static uint8_t pwm_cnt=0;
    uint8_t tmp=0;

    OCR1A += (uint16_t)T_PWM;

    if(pwm_setting[0] > pwm_cnt)
    	tmp |= (1<<PIN0);
    if(pwm_setting[1] > pwm_cnt)
    	tmp |= (1<<PIN1);
    if(pwm_setting[2] > pwm_cnt)
    	tmp |= (1<<PIN2);
    if(pwm_setting[3] > pwm_cnt)
    	tmp |= (1<<PIN3);
    if(pwm_setting[4] > pwm_cnt)
    	tmp |= (1<<PIN4);
    if(pwm_setting[5] > pwm_cnt)
    	tmp |= (1<<PIN5);
    if(pwm_setting[6] > pwm_cnt)
    	tmp |= (1<<PIN6);
    if(pwm_setting[7] > pwm_cnt)
    	tmp |= (1<<PIN7);
    PWM_PORT = tmp;
    if(pwm_cnt==(uint8_t)(PWM_STEPS-1))
        pwm_cnt=0;
    else
        pwm_cnt++;
}

static inline void setup_timer1(void) {

	/* timer at full speed */
    TCCR1B = 1;
	OCR1A = 1;
}

static inline void enable_timer1(void) {

	/* clear pending int */
	TIFR |= (1<<OCIE1A);
	TIMSK |= (1<<OCIE1A);
}

static inline void disable_timer1(void) {

	TIMSK &= ~(1<<OCIE1A);
	/* clear pending int */
	TIFR |= (1<<OCIE1A);
}

/*
 * only used for wakeup
 */
ISR(INT0_vect) {

}

static inline void setup_pwm(void) {
	uint8_t i;

	PWM_DDR = 0;
	for(i = 0; i < BATS; i++) {
		PWM_DDR |= (1 << i);
	}
}

/*
 * enable external interrupt0
 * use low level triggering which
 * is the default setting
 * so no setup is required
 */
static inline void enable_ext_int(void) {

	/* clear pending int */
	GIFR = (1<<INT0);
	GICR |= (1<<INT0);
}

static inline void disable_ext_int(void) {

	GICR &= ~(1<<INT0);
	/* clear pending int */
	GIFR = (1<<INT0);
}

int main(void) {
	struct state *next;
	uint16_t steps;
	uint8_t i;

	setup_pwm();
	setup_timer1();
	init_current_state();
	reset_pwm_settings();
	PWM_PORT = 0;
	sei();

    while(1) {
    	enable_ext_int();
    	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    	sleep_mode();
        /* awake */
    	disable_ext_int();
        enable_timer1();
        for(steps = 0; steps < (BAT_TIME * 1000) / STEP_DURATION; steps++) {
            for(i = 0; i < BATS; i++) {
            	next = get_next_state(cur_state[i]);
            	if(next) {
            		cur_state[i] = next;
            	}
            	pwm_setting[i] = cur_state[i]->pwm;
            }
        	_delay_ms(STEP_DURATION);
        }
        disable_timer1();
        reset_pwm_settings();
        PWM_PORT = 0;
    }
    return 0;
}
