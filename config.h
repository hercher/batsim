#ifndef CONFIG_H_
#define CONFIG_H_

#define BATS            4        //number of bats (1..8)
#define STEP_DURATION   20       //ms for each step in the markov chain
#define BAT_TIME        10       //sec active time after motion detection
#define PWM_PORT        PORTA    //PORT used to hook the bats up to
#define PWM_DDR         DDRA     //data direction register of the port

#endif /* CONFIG_H_ */
