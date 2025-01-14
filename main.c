#include "MKL25Z4.h"                    // Device header
#include "system_MKL25Z4.h"             // Keil::Device:Startup
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"

#define BAUD_RATE 9600
#define UART_RX_PORTE23 23
#define UART2_INT_PRIO 4
#define BLUE_LED 1
#define MASK(x) (1<<x)

#define Q_SIZE (32)
#define DIR_MASK 0b11000000


#define TPM_inst TPM0
#define TPM_CH_FW_LEFT 1
#define TPM_CH_BW_LEFT 2
#define TPM_CH_FW_RIGHT 4
#define TPM_CH_BW_RIGHT 5	
#define PTB0_Pin 0
#define PTB1_Pin 1
#define MOD_VAL 25500


osEventFlagsId_t brainFlag;
osEventFlagsId_t motorFlag;


//Os Thread Id for tasks
osThreadId_t tBrain;
osThreadId_t tMotorControl;
osThreadId_t tLED;
osThreadId_t tAudio;

//Os mutex Id
osMutexId_t mutex_instance;

typedef enum {
	STOP,
	RUN,
	FINISH
} t_state_robot;


volatile t_state_robot robot_state = STOP;

typedef struct{
	uint8_t motor; //left as 0 or right as 1
	uint8_t direction;//positive as 1 or negative as 0
	uint16_t speed;// max value equal to MOD_VAL
}motor_ins_t;

volatile osMessageQueueId_t motorQueue;
osMessageQueueId_t brainQueue;



	


void initUART2(uint32_t baud_rate)
{
	
	
	uint32_t divisor, bus_clock;
	SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	
	PORTE->PCR[23] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[23] |= PORT_PCR_MUX(4);
	
	UART2->C2 &= ~((UART_C2_RE_MASK));
	
	bus_clock = (DEFAULT_SYSTEM_CLOCK)/2;
	divisor = bus_clock/ (baud_rate*16);
	
	UART2->BDH = UART_BDH_SBR(divisor >> 8);
	UART2->BDL = UART_BDL_SBR(divisor);
	
	UART2->C1 = 0;
	UART2->S2 = 0;
	UART2->C3 = 0;
	
	
	//enable UART interrupt
	NVIC_SetPriority(UART2_IRQn, 2);
	NVIC_ClearPendingIRQ(UART2_IRQn);
	NVIC_EnableIRQ(UART2_IRQn);
	
	
	
		
	UART2->C2 |=(UART_C2_RE_MASK);
	
	UART2->C2 |= UART_C2_RIE_MASK;
	
	//initialize Message queue for uart data to tBrain thread
	brainQueue = osMessageQueueNew(Q_SIZE, sizeof(uint8_t), NULL);
}



volatile uint8_t brain_data;

void UART2_IRQHandler(void) {
	NVIC_ClearPendingIRQ(UART2_IRQn);
	
	
  if(UART2->S1 & UART_S1_RDRF_MASK) {
		//receive a character
		brain_data = UART2->D;
		
//		osMessageQueuePut(brainQueue, &brain_data, NULL, 0U);
//		osThreadFlagsSet(tBrain, 0x00000001U);
	}
	PORTE->ISFR = 0xffffffff;
}









void set_PS_PWM(uint8_t val) 
{	
	TPM_SC_REG(TPM_inst) &= ~TPM_SC_PS_MASK;
	TPM_SC_REG(TPM_inst) |= TPM_SC_PS(val);
}

//Control speed of motor
//Max value 25500
void set_CnV_Motor(uint16_t val, uint8_t TPM_CH)
{
	TPM_CnV_REG(TPM_inst,TPM_CH) = val;
}

void set_MOD_PWM(uint16_t MOD) 
{	
	TPM_MOD_REG(TPM_inst) = MOD;
}

void Motor_init(uint8_t PS_val) {
	//config output pin for PWM channel
	SIM->SCGC5 |= (SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTC_MASK);
	PTA->PDDR |= (MASK(4) | MASK(5));
	PTC->PDDR |= (MASK(8) | MASK(9));
	
	PORTA->PCR[4] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[5] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[8] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[9] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[4] |= PORT_PCR_MUX(3);
	PORTA->PCR[5] |= PORT_PCR_MUX(3);
	PORTC->PCR[8] |= PORT_PCR_MUX(3);
	PORTC->PCR[9] |= PORT_PCR_MUX(3);
	
	
	//enable clock TPM0
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;
	
	//select clk src for TPM
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
	
	
	set_MOD_PWM(MOD_VAL);
	
	
	//setup for up counting mode
	TPM_SC_REG(TPM_inst) &= ~TPM_SC_CPWMS_MASK;
	
	//setup for clock mode selection
	TPM_SC_REG(TPM_inst) &= ~((TPM_SC_CMOD_MASK));
	TPM_SC_REG(TPM_inst) |= (TPM_SC_CMOD(1));
	set_PS_PWM(PS_val);
	
	
	
	//channel configuration 
	TPM_CnSC_REG(TPM_inst,TPM_CH_FW_LEFT) &= ~((TPM_CnSC_MSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK));
	TPM_CnSC_REG(TPM_inst,TPM_CH_FW_LEFT) |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	TPM_CnSC_REG(TPM_inst,TPM_CH_BW_LEFT) &= ~((TPM_CnSC_MSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK));
	TPM_CnSC_REG(TPM_inst,TPM_CH_BW_LEFT) |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	TPM_CnSC_REG(TPM_inst,TPM_CH_FW_RIGHT) &= ~((TPM_CnSC_MSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK));
	TPM_CnSC_REG(TPM_inst,TPM_CH_FW_RIGHT) |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	TPM_CnSC_REG(TPM_inst,TPM_CH_BW_RIGHT) &= ~((TPM_CnSC_MSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK));
	TPM_CnSC_REG(TPM_inst,TPM_CH_BW_RIGHT) |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));	
}



/*
INPUT COMMAND FORMAT:
	1st MSB as finish bit
	2st MSB as left or right motor, 0 for left 1 for right
	3nd MSB if 0 negative value, 1 positive value
	the rest are magnitude
*/
motor_ins_t brain_command_process(uint8_t receiveChar) {
	motor_ins_t output;
	output.motor = receiveChar & 0b01000000;
	output.direction = receiveChar & 0b00100000;
	uint8_t speed_aft_mask = receiveChar & 0b00011111;
	float duty_cycle = ((float)speed_aft_mask * 100 / 32); //Max value of speed data is 2^5 - 1 = 31
	output.speed = duty_cycle * MOD_VAL / 100.0; 
	return output;
}



volatile uint8_t receiveChar = 0;
volatile 	motor_ins_t motorCmd;
void tBrain_Handler(void* argument) {
	
	motorQueue = osMessageQueueNew(Q_SIZE, sizeof(motor_ins_t), NULL);
	
	for(;;) {
//		osThreadFlagsWait(0x00000001U, osFlagsWaitAny, osWaitForever);
//		osMessageQueueGet(brainQueue, &receiveChar, NULL, osWaitForever);
		receiveChar = brain_data;
		
		if(brain_data & 0b10000000) { // handle when the finish signal is send
//			osThreadFlagsSet(tAudio, 0x00000001U);
//			osThreadFlagsSet(tMotorControl, 0x00000001U);
			osMutexAcquire(mutex_instance, osWaitForever);
		  robot_state = FINISH;
			osMutexRelease(mutex_instance);
			osThreadSuspend(tBrain);
		}
		else { // other case just proceed with the motor data
			motorCmd = brain_command_process(brain_data);
		}
		
		
		//changing robot state based on robot speed data
		if(motorCmd.speed == 0) {
			osMutexAcquire(mutex_instance, osWaitForever);
		  robot_state = STOP;
			osMutexRelease(mutex_instance);
		}
		else {
			osMutexAcquire(mutex_instance, osWaitForever);
		  robot_state = RUN;
			osMutexRelease(mutex_instance);
		}
		
		
		
		osMessageQueuePut(motorQueue, &motorCmd, NULL, osWaitForever);
				
		osThreadFlagsSet(tMotorControl, 0x00000001U);		
	}
}


/*----------------------------------------------------------------------------
 * Motor
 *---------------------------------------------------------------------------*/

void tMotorControl_Handler(void* argument) {
	
	motor_ins_t motorCmd;
	
	while(1) {
		osThreadFlagsWait(0x00000001U, osFlagsWaitAny,osWaitForever);
		if(robot_state == FINISH) { // stop all movement
				set_CnV_Motor(0, TPM_CH_FW_LEFT);
				set_CnV_Motor(0, TPM_CH_BW_LEFT);
				set_CnV_Motor(0, TPM_CH_FW_RIGHT);
				set_CnV_Motor(0, TPM_CH_BW_RIGHT);
				osThreadSuspend(tMotorControl);
		}
		osMessageQueueGet(motorQueue, &motorCmd, NULL, osWaitForever);
		if(motorCmd.motor == 0) {
			if(motorCmd.direction) {
				set_CnV_Motor(motorCmd.speed, TPM_CH_FW_LEFT);
				set_CnV_Motor(0, TPM_CH_BW_LEFT);
			}
			else {
				set_CnV_Motor(0, TPM_CH_FW_LEFT);
				set_CnV_Motor(motorCmd.speed, TPM_CH_BW_LEFT);
			}
		}
		else {
				if(motorCmd.direction) {
					set_CnV_Motor(motorCmd.speed, TPM_CH_FW_RIGHT);
					set_CnV_Motor(0, TPM_CH_BW_RIGHT);
				}
				else {
				set_CnV_Motor(0, TPM_CH_FW_RIGHT);
				set_CnV_Motor(motorCmd.speed, TPM_CH_BW_RIGHT);
				}	
		}
	}
}

/*----------------------------------------------------------------------------
 * LED
 *---------------------------------------------------------------------------*/

typedef enum {
    ONE = 7,
    TWO = 0,
    THREE = 3,
    FOUR = 4,
    FIVE = 5,
    SIX = 6,
    SEVEN = 10,
    EIGHT = 11
}GreenLEDenum;



void initLED(void) {
	//configure green led
	SIM->SCGC5 |= (SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTA_MASK);
	PTC->PDDR |= (MASK(0) | MASK(7) | MASK(3) | MASK(4) | MASK (6) | MASK(5) | MASK(10) | MASK(11));
	PORTC->PCR[0] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[0] |= PORT_PCR_MUX(1);
	PORTC->PCR[7] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[7] |= PORT_PCR_MUX(1);
	PORTC->PCR[3] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[3] |= PORT_PCR_MUX(1);
	PORTC->PCR[4] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[4] |= PORT_PCR_MUX(1);
	PORTC->PCR[5] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[5] |= PORT_PCR_MUX(1);
	PORTC->PCR[6] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[6] |= PORT_PCR_MUX(1);
	PORTC->PCR[10] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[10] |= PORT_PCR_MUX(1);
	PORTC->PCR[11] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[11] |= PORT_PCR_MUX(1);
	
	//configure red led
	PTA->PDDR |= MASK(12);
	PORTA->PCR[12] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[12] |= PORT_PCR_MUX(1);
}


GreenLEDenum getNextEnumValue(GreenLEDenum current) {
    switch(current) {
        case ONE: return TWO;
        case TWO: return THREE;
        case THREE: return FOUR;
        case FOUR: return FIVE;
        case FIVE: return SIX;
        case SIX: return SEVEN;
        case SEVEN: return EIGHT;
        case EIGHT: return ONE; // Wrap around to the first value
        default: return ONE;    // Default case
    }
}


void green_led_on(GreenLEDenum green_led) {
	PTC->PCOR &= ~MASK(green_led);
	PTC->PSOR |= MASK(green_led);
}


void green_led_off(GreenLEDenum green_led) {
	PTC->PSOR &= ~MASK(green_led);
	PTC->PCOR |= MASK(green_led);
}

void green_led_effect_run(GreenLEDenum current_led) {
		//Turn off all led
		green_led_off(ONE);
		green_led_off(TWO);
		green_led_off(THREE);
		green_led_off(FOUR);
		green_led_off(FIVE);
		green_led_off(SIX);
		green_led_off(SEVEN);
		green_led_off(EIGHT);
	
		
		green_led_on(current_led);
}

void green_led_effect_stop() {	
		green_led_on(ONE);
		green_led_on(TWO);
		green_led_on(THREE);
		green_led_on(FOUR);
		green_led_on(FIVE);
		green_led_on(SIX);
		green_led_on(SEVEN);
		green_led_on(EIGHT);
}


void red_led_on() {
	PTA->PCOR &= ~MASK(12);
	PTA->PSOR |= MASK(12);
}

void red_led_off() {
	PTA->PSOR &= ~MASK(12);
	PTA->PCOR |= MASK(12);
}

/*
TODO: Create an LED init for LED
Implement different LED State
*/
void tLED_Handler(void* argument) {
	GreenLEDenum green_led_state = ONE;
	uint8_t red_led_state = 0;
	while(1) {
		if(robot_state == RUN) {
				//when in state run, put LED here
				green_led_state = getNextEnumValue(green_led_state);
				green_led_effect_run(green_led_state);
				if(red_led_state == 0) {
					red_led_on();
					red_led_state = 1;
				}
				else {
					red_led_off();
					red_led_state = 0;
				}
				osDelay(500);
		}
		else {
			//When stationary or finished, put LED here
			green_led_effect_stop();
			if(red_led_state == 0) {
					red_led_on();
					red_led_state = 1;
			}
			else {
					red_led_off();
					red_led_state = 0;
			}
			osDelay(250);
		}
	}
}

/*----------------------------------------------------------------------------
 * Audio
 *---------------------------------------------------------------------------*/

void initAudio(void) {
  SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
  
  PORTB->PCR[PTB0_Pin] &= ~PORT_PCR_MUX_MASK;
  PORTB->PCR[PTB0_Pin] |= PORT_PCR_MUX(3);
  
  PORTB->PCR[PTB1_Pin] &= ~PORT_PCR_MUX_MASK;
  PORTB->PCR[PTB1_Pin] |= PORT_PCR_MUX(3);
  
  SIM_SCGC6 |= SIM_SCGC6_TPM1_MASK;

  SIM_SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
  SIM_SOPT2 |= SIM_SOPT2_TPMSRC(1);
  
	TPM1->MOD = 1432;
  TPM1_C0V = 1432/2;
	
  
  TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
  TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
  TPM1->SC &= ~(TPM_SC_CPWMS_MASK);
  
  TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
  TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
}



enum note_t {
  C = 0,
  D = 1,
  E = 2,
  F = 3,
  G = 4,
  A = 5,
  B = 6,
	P = 7
};

void playNote(enum note_t note) {
  switch (note) {
    case C:
      TPM1->MOD = 1432;
      TPM1_C0V = 1432/2;
      break;
    case D:
      TPM1->MOD = 1276;
      TPM1_C0V = 1276/2;
      break;
    case E:
      TPM1->MOD = 1137;
      TPM1_C0V = 1137/2;
      break;
    case F:
      TPM1->MOD = 1075;
      TPM1_C0V = 1075/2;
      break;
    case G:
      TPM1->MOD = 957;
      TPM1_C0V = 957/2;
      break;
    case A:
      TPM1->MOD = 853;
      TPM1_C0V = 853/2;
      break;
    case B:
      TPM1->MOD = 760;
      TPM1_C0V = 760/2;
      break;
		case P:
			TPM1_C0V = 1;
			break;
  }
}


void tAudio_Handler(void* argument) {
	for(;;) {
		if(robot_state == FINISH) {
			//put tune to play when finish here
	playNote(E); osDelay(200);
  playNote(P); osDelay(100);
  playNote(E); osDelay(200);
  playNote(P); osDelay(100);
  playNote(E); osDelay(200);
  playNote(P); osDelay(100);
  playNote(C); osDelay(200);
  playNote(P); osDelay(100);
  playNote(E); osDelay(200);
  playNote(P); osDelay(100);
  playNote(G); osDelay(400);
  playNote(P); osDelay(100);

  playNote(G); osDelay(400);
  playNote(P); osDelay(200);

  playNote(C); osDelay(200);
  playNote(P); osDelay(100);
  playNote(G); osDelay(200);
  playNote(P); osDelay(100);
  playNote(E); osDelay(300);
  playNote(P); osDelay(100);

  playNote(A); osDelay(400);
  playNote(P); osDelay(100);
  playNote(B); osDelay(400);
  playNote(P); osDelay(100);
  playNote(A); osDelay(300);
  playNote(P); osDelay(100);
  playNote(G); osDelay(300);

		}
		else {
		//put normal tune here
	playNote(C); osDelay(500);
		playNote(P); osDelay(250);		
  playNote(C); osDelay(500);
		playNote(P); osDelay(250);		
  playNote(G); osDelay(500);
		playNote(P); osDelay(250);		
  playNote(G); osDelay(500);
			playNote(P); osDelay(250);	
  playNote(A); osDelay(500);
		playNote(P); osDelay(250);		
  playNote(A); osDelay(500);
		playNote(P); osDelay(250);		
  playNote(G); osDelay(1000);
		playNote(P); osDelay(250);
  playNote(F); osDelay(500);
		playNote(P); osDelay(250);		
  playNote(F); osDelay(500);
		playNote(P); osDelay(250);		
  playNote(E); osDelay(500);
		playNote(P); osDelay(250);		
  playNote(E); osDelay(500);
		playNote(P); osDelay(250);
  playNote(D); osDelay(500);
		playNote(P); osDelay(250);
  playNote(D); osDelay(500);
		playNote(P); osDelay(250);
  playNote(C); osDelay(1000);
		playNote(P); osDelay(250);
		}	
	}
}

int main() {
	SystemCoreClockUpdate();
	initAudio();
	Motor_init(4);
	initLED();
	initUART2(9600);
	osKernelInitialize();
	tBrain = osThreadNew(tBrain_Handler, NULL, NULL);
	tMotorControl = osThreadNew(tMotorControl_Handler, NULL, NULL);
	tLED = osThreadNew(tLED_Handler, NULL,NULL);
	tAudio = osThreadNew(tAudio_Handler, NULL, NULL);
	mutex_instance = osMutexNew(NULL);
	osKernelStart();
}

