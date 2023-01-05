struct Line line_temp;
struct Line *current_line = 0;
volatile int32_t counter_x, counter_y, counter_z;
uint32_t iterations;
#define max(a,b) (((a) > (b)) ? (a) : (b))
struct Line {
	uint32_t steps_x, steps_y, steps_z;
	int32_t maximum_steps;
	uint8_t direction_bits;
	uint32_t rate;
};

//Configures the prescaler and ceiling of timer 1 to produce the given rate as accurately as possible.
//Returns the actual number of cycles per interrupt
//cycles �ĵ�λ�� cpu ticks per step��ÿ������ռ�ö��ٸ�cpu����
//�����������ʵ�ֵ����pwmƵ��Ϊ1000khz������ֵ������TIM3->PSC=84-1��TIM3->ARR=1-1
//���貽��������ת��Ϊ3r/s��32ϸ�֡���pwmƵ��=3*32*200=19200hz����ÿ������Ϊ52.1us��16ϸ��Ϊ104.2us��
//cycles�ĵ�λ��microseconds/step һ��(ÿ������)����΢��
static uint32_t config_step_timer(uint32_t cycles)
{
	uint16_t ceiling;
	uint16_t prescaler;
	uint32_t actual_cycles;
	if (cycles <= 0xffffL)
	{ // 65536
		ceiling = cycles;
		prescaler = 1; // prescaler: ����Ƶ
		actual_cycles = ceiling;
	}
	else if (cycles <= 0x7ffffL)//0x10000L-0x7ffffL
	{ // 65536*8
		ceiling = cycles >> 3;//������λ��һ��ʱ��
		prescaler = 8; // prescaler: 8��Ƶ
		actual_cycles = ceiling * 8L;
	}
	else if (cycles <= 0x3fffffL)
	{ // 65536*64
		ceiling = cycles >> 6;
		prescaler = 64; // prescaler: 64��Ƶ
		actual_cycles = ceiling * 64L;
	}
	else if (cycles <= 0xffffffL)
	{ // 65536*256
		ceiling = (cycles >> 8);
		prescaler = 256; // prescaler: 256��Ƶ
		actual_cycles = ceiling * 256L;
	}
	else if (cycles <= 0x3ffffffL)//0x1000000L-0x3ffffffL
	{ // 65536*1024
		ceiling = (cycles >> 10);
		prescaler = 1024; // prescaler: 1024��Ƶ
		actual_cycles = ceiling * 1024L;
	}
	else
	{
		// Okay, that was slower than we actually go. Just set the slowest speed
		//����1024�ķ�Ƶ��Χ�Ļ�������Ķ�ʱ�����������ٶȣ�����psc�Ĵ�����16λ�ģ�65536/84=780.2����˶���780*84��Ƶ��
		ceiling = 0xffff;
		prescaler = 1024;
		actual_cycles = 0xffff * 1024;
	}
	// Set prescaler��avr�Ķ�ʱ��Ϊ16mhz��stm32��tim3Ϊ84mhz
	//�ɹ�ʽ�ɵ�arr*psc = 84*y/x = 84 * line->rate �� ��84��ֵ��TIM3->PSC����rate����cycles����ֵ��TIM3->ARR
	TIM3->PSC = prescaler * 84 - 1;
	TIM3->ARR = ceiling - 1; //�������Զ���װֵ
	//�趨pwm��ռ�ձ�Ϊ���ڵ�һ�룬��ȻҲ�����趨Ϊ1���̶�ֵ������10us��
	TIM4->PSC = TIM3->PSC;
	TIM4->ARR = (ceiling >> 1) - 1;
	return (actual_cycles);
}


void st_buffer_line(int32_t steps_x, int32_t steps_y, int32_t steps_z, uint32_t microseconds)
{
	struct Line *line = &line_temp;
	line->steps_x = labs(steps_x);
	line->steps_y = labs(steps_y);
	line->steps_z = labs(steps_z); 
	line->maximum_steps = max(line->steps_x, max(line->steps_y, line->steps_z));
	// Bail if this is a zero-length line
	if (line->maximum_steps == 0) { return; };
	line->rate = microseconds/line->maximum_steps;
	uint8_t direction_bits = 0;
	if (steps_x < 0) { direction_bits |= (1<<X_DIRECTION_BIT); }
	if (steps_y < 0) { direction_bits |= (1<<Y_DIRECTION_BIT); }
	if (steps_z < 0) { direction_bits |= (1<<Z_DIRECTION_BIT); }
	line->direction_bits = direction_bits;
	
	current_line = &line_temp;
	config_step_timer(current_line->rate);
	counter_x = -(current_line->maximum_steps >> 1);
	counter_y = counter_x;
	counter_z = counter_x;
	iterations = current_line->maximum_steps;
	
	TIM_Cmd(TIM3, ENABLE);
}
