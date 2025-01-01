#include "Arduino.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "U8g2lib.h"

// ----------- OLED 128x32 ------------------------------------------------------
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
// ------------------------------------------------------------------------------

/***************************************************************************
*	JJY Parameters
****************************************************************************/

enum rec_state {
   IDLE,
   PULSE_WIDHT_DET,
   PULSE_WAIT,
   JJY_TIME_OUT		/* assumed as receiving JJY call sign. */
};

enum bit_state {
   BIN_0,		/* binary 0 */
   BIN_1,		/* binary 1 */
   M,			/* Marker */
   PM,			/* Position Marker */
   UNKNOWN,
};

enum tone_state {
   TONE_E,
   TONE_T,
   TONE_N,
   TONE_J,
   TONE_Y,
   TONE_IDLE
};

#define nBlock	6
#define nBit	10

struct JJY_data {
	uint16_t	nMinute;
	uint16_t	nHour;
	uint16_t	nDay;
	uint16_t	nYear;
	uint16_t	nDay_of_week;
	uint16_t	month;
	uint16_t	day;
	uint16_t   day_of_week[3];
};

// Pulse Width (msec)
#define PW_M  200
#define PW_1  500
#define PW_0  800
#define PVAL  50    // pulse width should be (PW_x +/- PVAL)

#define TIME_PULSE_PIN D8
#define TIME_LED_PIN D9
#define TONE_PIN D10

/***************************************************************************
*	Resources for FreeRTOS
****************************************************************************/
xQueueHandle xQueue_Pule_Width;     /* Pulse width data from ISR */
xQueueHandle xQueue_JJY_Bit;     /* Queue Data of JJY bit from Task_JJY */
xQueueHandle xQueue_TONE;       /* Queue Data from Task_SWITCH -> Task_TONE */

void vHandlerTask(void *pvParameters);
void Task_JJY (void *pvParameters);
void Task_TONE (void *pvParameters);

/***************************************************************************
*  main()
****************************************************************************/
void setup()
{
    pinMode(TIME_PULSE_PIN, INPUT);
    pinMode(TIME_LED_PIN, OUTPUT);
    pinMode(TONE_PIN, OUTPUT);

    pinMode(16, OUTPUT);
    pinMode(17, OUTPUT);
    pinMode(25, OUTPUT);

    //GPIO 16, 17, 25 を HIGHにすることで消灯する
    digitalWrite(16, HIGH);
    digitalWrite(17, HIGH);
    digitalWrite(25, HIGH);

    //Serial.begin(9600);
    //delay(500);
    
    u8g2.begin();
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_8x13_tf);
    u8g2.drawStr(0, 10, "JJY Decoder  ");
    u8g2.drawStr(0, 21, "40kHz        ");
    //u8g2.drawStr(0, 32, "with U8g2lib");
    u8g2.sendBuffer();

    delay(500);
    Serial.println("***************************");
    Serial.println("***  JJY Decoder Pico   ***");
    Serial.println("****************************");
    Serial.println("Raspberry Pi Pico w/FreeRTOS");

    delay(500);
    u8g2.clearBuffer();
  
    /* Create Tasks */
    xQueue_JJY_Bit = xQueueCreate(4, 1);     /* 4 x 1 byte */
    xQueue_TONE = xQueueCreate(8, 1);     /* 4 x 1 byte */

    //xBinarySemaphore = xSemaphoreCreateBinary();
    
    xTaskCreate(Task_JJY, "JJY", 1024, NULL, 4, NULL);
    xTaskCreate(Task_OLED, "OLED", 1024, NULL, 3, NULL);
    xTaskCreate(Task_TONE, "TONE", 512, NULL, 2, NULL);

  /* Start RTOS Kernel */
  //vTaskStartScheduler();
}

void loop()
{
  // nothing to do
}

//*****************************************************************************
//  Task_JJY
//*****************************************************************************
//
void Task_JJY(void *pvParameters)
{
    enum bit_state last_bit = UNKNOWN;  /* Last received bit */
    uint32_t raise_time = 0;
    uint16_t pulse_width = 0;         /* pulse width (usec) */
    bool time_pulse = digitalRead(TIME_PULSE_PIN);
    bool pre_time_pulse = digitalRead(TIME_PULSE_PIN);
    
    const portTickType xDelay = 2u / portTICK_RATE_MS;
    
    for(;;)
    {
        time_pulse = digitalRead(TIME_PULSE_PIN);
        if (time_pulse != pre_time_pulse) {
            uint32_t now = millis();
            if (time_pulse) {
                digitalWrite(TIME_LED_PIN, HIGH);
                raise_time = now;
            } else {
                digitalWrite(TIME_LED_PIN, LOW);
                if (raise_time != 0) {
                      pulse_width = (uint16_t)(now - raise_time);
                      raise_time = 0;
                }
            }
            pre_time_pulse = time_pulse;
        }

        /********************************************
        *  Falling edge of a pulse is detected
        *********************************************/
        if (pulse_width > PW_M - PVAL)
        {
            /* Marker or Position Marker */
            if ((pulse_width > PW_M - PVAL) && (pulse_width < PW_M + PVAL))
            {
                /* Are there two continuous Markers?�@*/
                if (last_bit == PM)
                  last_bit = M;
                else
                  last_bit = PM;
            }
            else if ((pulse_width > PW_1 - PVAL) && (pulse_width < PW_1 + PVAL))
                last_bit = BIN_1;
            else if ((pulse_width > PW_0 - PVAL) && (pulse_width < PW_0 + PVAL))
                last_bit = BIN_0;
            else
                last_bit = UNKNOWN;
    
            /* Send the received data to Task_OLED */
            xQueueSend(xQueue_JJY_Bit, &last_bit, 0);
            //Serial.printf(" %d ms", pulse_width);
            pulse_width = 0;
        }
        vTaskDelay(xDelay);
    }
}

/***************************************************************************
*	Task_OLED
****************************************************************************/
void BCD_decode_data (uint8_t block);
uint32_t BCD2time (uint8_t block);
void display_rx_data (void);
void display_time (uint8_t block);
void nDay_to_calendar (void);
bool JJY_period = false;

struct JJY_data	ct;     /* current JJY decoded data */
enum bit_state tmp_buf[nBlock][nBit];	   /* 1 frame temp data */

    //u8g2.clearBuffer();
    //String s1 = "                ";
    String s1 = "20YY/MM/DD HH/MM";
    String s2 = "                ";
    //String s3 = "(n) xxxxxxxxxx  ";
    String s3 = "                ";

void Task_OLED(void *pvParameters)
{
  	enum bit_state last_bit = UNKNOWN;	/* Last received bit */
    enum tone_state tone_flag;

    bool OLED_sync = false; 
    bool OLED_update = false; 
    uint8_t OLED_block = 0;
    uint8_t OLED_bit = 0;
    uint8_t JJY_call_count = 0;
	char buf[32];           /* OLED */

    const portTickType xDelay = 1u      / portTICK_RATE_MS;

    u8g2.clearBuffer();
    u8g2.drawStr(0, 10, s1.c_str());
    u8g2.drawStr(0, 21, s2.c_str());
    u8g2.drawStr(0, 32, s3.c_str());
    u8g2.sendBuffer();

    ct.nMinute = 0;
	ct.nHour = 0;
	ct.nDay = 0;
	ct.nYear = 0;
	ct.nDay_of_week = 0;
	ct.month = 0;
	ct.day = 0;

    for(;;)
    {
        if (uxQueueMessagesWaiting(xQueue_JJY_Bit) > 0)
        {
            xQueueReceive(xQueue_JJY_Bit, &last_bit, 0);
            
            if (JJY_period) /* JJY Call? */
            {
                JJY_call_count++;
                /* JJY call is "JJY JJY" in Morse code 
                *-----------------------------------------------------------------------------
                *     01 02 03    04 05 06 07    08 09    10 11 12    13 14 15 16    17 18 19
                *  .  -  -  -  .  -  -  -  -  .  -  -  .  -  -  -  .  -  -  -  -  .  -  -  P
                *-----------------------------------------------------------------------------
                *     (4)   (J)        (J)         (Y)          (_J)        (J)         (Y) ; OLED
                *     (JJY)                        (JJY)                                    ; TONE
                */
                switch (JJY_call_count)
                {
                    case 1:
                        s3 = "(4)"; OLED_update = true;
                        tone_flag = TONE_J; xQueueSend(xQueue_TONE, &tone_flag, 0);
                        tone_flag = TONE_J; xQueueSend(xQueue_TONE, &tone_flag, 0);
                        tone_flag = TONE_Y; xQueueSend(xQueue_TONE, &tone_flag, 0);
                        break;
                    case 3:
                    case 6:
                        s3 += ("J"); OLED_update = true;
                        break;
                    case 9:
                        s3 += ("Y"); OLED_update = true;
                        tone_flag = TONE_J; xQueueSend(xQueue_TONE, &tone_flag, 0);
                        tone_flag = TONE_J; xQueueSend(xQueue_TONE, &tone_flag, 0);
                        tone_flag = TONE_Y; xQueueSend(xQueue_TONE, &tone_flag, 0);
                        break;
                    case 12:
                        s3 += (" J"); OLED_update = true;
                        break;
                    case 15:
                        s3 += ("J"); OLED_update = true;
                        break;
                    case 18:
                        s3 += ("Y"); OLED_update = true;
                        break;
                    // P Marker
                    case 19:
                        s3 += (" P"); OLED_update = true;
                        OLED_block = 5;
                		OLED_bit = 0;
                        JJY_period = false;      /* set Time mode */
                        break;
                }
                if (OLED_update) {
                        // OLED update
                        u8g2.clearBuffer();
                        u8g2.drawStr(0, 10, s1.c_str());
                        u8g2.drawStr(0, 21, s2.c_str());
                        u8g2.drawStr(0, 32, s3.c_str());
                        u8g2.sendBuffer();
                        OLED_update = false;
                }
                continue;   /* skip the followings */
            }

        	if (last_bit == M)
        	{
        		OLED_sync = true;
        		OLED_block = 0;
        		OLED_bit = 0;
                JJY_call_count = 0;
        	}
        	/* character position */
        	if (OLED_bit == 0)
        	{
        		if (OLED_sync)
        		{
                    sprintf(buf, "(%d)             ", OLED_block);
                    s3 = buf;
        		}
        		else
        			s3 = ("(x)             ");
        	}

        	/* print received data */
        	if (last_bit == M)
        		s3.setCharAt((OLED_bit + 3), 'M');
            else if (last_bit == PM)
                s3.setCharAt((OLED_bit + 3), 'P');
        	else if (last_bit == BIN_0)
                s3.setCharAt((OLED_bit + 3), '0');
        	else if (last_bit == BIN_1)
                s3.setCharAt((OLED_bit + 3), '1');
        	else if (last_bit == UNKNOWN)
        	{
                s3.setCharAt((OLED_bit + 3), '?');
        		OLED_sync = false;
        	}

            tmp_buf[OLED_block][OLED_bit] = last_bit;

        	OLED_bit++;
            if (OLED_bit > 10)
            {
                OLED_sync = false; 
                OLED_bit = 0;
                OLED_block = 0;
            }

        	if (last_bit == M)
        	{
                /* Tone Signal */
				if (ct.nMinute == 59)      /* if the last time was 59 min, */
                    tone_flag = TONE_N;   /* this marker is for 0 min period. */
                else
                    tone_flag = TONE_E;
                xQueueSend(xQueue_TONE, &tone_flag, 0);
            }

            if (last_bit == PM)
        	{
          		/* Display the current time on the 2nd line */
                if (OLED_sync)
                {
                    BCD_decode_data(OLED_block);
            		display_time(OLED_block);
                }
                /* JJY Call check the next block is JJY */
    			if (((ct.nMinute == 15) || (ct.nMinute == 45)) && (OLED_block == 3))
                    JJY_period = true;      /* set JJY mode */
                else
                    JJY_period = false;      /* set Time mode */

                /* next block */
                OLED_block++;
                if (OLED_block > 5)
                {
                    OLED_sync = false; 
                    OLED_block = 0;
                }
        		OLED_bit = 0;
            }
            // OLED update
            u8g2.clearBuffer();
            u8g2.drawStr(0, 10, s1.c_str());
            u8g2.drawStr(0, 21, s2.c_str());
            u8g2.drawStr(0, 32, s3.c_str());
            u8g2.sendBuffer();
        }
        vTaskDelay(xDelay);
    }
}

/***************************************************
*  Decode the time data
****************************************************/
void BCD_decode_data (uint8_t index)
{
	switch(index)
	{
		case 0:
			ct.nMinute = BCD2time(index);
			break;
		case 1:
			ct.nHour = BCD2time(index);
			break;
		case 3:
			ct.nDay = BCD2time(index - 1) + BCD2time(index);
			break;
		case 4:
			if ((ct.nMinute != 15) && (ct.nMinute != 45))
				ct.nYear = BCD2time(index);
			break;
		case 5:
			ct.nDay_of_week = BCD2time(index);
			break;
	}
}

/***************************************************
*  BCD code to Time data
****************************************************/
uint32_t BCD2time (uint8_t block)
{
        /* BCD code table */
    const uint8_t BCD_table [nBlock][nBit] =
    {
    	{ 0, 40, 20, 10,  0, 8, 4, 2, 1, 0 },		// minute
    	{ 0,  0, 20, 10,  0, 8, 4, 2, 1, 0 },		// hour
    	{ 0,  0, 200, 100, 0, 80, 40, 20, 10, 0 },	// upper day
    	{ 8,  4,  2,  1,  0, 0, 0, 0, 0, 0 },		// lower day
    	{ 0, 80, 40, 20, 10, 8, 4, 2, 1, 0 },		// lower year (+2000)
    	{ 4,  2,  1,  0,  0, 0, 0, 0, 0, 0 }		// days of the week
    };
    
	uint32_t time_code = 0;
	uint8_t bit;

	for (bit = 0; bit < nBit; bit++)
	{
		if (tmp_buf[block][bit] == BIN_1)
			time_code += BCD_table[block][bit];
	}
	return time_code;
}

/***************************************************
*  Display the time on OLED
****************************************************/
/*
* display format is "20YY/MM/DD HH:MM"
*                    0000000000111111
*                    0123456789012345  
*/
String str_YY = "20yy/";
String str_MM_DD = "mm/dd ";
String str_HH = "hh:";
String str_MM = "mm";

void display_time (uint8_t block)
{
	char buf[32];	/* buffer */

	switch (block)
	{
		case 0:	/* minute */
			sprintf(buf, "%02d", ct.nMinute);
            str_MM = buf;
			break;
		case 1: /* hour */
			sprintf(buf, "%02d:", ct.nHour);
            str_HH = buf;
			break;
		case 3: /* calendar */
			nDay_to_calendar();
			sprintf(buf, "%02d/%02d ", ct.month, ct.day);
            str_MM_DD = buf;
			break;
		case 4: /* year */
			sprintf(buf, "20%02d/", ct.nYear);
            str_YY = buf;
			break;
		case 5: /* day of the week */
			/* there is no space to use */
			break;
	}
    s1 = str_YY + str_MM_DD + str_HH + str_MM;
}

/***************************************************
*  The day number in the year to calendar
****************************************************/
void nDay_to_calendar (void)
{
    //const char week[7][4] = { "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat" };
    const unsigned int norm_month_Days[13] = { 0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
    const unsigned int leap_month_Days[13] = { 0, 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

	uint8_t i;
	uint16_t sum_of_month_days = 0;
	bool leap_year = false;
    uint8_t monthDays[13] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

	if ((ct.nYear % 4) == 0)
		leap_year = true;

    for (i = 1; i < 13; i++)
    {
        if (leap_year == false)
            monthDays[i] = norm_month_Days[i];
        else
            monthDays[i] = leap_month_Days[i];
    }

	for (i = 1; i < 13; i++)
	{
		sum_of_month_days += monthDays[i];
		if( ct.nDay <= sum_of_month_days )
		{
			ct.month = i;
			ct.day = ct.nDay - (sum_of_month_days - monthDays[i]);
			break;
		}
	}
}

//*****************************************************************************
//  T a s k _ T O N E
//*****************************************************************************
//
#define T_LEVEL 20 

void Task_TONE(void *pvParameters)
{
    const portTickType xDelay    = 100u   / portTICK_RATE_MS;
    const portTickType dotDelay  = 80u / portTICK_RATE_MS;
    const portTickType dashDelay = 240u / portTICK_RATE_MS;

    tone_state tone_flag;

    /* start the tone generator */
    //analogWriteFreq(1500);
    analogWriteFreq(800);
    analogWriteRange(1023);
    // Generate a voltage between 0 and 3.3V.
    // 0= 0V, 255=3.3V, 127=1.65V, 77 = 1.0V, etc.
    //analogWrite(DAC_PIN, dacVoltage);
    analogWrite(TONE_PIN, 0);

    for(;;)
    {
        /* Get tone generate request from Task_JJT */
        if (uxQueueMessagesWaiting(xQueue_TONE) > 0)
        {
            xQueueReceive(xQueue_TONE, &tone_flag, 0);

            switch(tone_flag)
            {
                case TONE_E:     /* 100ms */
                    analogWrite(TONE_PIN, T_LEVEL); vTaskDelay(dotDelay); analogWrite(TONE_PIN, 0);
                    vTaskDelay(dashDelay);
                    break;
                case TONE_T:     /* 100ms */
                    analogWrite(TONE_PIN, T_LEVEL); vTaskDelay(dashDelay); analogWrite(TONE_PIN, 0);
                    vTaskDelay(dashDelay);
                    break;
                case TONE_N:     /* 300ms + 100ms */
                    analogWrite(TONE_PIN, T_LEVEL); vTaskDelay(dashDelay); analogWrite(TONE_PIN, 0);
                    vTaskDelay(dotDelay);
                    analogWrite(TONE_PIN, T_LEVEL); vTaskDelay(dotDelay); analogWrite(TONE_PIN, 0);
                    vTaskDelay(dashDelay);
                    break;
                case TONE_J:
                    analogWrite(TONE_PIN, T_LEVEL); vTaskDelay(dotDelay); analogWrite(TONE_PIN, 0);
                    vTaskDelay(dotDelay);
                    analogWrite(TONE_PIN, T_LEVEL); vTaskDelay(dashDelay); analogWrite(TONE_PIN, 0);
                    vTaskDelay(dotDelay);
                    analogWrite(TONE_PIN, T_LEVEL); vTaskDelay(dashDelay); analogWrite(TONE_PIN, 0);
                    vTaskDelay(dotDelay);
                    analogWrite(TONE_PIN, T_LEVEL); vTaskDelay(dashDelay); analogWrite(TONE_PIN, 0);
                    vTaskDelay(dashDelay);
                    break;
                case TONE_Y:
                    analogWrite(TONE_PIN, T_LEVEL); vTaskDelay(dashDelay); analogWrite(TONE_PIN, 0);
                    vTaskDelay(dotDelay);
                    analogWrite(TONE_PIN, T_LEVEL); vTaskDelay(dotDelay); analogWrite(TONE_PIN, 0);
                    vTaskDelay(dotDelay);
                    analogWrite(TONE_PIN, T_LEVEL); vTaskDelay(dashDelay); analogWrite(TONE_PIN, 0);
                    vTaskDelay(dotDelay);
                    analogWrite(TONE_PIN, T_LEVEL); vTaskDelay(dashDelay); analogWrite(TONE_PIN, 0);
                    vTaskDelay(dashDelay);
                    break;
                default:
                    tone_flag = TONE_IDLE;
                    vTaskDelay(dashDelay);
                    break;
            }
            tone_flag = TONE_IDLE;

        
        
        }
        vTaskDelay(xDelay);
    }
}

/* [] END OF FILE */
