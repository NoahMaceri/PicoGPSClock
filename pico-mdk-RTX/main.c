/* Noah Maceri - Final Project (GPS Clock)*/

/*INCLUDES*/
//General
#include <string.h>
//Pico Specifics
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
//LCD Specifics
#include "DEV_Config.h"
#include "LCD_1In3.h"
#include "GLCD_Config.h"
#include "GUI_Paint.h"
//Keil Specifics
#include "perf_counter.h"
#include <EventRecorder.h>
//RTX5 Specifics
#include "cmsis_os2.h"
#include "RTE_Components.h"

/*DEFINES*/
//Datatypes
#define UBYTE   uint8_t
#define BYTE    int8_t
#define UWORD   uint16_t
#define WORD    int16_t
#define UDOUBLE uint32_t
#define DOUBLE  int32_t
//LCD Configs
#define xSize 			168
#define ySize				168
#define	sqOffset		36
//UART
#define UART_ID 		uart0
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define BUFFSIZE 107
//Device
#define SYS_CLOCK_SPEED 250000

/*FUNCTION DEFINITIONS*/
extern void SystemCoreClockUpdate();
char* UART_RX();
const char* getStatusName(DOUBLE osStatus);
void setup_GNSS();
void L76X_Update();
void location_screen();
void clock_screen();
void techdata_screen();
void gpio_callback(uint gpio, uint32_t events);
void system_info();

/*RTX Specific Configurations*/
//Threads
osThreadId_t setup_TID;    
osThreadId_t clock_screen_TID;
osThreadId_t location_screen_TID;
osThreadId_t techdata_screen_TID;

const osThreadAttr_t setup_attr = {
	.name = "SETUP_THREAD",
	.priority = osPriorityAboveNormal
};
const osThreadAttr_t clock_screen_attr = {
	.name = "CLOCK_SCREEN_THREAD",
	.priority = osPriorityNormal
};
const osThreadAttr_t location_screen_attr = {
	.name = "LOCATION_SCREEN_THREAD",
	.priority = osPriorityNormal 
};
const osThreadAttr_t techdata_screen_attr = {
	.name = "TECHDATA_SCREEN_THREAD",
	.priority = osPriorityNormal 
};

//Mutexes
osMutexId_t LCD_Mutex; 
const osMutexAttr_t LCD_mutex_attr = {
  "LCD_MUTEX",     // human readable mutex name
  osMutexRobust,   // attr_bits
  NULL,            // memory for control block   
  0U               // size for control block
};
//Event Flags
osEventFlagsId_t clock_screen_signal;
osEventFlagsId_t location_screen_signal;
osEventFlagsId_t techdata_screen_signal;
//Message Queues
osMessageQueueId_t sat_data;

/*GLOBAL VARIABLES*/
UWORD frameBuffer[56448]; //Width*Height*2 The reason we are downscaled is beacuse there is only 256000 bytes of ram
bool printGNSS = false;
typedef struct{
	UBYTE day;
	UBYTE month;
	UWORD year;
	UBYTE hour;
	UBYTE minute;
	UBYTE second;
	float lat;
	float lon;
	char lat_area;
	char lon_area;
	UBYTE lock_type;
	UBYTE num_of_sats;
	float altitude;
}satellite_data; 

/*CODE*/

//UART RX is a simple function that flushes the UART buffer to a char pointer
char* UART_RX() {
    char* tmp_string = malloc(BUFFSIZE);
		for(int i = 0; i<BUFFSIZE+1; i++) {
        char ch = uart_getc(UART_ID);
        tmp_string[i] = ch;
    }
		//Rotate char array to get correct offset
		int charCount = 105;
		int offset = 73;
		for(int i = 0; i<offset; i++)\
		{
			char x = tmp_string[charCount-1], i;
			for (i = charCount-1; i > 0; i--)
			{
				tmp_string[i] = tmp_string[i-1];
			}
			tmp_string[0] = x;
		}
		tmp_string[BUFFSIZE-2] = '\0';
    return tmp_string;
}

//Converts osStatus signed int to string representation
const char* getStatusName(DOUBLE osStatus) 
{
	switch (osStatus) 
	{
		default: 	return "Fail";
		case 0: 	return "osOk";
		case -1: 	return "osError";
		case -2: 	return "osErrorTimeout";
		case -3:	return "osErrorResource";
		case -4:	return "osErrorParameter";
		case -5:	return "osErrorNoMemory";
		case -6:	return "osErrorISR";
		case 0x7FFFFFFF: return "osStatusReserved";
	}
}

//Setup GNSS initilizes the LCD then sets up the Quectel L76B GNSS chip with our specific configuration
void setup_GNSS()
{
	system_info();
    DEV_Delay_ms(100);

    if(DEV_Module_Init()!=0){
			printf("Dev init failed");
    }
	DEV_SET_PWM(50);
    
    LCD_1IN3_Init(HORIZONTAL);
    LCD_1IN3_Clear(WHITE);
    
    for (int n = 0; n < KEY_NUM; n++) {
            dev_key_init(n);
    }
		
	//Interupts are enabled here to prevent false triggering
	//Must use RPi Pico Hardware interupts to interupt on GPIO changes (RTX reconizes this as an interupt)
	gpio_set_irq_enabled_with_callback(3, GPIO_IRQ_EDGE_RISE, true, &gpio_callback); //Center joystick
	gpio_set_irq_enabled(16, GPIO_IRQ_EDGE_RISE, true); //Left joystick
	gpio_set_irq_enabled(20, GPIO_IRQ_EDGE_RISE, true); //Right joystick
	gpio_set_irq_enabled(2, GPIO_IRQ_EDGE_RISE, true);  //Up joystick
	gpio_set_irq_enabled(21, GPIO_IRQ_EDGE_RISE, true); //Button Y

    //Alert user
    Paint_NewImage((UBYTE *)frameBuffer, xSize, ySize, 0, BLACK);
    Paint_SetScale(65);

    Paint_Clear(WHITE);
    Paint_DrawString_EN(7, 89, "Initilizing...", &Font16, WHITE, BLACK);

	osMutexAcquire(LCD_Mutex, osWaitForever);
	GLCD_DrawBitmap(sqOffset, sqOffset, xSize, ySize, frameBuffer); //Centered resolution - no upscailing
	osMutexRelease(LCD_Mutex);

    //CHECKSUMS ARE PRE-CALCULATED AND INCLUDED WITH INSTRUCTIONS TO SAVE MEMORY
	#define SET_NMEA_BAUDRATE_115200    "$PMTK251,115200*1F\r\n"                                //Set baudrate of GNSS module to 115200
    #define SET_POS_FIX_400MS           "$PMTK220,400*2A\r\n"                                   //Update position every 400ms
    #define SET_NMEA_OUTPUT             "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0*28\r\n" //Set NMEA sentence output frequencies (See page 35 of Quectel_L76_Series_GNSS_Protocol_Specification_V3.3)
																				//GLL, RMC, VTG, GGA, GSA, GSV, GRS, GST, ZDA
    #define SET_SYNC_PPS_NMEA_ON        "$PMTK255,1*2D\r\n"                                     //Enable fixed NMEA output times behind PPS function
    
    #define SETUP_BAUD_RATE 9600
    #define BAUD_RATE 115200
    
    #define FORCE_PIN 14 //When high, GNSS module has been forced on
	#define STANDBY_PIN 17 //When high, GNSS module is in standby
    
    printf("Initializing GNSS module, please wait...\n");
    
    //Init UART Pins and then setup UART with 9600 baud (Default GNSS rate)
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    UWORD baudRate = uart_init(UART_ID, SETUP_BAUD_RATE);
	printf("Initilized UART at: %i baud\n\n", baudRate);
		
    //Init force and standyby pins
    gpio_init(FORCE_PIN);
    gpio_set_dir(FORCE_PIN, GPIO_OUT);

    gpio_init(STANDBY_PIN);
    gpio_set_dir(STANDBY_PIN, GPIO_OUT);
    
    //Init pins to off
    gpio_put(FORCE_PIN, 0);
    gpio_put(STANDBY_PIN, 0);
    
    uart_puts(UART_ID, SET_NMEA_BAUDRATE_115200);
	printf("Sent: %s\n", SET_NMEA_BAUDRATE_115200);
    osDelay(2000);
		
    //Raise PICOs baud to match GNSS module
	printf("Restarting UART interface...\n");
	baudRate = uart_set_baudrate(UART_ID, BAUD_RATE) * 2;
    printf("Initilized UART at: %i baud\n\n", baudRate);
		
    //General GNSS module setup, set 900ms update interval, NMEA formatted packets, and sync NMEA output
    uart_puts(UART_ID, SET_POS_FIX_400MS);
	printf("Sent: %s\n", SET_POS_FIX_400MS);
    osDelay(500);

    uart_puts(UART_ID, SET_NMEA_OUTPUT);
	printf("Sent: %s\n", SET_NMEA_OUTPUT);
    osDelay(500);
    gpio_put(FORCE_PIN, 1);
    osDelay(2000);
    gpio_put(FORCE_PIN, 0);
    gpio_set_dir(FORCE_PIN, GPIO_IN);
		
    
    uart_puts(UART_ID, SET_SYNC_PPS_NMEA_ON);
	printf("Sent: %s\n", SET_SYNC_PPS_NMEA_ON);
    osDelay(500);
    
    Paint_Clear(WHITE);
    Paint_DrawString_EN(23, 89, "Initialized!", &Font16, WHITE, BLACK);

	osMutexAcquire(LCD_Mutex, osWaitForever);
	GLCD_DrawBitmap(sqOffset, sqOffset, xSize, ySize, frameBuffer); //Centered resolution - no upscailing
	osMutexRelease(LCD_Mutex);

    osDelay(500);
		
	LCD_1IN3_Clear(BLACK);  
	
	//All threads created here to ensure the setup thread completes without interuption
	clock_screen_TID = osThreadNew(clock_screen, NULL, &clock_screen_attr);
	location_screen_TID = osThreadNew(location_screen, NULL, &location_screen_attr);
	techdata_screen_TID = osThreadNew(techdata_screen, NULL, &techdata_screen_attr);
	osThreadSuspend(location_screen_TID);
	osThreadSuspend(techdata_screen_TID);
	
	osThreadTerminate	(setup_TID);	//free up the mem, setup thread only needs to be ran once
}

//L76X_Update receives the UART buffer and proceccess the raw strings into useable time, location, and tracking information
void L76X_Update()
{
    char* tmp_string = UART_RX();
	
		if(printGNSS)
		{
			printf("%s",tmp_string);
			printf("\n------------------------\n");
		}
		
		satellite_data data;
	
    char *pch = strtok (tmp_string, "\r\n");
    while (pch != NULL)
    {
        //It is inherent that you must check for a specifc NMEA packet in the order they are recieved
        //Data is accessed from a static position to avoid slowing in for loops
        if(pch[0] == '$' && pch[1] == 'G' && pch[2] == 'N' && pch[3] == 'Z' && pch[4] == 'D' && pch[5] == 'A')
        {
            //$GNZDA (ZDA (Zone Distribution Area) interval - Time and date)
						char *raw_day_str = (char*)malloc(3*sizeof(char));
            raw_day_str[0] = pch[18];
            raw_day_str[1] = pch[19];
            raw_day_str[2] = '\0';
            data.day = atoi(raw_day_str);
            
						char *raw_month_str = (char*)malloc(3*sizeof(char));
            raw_month_str[0] = pch[21];
            raw_month_str[1] = pch[22];
            raw_month_str[2] = '\0';
            data.month = atoi(raw_month_str);
            
						char *raw_year_str = (char*)malloc(5*sizeof(char));
            raw_year_str[0] = pch[24];
            raw_year_str[1] = pch[25];
            raw_year_str[2] = pch[26];
            raw_year_str[3] = pch[27];
            raw_year_str[4] = '\0';
            data.year = atoi(raw_year_str);
					
						free(raw_day_str);
						free(raw_month_str);
						free(raw_year_str);
        }
        else if(pch[0] == '$' && pch[1] == 'G' && pch[2] == 'N' && pch[3] == 'G' && pch[4] == 'G' && pch[5] == 'A')
        {
            //$GNGGA (GGA, Global Positioning System Fix Data)
						char *raw_time_str = (char*)malloc(7*sizeof(char));
            raw_time_str[0] = pch[7];
            raw_time_str[1] = pch[8];
            raw_time_str[2] = pch[9];
            raw_time_str[3] = pch[10];
            raw_time_str[4] = pch[11];
            raw_time_str[5] = pch[12];
            raw_time_str[6] = '\0';
            
            UDOUBLE raw_time = atoi(raw_time_str);
						//printf("Raw Time: %s\n", raw_time_str);
						//printf("Time: %d\n", raw_time);
            
            data.hour = (raw_time/10000); 		//First two digits
            data.minute = raw_time/100%100;	//Next two digits
            data.second = raw_time%100;			//Last two digits
            
						char *raw_lat_string = (char*)malloc(10*sizeof(char));
            raw_lat_string[0] = pch[18];
            raw_lat_string[1] = pch[19];
            raw_lat_string[2] = pch[20];
            raw_lat_string[3] = pch[21];
            raw_lat_string[4] = '.';
            raw_lat_string[5] = pch[23];
            raw_lat_string[6] = pch[24];
            raw_lat_string[9] = '\0';
            data.lat = atof(raw_lat_string);
            data.lat_area = pch[28];
            
						char *raw_lon_string = (char*)malloc(11*sizeof(char));
            raw_lon_string[0] = pch[30];
            raw_lon_string[1] = pch[31];
            raw_lon_string[2] = pch[32];
            raw_lon_string[3] = pch[33];
            raw_lon_string[4] = pch[34];
            raw_lon_string[5] = '.';
            raw_lon_string[6] = pch[36];
            raw_lon_string[7] = pch[37];
            raw_lon_string[10] = '\0';
            data.lon = atof(raw_lon_string);
            data.lon_area = pch[38]; 
						
						free(raw_lat_string);
						free(raw_lon_string);
						
            data.lock_type = (UBYTE)(pch[40]);
						char *raw_alt_string = (char*)malloc(6*sizeof(char));
            if(pch[43] == ',')
            {
								data.num_of_sats = pch[42] - '0'; //conversion from char rep of int to int
								raw_alt_string[0] = pch[49];
								raw_alt_string[1] = pch[50];
								raw_alt_string[2] = pch[51];
								raw_alt_string[3] = '.';
								raw_alt_string[4] = pch[53];
								raw_alt_string[5] = '\0';
            }
            else
            {
                char raw_sat_string[3];
                raw_sat_string[0] = pch[42];
                raw_sat_string[1] = pch[43];
                raw_sat_string[2] = '\0';
                data.num_of_sats = atoi(raw_sat_string);

								raw_alt_string[0] = pch[50];
								raw_alt_string[1] = pch[51];
								raw_alt_string[2] = pch[52];
								raw_alt_string[3] = '.';
								raw_alt_string[4] = pch[54];
								raw_alt_string[5] = '\0';
            }
						data.altitude = atof(raw_alt_string);
						free(raw_alt_string);
        }

        pch = strtok (NULL, "\r\n");
    }
		osStatus_t qStatus = osMessageQueuePut(sat_data, &data, 0U, 0U); 
		//printf("Q Put status: %s\n",getStatusName(qStatus));
    memset(tmp_string,0,strlen(tmp_string));
    free(tmp_string);
}


void techdata_screen()
{		
		while(1)
    {
			/*  
					Font24 - 17pixels/char
					Font20 - 14pixels/char
					Font16 - 11pixels/char
					To calculate offset:
							(xSize-(#numOfChars * fontPixPerChar))/2
			*/
			satellite_data data;
			osStatus_t qStatus = osMessageQueueGet(sat_data, &data, 0U, 0U);
			//printf("Q Get status: %s\n",getStatusName(qStatus));
			Paint_Clear(BLACK); //reset frame buffer
			Paint_DrawString_EN(7, 1, "TECH DATA", &Font24, BLACK, WHITE);
			L76X_Update();
			
			Paint_DrawString_EN(7, 40, "Num of Sats", &Font20, BLACK, MINT);
			char *numOfSatsString = (char*)malloc(3*sizeof(char));
			sprintf(numOfSatsString, "%02d", data.num_of_sats);  
			Paint_DrawString_EN(73, 65, numOfSatsString, &Font16, BLACK, WHITE);
			
			Paint_DrawString_EN(28, 90, "Altitude", &Font20, BLACK, MINT);
			char *altString = (char*)malloc(8*sizeof(char));
			sprintf(altString, "%05.2f M", data.altitude);
			Paint_DrawString_EN(40, 120, altString, &Font16, BLACK, WHITE);
			
			switch(data.lock_type){
					case '1':
							Paint_DrawString_EN(29, 150, "Lock: ", &Font16, BLACK, WHITE);
							Paint_DrawString_EN(88, 150, "GNSS", &Font16, BLACK, GREEN );
							break;
					case '2':
							Paint_DrawString_EN(29,  150, "Lock: ", &Font16, BLACK, WHITE);
							Paint_DrawString_EN(88, 150, "DGPS", &Font16, BLACK, GREEN );
							break;
					case '6':
							Paint_DrawString_EN(1, 150, "Lock: ", &Font16, BLACK, WHITE);
							Paint_DrawString_EN(62, 150, "Estimated", &Font16, BLACK, GOLD);
							break;
					default:
							Paint_DrawString_EN(29,  150, "Lock: ", &Font16, BLACK, WHITE); 
							Paint_DrawString_EN(88, 150, "None", &Font16, BLACK, RED);
							break;
			}
			osMutexAcquire(LCD_Mutex, osWaitForever);
			GLCD_DrawBitmap(sqOffset, sqOffset, xSize, ySize, frameBuffer); //Centered resolution - no upscailing
			osMutexRelease(LCD_Mutex);
			osDelay(200);
			if(osEventFlagsGet(clock_screen_signal) != 0)
			{
				osEventFlagsClear(clock_screen_signal, 0x00000001U);
				osStatus_t resume = osThreadResume(clock_screen_TID);
				printf("Resumed %s thread. Result: %s\n", osThreadGetName(clock_screen_TID), getStatusName(resume));
				osStatus_t suspend = osThreadSuspend(techdata_screen_TID);
			}
			else if(osEventFlagsGet(location_screen_signal) != 0)
			{
				osEventFlagsClear(location_screen_signal, 0x00000001U);
				osStatus_t resume = osThreadResume(location_screen_TID);
				printf("Resumed %s thread. Result: %s\n", osThreadGetName(location_screen_TID), getStatusName(resume));
				osStatus_t suspend = osThreadSuspend(techdata_screen_TID);
			}
		}
}

void location_screen()
{		
		while(1)
    {
			/*  
					Font24 - 17pixels/char
					Font20 - 14pixels/char
					Font16 - 11pixels/char
					To calculate offset:
							(xSize-(#numOfChars * fontPixPerChar))/2
			*/
			satellite_data data;
			osStatus_t qStatus = osMessageQueueGet(sat_data, &data, 0U, 0U);
			//printf("Q Get status: %s\n",getStatusName(qStatus));
			Paint_Clear(BLACK); //reset frame buffer
			Paint_DrawString_EN(16, 1, "LOCATION", &Font24, BLACK, WHITE);
			L76X_Update();
			
			Paint_DrawString_EN(28, 50, "Latitude", &Font20, BLACK, MINT);
			char *latString = (char*)malloc(10*sizeof(char));
			sprintf(latString, "%4.2f %c", data.lat, data.lat_area);  //Format: ffff.ff c (4 digits before decimal, two after the decimal)
			Paint_DrawString_EN(35, 80, latString, &Font16, BLACK, WHITE);
			Paint_DrawString_EN(21, 110, "Longitude", &Font20, BLACK, MINT);
			char *lonString = (char*)malloc(11*sizeof(char));
			sprintf(lonString, "%08.2f %c", data.lon, data.lon_area); //Format: fffff.ff c (seven digits in total, pad zeros, two after the decimal)
			Paint_DrawString_EN(29, 140, lonString, &Font16, BLACK, WHITE);
			
			osMutexAcquire(LCD_Mutex, osWaitForever);
			GLCD_DrawBitmap(sqOffset, sqOffset, xSize, ySize, frameBuffer); //Centered resolution - no upscailing
			osMutexRelease(LCD_Mutex);
			osDelay(200);
			
			free(latString);
			free(lonString);
			
			if(osEventFlagsGet(clock_screen_signal) != 0)
			{
				osEventFlagsClear(clock_screen_signal, 0x00000001U);
				osStatus_t resume = osThreadResume(clock_screen_TID);
				printf("Resumed %s thread. Result: %s\n", osThreadGetName(clock_screen_TID), getStatusName(resume));
				osStatus_t suspend = osThreadSuspend(location_screen_TID);
			}
			else if(osEventFlagsGet(techdata_screen_signal) != 0)
			{
				osEventFlagsClear(techdata_screen_signal, 0x00000001U);
				osStatus_t resume = osThreadResume(techdata_screen_TID);
				printf("Resumed %s thread. Result: %s\n", osThreadGetName(techdata_screen_TID), getStatusName(resume));
				osStatus_t suspend = osThreadSuspend(location_screen_TID);
			}
		}
}

void clock_screen()
{
    while(1)
    {
			/*  
					Font24 - 17pixels/char
					Font20 - 14pixels/char
					Font16 - 11pixels/char
					To calculate offset:
							(xSize-(#numOfChars * fontPixPerChar))/2
			*/
			satellite_data data;
			osStatus_t qStatus = osMessageQueueGet(sat_data, &data, 0U, 0U);
			//printf("Q Get status: %s\n",getStatusName(qStatus));
			Paint_Clear(BLACK); //reset frame buffer
			Paint_DrawString_EN(7, 1, "GPS CLOCK", &Font24, BLACK, WHITE);
			L76X_Update();

			char timeOfDay = 'A';
			//EST Adjustment
			if(data.hour - 4 <= 0) //rollover
			{
				switch(data.hour){
					case 0: 
						data.hour = (23-3);
						break;
					case 1: 
						data.hour = (23-2);
						break;
					case 2: data.hour = (23-1);
						break;
					case 3: data.hour = (23);
						break;
				}
				data.day = data.day - 1; //UTC adjustment
			}
			else
			{
				data.hour = data.hour - 4;
			}
			UBYTE hourConv = data.hour;
			//24 hour adjustment
			if(hourConv > 12)
			{
					hourConv = hourConv - 12; //24 hour adjusted
					timeOfDay = 'P';
			}
			else if(hourConv == 12)
			{
					timeOfDay = 'P';
			}
			char *timeString = (char*)malloc(12*sizeof(char));
			sprintf(timeString, "%d:%02d:%02d%cM", hourConv, data.minute, data.second, timeOfDay);
			//Adjust for single digit hour
			if(((UBYTE)(hourConv/10)) == 0)
			{
				Paint_DrawString_EN(21, 60, timeString, &Font20, BLACK, WHITE);
			}
			else
			{
				Paint_DrawString_EN(14, 60, timeString, &Font20, BLACK, WHITE);
			}


			memset(timeString,0,strlen(timeString));
			char *dateString = (char*)malloc(9*sizeof(char));
			sprintf(dateString, "%02d/%02d/%02d", data.month, data.day, data.year);
			Paint_DrawString_EN(14, 104, dateString, &Font20, BLACK, WHITE);
			memset(dateString,0,strlen(dateString));

			switch(data.lock_type){
					case '1':
							Paint_DrawString_EN(29, 150, "Lock: ", &Font16, BLACK, WHITE);
							Paint_DrawString_EN(88, 150, "GNSS", &Font16, BLACK, GREEN );
							break;
					case '2':
							Paint_DrawString_EN(29,  150, "Lock: ", &Font16, BLACK, WHITE);
							Paint_DrawString_EN(88, 150, "DGPS", &Font16, BLACK, GREEN );
							break;
					case '6':
							Paint_DrawString_EN(1, 150, "Lock: ", &Font16, BLACK, WHITE);
							Paint_DrawString_EN(62, 150, "Estimated", &Font16, BLACK, GOLD);
							break;
					default:
							Paint_DrawString_EN(29,  150, "Lock: ", &Font16, BLACK, WHITE); 
							Paint_DrawString_EN(88, 150, "None", &Font16, BLACK, RED);
							break;
			}
			osMutexAcquire(LCD_Mutex, osWaitForever);
			GLCD_DrawBitmap(sqOffset, sqOffset, xSize, ySize, frameBuffer); //Centered resolution - no upscailing
			osMutexRelease(LCD_Mutex);
			osDelay(200);
			
			free(timeString);
			free(dateString);
			
			if(osEventFlagsGet(location_screen_signal) != 0)
			{
				osEventFlagsClear(location_screen_signal, 0x00000001U);
				osStatus_t resume = osThreadResume(location_screen_TID);
				printf("Resumed %s thread. Result: %s\n", osThreadGetName(location_screen_TID), getStatusName(resume));
				osStatus_t suspend = osThreadSuspend(clock_screen_TID);
			}
			else if(osEventFlagsGet(techdata_screen_signal) != 0)
			{
				osEventFlagsClear(techdata_screen_signal, 0x00000001U);
				osStatus_t resume = osThreadResume(techdata_screen_TID);
				printf("Resumed %s thread. Result: %s\n", osThreadGetName(techdata_screen_TID), getStatusName(resume));
				osStatus_t suspend = osThreadSuspend(clock_screen_TID);
			}
		}
}

void system_info()
{
	char infobuf[11];
  osVersion_t osv;
  osStatus_t status;
 
  status = osKernelGetInfo(&osv, infobuf, sizeof(infobuf));
  if(status == osOK) {
    printf("Kernel Information: %s\r\n", infobuf);
    printf("Kernel Version    : %d\r\n", osv.kernel);
    printf("Kernel API Version: %d\r\n", osv.api);
  }
	/*
		Program Size: Code=40480 RO-data=15284 RW-data=288 ZI-data=189060  
		Total RAM usage = Total Program Size / 256Kb
										= 245112 / 256000 = .9574 = 95.7%
	*/
	printf("Total RAM Usage: 95.7%%\r\n");
	uint32_t tickFreq = osKernelGetTickFreq();
	printf("Kernel Tick Freq: %dHz\r\n\r\n", tickFreq);
	
}

//Interupt for GPIO, flags are set here to switch threads
void gpio_callback(uint gpio, uint32_t events) {
		if(gpio==3) //joystick center
		{
			//trigger primary screen
			osEventFlagsSet(clock_screen_signal, 0x00000001U);
			osDelay(500); //debounce
		}
    if(gpio==16) //joystick left
		{
			//trigger location screen
			osEventFlagsSet(location_screen_signal, 0x00000001U);
			osDelay(500); //debounce
		}
		if(gpio==20) //joystick right
		{
			//trigger technical data screen
			osEventFlagsSet(techdata_screen_signal, 0x00000001U);
			osDelay(500); //debounce
		}
		if(gpio==21) //Button Y
		{
			if(printGNSS == false)
			{
				printGNSS = true;
			}
			else if(printGNSS == true)
			{
				printGNSS = false;
			}
			osDelay(500); //debounce
		}
}

int main(void) 

{
		set_sys_clock_khz(SYS_CLOCK_SPEED, true); //Overclock Pico to 250MHz
		SystemCoreClockUpdate();
		#if defined(RTE_Compiler_EventRecorder) && defined(USE_EVR_FOR_STDOUR)
			EventRecorderInitialize(0, 1);
		#endif 
	
		//Hardware interupts are created in setup_GNSS to ensure no false triggering
		osKernelInitialize(); 
		LCD_Mutex = osMutexNew(&LCD_mutex_attr);
		clock_screen_signal = osEventFlagsNew(NULL);
		location_screen_signal = osEventFlagsNew(NULL);
		techdata_screen_signal = osEventFlagsNew(NULL);
		setup_TID = osThreadNew(setup_GNSS, NULL, &setup_attr);
		sat_data = osMessageQueueNew(2, sizeof(satellite_data), NULL);
		osKernelStart();   

    //Catch all, should never reach
    while (true);
}