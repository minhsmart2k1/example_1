/*
This program demonstrate how to use hps communicate with FPGA through light AXI Bridge.
uses should program the FPGA by GHRD project before executing the program
refer to user manual chapter 7 for details about the demo
*/


#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/mman.h>
#include "stdint.h"
#include "hwlib.h"
#include "socal/socal.h"
#include "socal/hps.h"
#include "socal/alt_gpio.h"
#include "hps_0.h"
#include "oc_i2c_master.h"
#include "oc_i2c_master_regs.h"

#include "iothub.h"
#include "iothub_device_client_ll.h"
#include "iothub_client_options.h"
#include "iothub_message.h"
#include "azure_c_shared_utility/threadapi.h"
#include "azure_c_shared_utility/crt_abstractions.h"
#include "azure_c_shared_utility/shared_util_options.h"
#include "parson.h"

#ifdef SET_TRUSTED_CERT_IN_SAMPLES
#include "certs.h"
#endif // SET_TRUSTED_CERT_IN_SAMPLES

/* This sample uses the _LL APIs of iothub_client for example purposes.
Simply changing the using the convenience layer (functions not having _LL)
and removing calls to _DoWork will yield the same results. */

// The protocol you wish to use should be uncommented
//
#define SAMPLE_MQTT
//#define SAMPLE_MQTT_OVER_WEBSOCKETS
//#define SAMPLE_AMQP
//#define SAMPLE_AMQP_OVER_WEBSOCKETS
//#define SAMPLE_HTTP

#ifdef SAMPLE_MQTT
    #include "iothubtransportmqtt.h"
#endif // SAMPLE_MQTT
#ifdef SAMPLE_MQTT_OVER_WEBSOCKETS
    #include "iothubtransportmqtt_websockets.h"
#endif // SAMPLE_MQTT_OVER_WEBSOCKETS
#ifdef SAMPLE_AMQP
    #include "iothubtransportamqp.h"
#endif // SAMPLE_AMQP
#ifdef SAMPLE_AMQP_OVER_WEBSOCKETS
    #include "iothubtransportamqp_websockets.h"
#endif // SAMPLE_AMQP_OVER_WEBSOCKETS
#ifdef SAMPLE_HTTP
    #include "iothubtransporthttp.h"
#endif // SAMPLE_HTTP


/* Paste in the your iothub connection string  */
static const char* connectionString = "HostName=de10-nano-hub.azure-devices.net;DeviceId=de10-standard;SharedAccessKey=smlA19CBRRjYGKkVkzMvwdJ6D/DYCR5rN6jGc4W6q5k=";
//static const char* connectionString = "HostName=de10-nano-hub.azure-devices.net;SharedAccessKeyName=iothubowner;SharedAccessKey=hh5hnXVtVnXaLpz2xDkHGr03HNgMQ/NORrTpQMHFoD8=";
#define MESSAGE_COUNT        5
static bool g_continueRunning = true;
static size_t g_message_count_send_confirmations = 0;

static void send_confirm_callback(IOTHUB_CLIENT_CONFIRMATION_RESULT result, void* userContextCallback)
{
    (void)userContextCallback;
    // When a message is sent this callback will get invoked
    g_message_count_send_confirmations++;
    (void)printf("Confirmation callback received for message %lu with result %s\r\n", (unsigned long)g_message_count_send_confirmations, MU_ENUM_TO_STRING(IOTHUB_CLIENT_CONFIRMATION_RESULT, result));
}

static void connection_status_callback(IOTHUB_CLIENT_CONNECTION_STATUS result, IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason, void* user_context)
{
    (void)reason;
    (void)user_context;
    // This sample DOES NOT take into consideration network outages.
    if (result == IOTHUB_CLIENT_CONNECTION_AUTHENTICATED)
    {
        (void)printf("The device client is connected to iothub\r\n");
    }
    else
    {
        (void)printf("The device client has been disconnected\r\n");
    }
}


#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )

//===================================================================================
void oc_i2c_master_init(int base, int freq);

void oc_i2c_master_write(oc_i2c_master_dev *i2c_dev, unsigned char address, unsigned char data);
void oc_i2c_master_write_reg(oc_i2c_master_dev *i2c_dev, unsigned char address, unsigned char reg, unsigned char data);

unsigned char oc_i2c_master_read(oc_i2c_master_dev *i2c_dev, unsigned char address);
unsigned char oc_i2c_master_read_reg(oc_i2c_master_dev *i2c_dev, unsigned char address, unsigned char reg);

void IOWR_OC_I2C_MASTER_PRERLO(volatile void * base, unsigned char data) ;
unsigned char IORD_OC_I2C_MASTER_PRERLO(volatile void * base) ;
void IOWR_OC_I2C_MASTER_PRERHI(volatile void * base, unsigned char data);
unsigned char IORD_OC_I2C_MASTER_PRERHI(volatile void * base);
void IOWR_OC_I2C_MASTER_CTR(volatile void * base, unsigned char data);
unsigned char IORD_OC_I2C_MASTER_CTR(volatile void * base);
void IOWR_OC_I2C_MASTER_TXR(volatile void * base, unsigned char data);
unsigned char IORD_OC_I2C_MASTER_RXR(volatile void * base);
void IOWR_OC_I2C_MASTER_CR(volatile void * base, unsigned char data);
unsigned char IORD_OC_I2C_MASTER_SR(volatile void * base);
//===================================================================================
	
int main() {

	void *virtual_base;
	int fd;
	int loop_count;
	int led_direction;
	int led_mask;
	void *i2c_master, *i2c_slave;

	// map the address space for the LED registers into user space so we can interact with them.
	// we'll actually map in the entire CSR span of the HPS since we want to access various registers within that span

	if( ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) {
		printf( "ERROR: could not open \"/dev/mem\"...\n" );
		return( 1 );
	}

	virtual_base = mmap( NULL, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, HW_REGS_BASE );

	if( virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap() failed...\n" );
		close( fd );
		return( 1 );
	}
	//==================================================================================//

	//i2c_master = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + I2C_MASTER_0_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	i2c_slave = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + I2C_SLAVE_0_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	//==================================================================================//

	// oc_i2c_master_dev  *ptr = (oc_i2c_master_dev*)malloc(sizeof(oc_i2c_master_dev));
	// ptr->base = i2c_master;

	// oc_i2c_master_init(i2c_master,50000000);
	//---------------------------------------------
	int i = 0;
  int j; 
	unsigned char t = 0;
  unsigned char rx[]= {};
	//test Master - Arduino's Slave
	/*
	printf("Master write == 0x%x\n", 9);
	oc_i2c_master_write(ptr,I2C_SLAVE_ADDR,9);

	t = oc_i2c_master_read(ptr,I2C_SLAVE_ADDR);
	printf("Master read == 0x%x\n", t);
	*/

  /*azure iot hub*/

  IOTHUB_CLIENT_TRANSPORT_PROVIDER protocol;
    IOTHUB_MESSAGE_HANDLE message_handle;
    size_t messages_sent = 0;
    const char* telemetry_msg = "hello";
	const unsigned char data[] = {0x01, 0x02, 0x03, 0x04, 0x05}; // Mảng byte dữ liệu từ cảm biến

    // Select the Protocol to use with the connection
#ifdef SAMPLE_MQTT
    protocol = MQTT_Protocol;
#endif // SAMPLE_MQTT
#ifdef SAMPLE_MQTT_OVER_WEBSOCKETS
    protocol = MQTT_WebSocket_Protocol;
#endif // SAMPLE_MQTT_OVER_WEBSOCKETS
#ifdef SAMPLE_AMQP
    protocol = AMQP_Protocol;
#endif // SAMPLE_AMQP
#ifdef SAMPLE_AMQP_OVER_WEBSOCKETS
    protocol = AMQP_Protocol_over_WebSocketsTls;
#endif // SAMPLE_AMQP_OVER_WEBSOCKETS
#ifdef SAMPLE_HTTP
    protocol = HTTP_Protocol;
#endif // SAMPLE_HTTP

    // Used to initialize IoTHub SDK subsystem
    (void)IoTHub_Init();

    IOTHUB_DEVICE_CLIENT_LL_HANDLE device_ll_handle;

    (void)printf("Creating IoTHub Device handle\r\n");
    // Create the iothub handle here
    device_ll_handle = IoTHubDeviceClient_LL_CreateFromConnectionString(connectionString, protocol);
    if (device_ll_handle == NULL)
    {
        (void)printf("Failure creating IotHub device. Hint: Check your connection string.\r\n");
    }
    else
    {
        // Set any option that are necessary.
        // For available options please see the iothub_sdk_options.md documentation

#ifndef SAMPLE_HTTP
        // Can not set this options in HTTP
        bool traceOn = true;
        IoTHubDeviceClient_LL_SetOption(device_ll_handle, OPTION_LOG_TRACE, &traceOn);
#endif

#ifdef SET_TRUSTED_CERT_IN_SAMPLES
        // Setting the Trusted Certificate. This is only necessary on systems without
        // built in certificate stores.
            IoTHubDeviceClient_LL_SetOption(device_ll_handle, OPTION_TRUSTED_CERT, certificates);
#endif // SET_TRUSTED_CERT_IN_SAMPLES

#if defined SAMPLE_MQTT || defined SAMPLE_MQTT_OVER_WEBSOCKETS
        //Setting the auto URL Encoder (recommended for MQTT). Please use this option unless
        //you are URL Encoding inputs yourself.
        //ONLY valid for use with MQTT
        bool urlEncodeOn = true;
        (void)IoTHubDeviceClient_LL_SetOption(device_ll_handle, OPTION_AUTO_URL_ENCODE_DECODE, &urlEncodeOn);
#endif

        // Setting connection status callback to get indication of connection to iothub
        (void)IoTHubDeviceClient_LL_SetConnectionStatusCallback(device_ll_handle, connection_status_callback, NULL);


        // Create JSON object to hold temperature data
    JSON_Value *rootValue = json_value_init_object();
    JSON_Object *rootObject = json_value_get_object(rootValue);

    float temperature = 30.4;
    float humidity = 60.7;

        while (1)
        {
          /* Read data by I2C*/
   
             printf("Slave at reg 2 changes\n");
    for (j = 0; j < 19; ){
      if (*(unsigned int*)(i2c_slave + 2*4) != 99)
		{
			rx[j] = *(unsigned int*)(i2c_slave + 2*4);
			printf("rx[%d]: 0x%x\n", j, rx[j]);
      *(unsigned int*)(i2c_slave + 2*4) = 99;
      j++;

		}
		}
    printf("Hr: %d, SpO2: %.2f, Temp: %.2f\n", rx[0], 
        (float) ((rx[1] << 24 | rx[2] << 16 | rx[3] << 8 | rx[4]) / 100.0),
        (float) ((rx[5] << 8 | rx[6]) / 100.0));
    printf("AccX: %d, AccY: %d, AccY: %d\n", (int16_t)(rx[8] << 8)
                  | rx[7], (int16_t)(rx[10] << 8)
                  | rx[9], (int16_t)(rx[12] << 8)
                  | rx[11]);
   printf("gyroX: %d, gyroY: %d, gyroY: %d\n", (int16_t)(rx[14] << 8)
                  | rx[13], (int16_t)(rx[16] << 8)
                  | rx[15], (int16_t)(rx[18] << 8)
                  | rx[17]);
   for(i = 0; i < 19; i++)
    printf("check rx[%d]: %x \n", i, rx[i]);
    
    temperature++; 
    humidity++;
    printf("Temp: %.2f\n", temperature);
     // Update temperature value in JSON object
        json_object_dotset_number(rootObject, "temperature", temperature);

        json_object_dotset_number(rootObject, "humidity", humidity);

        // Serialize JSON object to string
        char *serializedString = json_serialize_to_string(rootValue);

            //message_handle = IoTHubMessage_CreateFromByteArray((const unsigned char*)data, sizeof(data));
            //message_handle = IoTHubMessage_CreateFromByteArray((unsigned char*)rx, 7);
			message_handle = IoTHubMessage_CreateFromString(serializedString);
			
			// Set Message property
                
                (void)IoTHubMessage_SetMessageId(message_handle, "MSG_ID");
                (void)IoTHubMessage_SetCorrelationId(message_handle, "CORE_ID");
                (void)IoTHubMessage_SetContentTypeSystemProperty(message_handle, "application%2fjson");
                (void)IoTHubMessage_SetContentEncodingSystemProperty(message_handle, "utf-8");
                //(void)IoTHubMessage_SetMessageCreationTimeUtcSystemProperty(message_handle, "2020-07-01T01:00:00.346Z");
                
            // Gửi tin nhắn

            (void)IoTHubMessage_SetProperty(message_handle, "property_key", "property_value");

            IoTHubDeviceClient_LL_SendEventAsync(device_ll_handle, message_handle, send_confirm_callback, NULL);

            printf("Đã gửi tin nhắn thành công\n");

            // Hủy tin nhắn
            IoTHubMessage_Destroy(message_handle);

            IoTHubDeviceClient_LL_DoWork(device_ll_handle);
            ThreadAPI_Sleep(1);

            // Dừng thực thi trong 2 giây
            //sleep(4);
        }

         // Clean up resources
    json_value_free(rootValue);

        // Clean up the iothub sdk handle
        IoTHubDeviceClient_LL_Destroy(device_ll_handle);
    }
    // Free all the sdk subsystem
    IoTHub_Deinit();



	//Test Arduino's Master - Slave
/*
*(unsigned int*)(i2c_slave + 2*4) = 0xFF;
printf("rx: 0x%d\n", *(unsigned int*)(i2c_slave + 2*4));
	while (1)
	{
    /*
		if (*(unsigned int*)(i2c_slave + 2*4) != t)
		{
			t = *(unsigned int*)(i2c_slave + 2*4);
			printf("Slave at reg 2 changes 0x%x\n", t);
		}
    */
   /*
   printf("Slave at reg 2 changes\n");
    for (j = 0; j < 7; ){
      if (*(unsigned int*)(i2c_slave + 2*4) != 0xFF)
		{
			rx[j] = *(unsigned int*)(i2c_slave + 2*4);
			printf("rx[%d]: 0x%x\n", j, rx[j]);
      *(unsigned int*)(i2c_slave + 2*4) = 0xFF;
      j++;
		}
	}
    printf("Hr: %d, SpO2: %.2f, Temp: %.2f\n", rx[0], 
        (float) ((rx[1] << 24 | rx[2] << 16 | rx[3] << 8 | rx[4]) / 100.0),
        (float) ((rx[5] << 8 | rx[6]) / 100.0));
			
		}
*/



  /*
	for (i = 0; i < 256; i++) {
		t = 255 - i;
		printf("Slave write == %d\n", t);
		*(unsigned int*)(i2c_slave + i*4) = t;
	}
  */

	//Test Master - Slave
	/*
	//Master write -> Master read and Slave read -> compare
	for (i = 0; i < 256; i++) {
		printf("Master write == 0x%x\n", i);
		oc_i2c_master_write(ptr,I2C_SLAVE_ADDR,i,i);
	}
	for (i = 0; i < 256; i++) {
		t = oc_i2c_master_read(ptr,I2C_SLAVE_ADDR,i);
		printf("Master read === 0x%x\t", t);
		if (t == *(unsigned int*)(i2c_slave + i*4))
			printf("Slave read: True\n");
		else
			printf("Slave read: False\n");
	}
	*/
	/*
	//Slave write -> Master read and Slave read -> Compare
	for (i = 0; i < 256; i++) {
		t = 255 - i;
		printf("Slave write == %d\n", t);
		*(unsigned int*)(i2c_slave + i*4) = t;
	}

	for (i = 0; i < 256; i++) {
		t = oc_i2c_master_read(ptr,I2C_SLAVE_ADDR,i);
		printf("Master read === %d\t", t);
		if (t == *(unsigned int*)(i2c_slave + i*4))
			printf("Slave read: True\n");
		else
			printf("Slave read: False\n");
	}
	*/
	printf("FINISH!!!\n");
	//==================================================================================//
	// clean up our memory mapping and exit
	
	if( munmap( virtual_base, HW_REGS_SPAN ) != 0 ) {
		printf( "ERROR: munmap() failed...\n" );
		close( fd );
		return( 1 );
	}

	close( fd );

	return( 0 );
}

//===================================================================================
// scl prescale register lo
void IOWR_OC_I2C_MASTER_PRERLO(volatile void * base, unsigned char data)  {
	//IOWR(base, 0, data);
	*(unsigned int*)(base + 0*4) = data;
}
unsigned char IORD_OC_I2C_MASTER_PRERLO(volatile void * base) {
    //return IORD(base, 0);
	return *(unsigned int*)(base + 0*4);
}

// scl prescale register hi
void IOWR_OC_I2C_MASTER_PRERHI(volatile void * base, unsigned char data)  {
	//IOWR(base, 1, data);
	*(unsigned int*)(base + 1*4) = data;
}
unsigned char IORD_OC_I2C_MASTER_PRERHI(volatile void * base) {
    //return IORD(base, 1);
	return *(unsigned int*)(base + 1*4);
}

// control register
void IOWR_OC_I2C_MASTER_CTR(volatile void * base, unsigned char data)  {
	//IOWR(base, 2, data);
	*(unsigned int*)(base + 2*4) = data;
}
unsigned char IORD_OC_I2C_MASTER_CTR(volatile void * base) {
    //return IORD(base, 2);
	return *(unsigned int*)(base + 2*4);
}

// tx and rx registers
void IOWR_OC_I2C_MASTER_TXR(volatile void * base, unsigned char data)  {
	//IOWR(base, 3, data);
	*(unsigned int*)(base + 3*4) = data;
}
unsigned char IORD_OC_I2C_MASTER_RXR(volatile void * base) {
    //return IORD(base, 3);
	return *(unsigned int*)(base + 3*4);
}
// command and status register
void IOWR_OC_I2C_MASTER_CR(volatile void * base, unsigned char data)  {
	//IOWR(base, 4, data);
	*(unsigned int*)(base + 4*4) = data;
}
unsigned char IORD_OC_I2C_MASTER_SR(volatile void * base) {
    //return IORD(base, 4);
	return *(unsigned int*)(base + 4*4);
}
//===================================================================================
void oc_i2c_master_init(int base, int freq)
{
    // Setup prescaler for a 100KHz I2C clock based on the frequency of the oc_i2c_master clock

    int prescale = ((freq) / (5*100000)) - 1;                   // calculate the prescaler value

    IOWR_OC_I2C_MASTER_CTR(&base, 0x00);                         // disable the I2C core

    IOWR_OC_I2C_MASTER_PRERLO(&base, prescale & 0xff);           // write the lo prescaler register
    IOWR_OC_I2C_MASTER_PRERHI(&base, (prescale & 0xff00)>>8);    // write the hi prescaler register

    IOWR_OC_I2C_MASTER_CTR(&base, OC_I2C_MASTER_CTR_CORE_EN);    // enable the I2C core
}
//===================================================================================
void oc_i2c_master_write(oc_i2c_master_dev *i2c_dev, unsigned char address, unsigned char data)
{
  unsigned char temp;

  do
  {
    temp = IORD_OC_I2C_MASTER_SR(i2c_dev->base);
  } while((temp & OC_I2C_MASTER_SR_TIP));
  printf("TIP\n");
  while(1)
  {
    temp = IORD_OC_I2C_MASTER_SR(i2c_dev->base);
    while(temp & OC_I2C_MASTER_SR_BUSY)
    {
      i2c_dev->busy_on_entry++;

      temp = IORD_OC_I2C_MASTER_SR(i2c_dev->base);
      if(temp & OC_I2C_MASTER_SR_BUSY)
      {
        IOWR_OC_I2C_MASTER_CR(i2c_dev->base, OC_I2C_MASTER_CR_STO | OC_I2C_MASTER_SR_IF);

        do
        {
          temp = IORD_OC_I2C_MASTER_SR(i2c_dev->base);
        } while((temp & OC_I2C_MASTER_SR_TIP) || (!(temp & OC_I2C_MASTER_SR_IF)));
      }
    }
    printf("Busy\n");
    // write address
    IOWR_OC_I2C_MASTER_TXR(i2c_dev->base, address<<1);
    IOWR_OC_I2C_MASTER_CR(i2c_dev->base, OC_I2C_MASTER_CR_STA | OC_I2C_MASTER_CR_WR | OC_I2C_MASTER_SR_IF);
    printf("Send Addr\n");
    do
    {
      temp = IORD_OC_I2C_MASTER_SR(i2c_dev->base);
    } while((temp & OC_I2C_MASTER_SR_TIP) || (!(temp & OC_I2C_MASTER_SR_IF)));
    if((temp & OC_I2C_MASTER_SR_RxACK) || (temp & OC_I2C_MASTER_SR_AL))
    {
      i2c_dev->bad_cycle_term++;
      continue;
    }
    printf("ADDR\n");
    // write data
    IOWR_OC_I2C_MASTER_TXR(i2c_dev->base, data);
    IOWR_OC_I2C_MASTER_CR(i2c_dev->base, OC_I2C_MASTER_CR_STO | OC_I2C_MASTER_CR_WR | OC_I2C_MASTER_SR_IF);
    do
    {
      temp = IORD_OC_I2C_MASTER_SR(i2c_dev->base);
    } while((temp & OC_I2C_MASTER_SR_TIP) || (!(temp & OC_I2C_MASTER_SR_IF)));
    if((temp & OC_I2C_MASTER_SR_RxACK) || (temp & OC_I2C_MASTER_SR_AL))
    {
      i2c_dev->bad_cycle_term++;
      continue;
    }
    printf("DATA\n");
    break;
  }
}

void oc_i2c_master_write_reg(oc_i2c_master_dev *i2c_dev, unsigned char address, unsigned char reg, unsigned char data)
{
  unsigned char temp;

  do
  {
    temp = IORD_OC_I2C_MASTER_SR(i2c_dev->base);
  } while((temp & OC_I2C_MASTER_SR_TIP));
  printf("TIP\n");
  while(1)
  {
    temp = IORD_OC_I2C_MASTER_SR(i2c_dev->base);
    while(temp & OC_I2C_MASTER_SR_BUSY)
    {
      i2c_dev->busy_on_entry++;

      temp = IORD_OC_I2C_MASTER_SR(i2c_dev->base);
      if(temp & OC_I2C_MASTER_SR_BUSY)
      {
        IOWR_OC_I2C_MASTER_CR(i2c_dev->base, OC_I2C_MASTER_CR_STO | OC_I2C_MASTER_SR_IF);

        do
        {
          temp = IORD_OC_I2C_MASTER_SR(i2c_dev->base);
        } while((temp & OC_I2C_MASTER_SR_TIP) || (!(temp & OC_I2C_MASTER_SR_IF)));
      }
    }
    printf("Busy\n");
    // write address
    IOWR_OC_I2C_MASTER_TXR(i2c_dev->base, address<<1);
    IOWR_OC_I2C_MASTER_CR(i2c_dev->base, OC_I2C_MASTER_CR_STA | OC_I2C_MASTER_CR_WR | OC_I2C_MASTER_SR_IF);
    do
    {
      temp = IORD_OC_I2C_MASTER_SR(i2c_dev->base);
    } while((temp & OC_I2C_MASTER_SR_TIP) || (!(temp & OC_I2C_MASTER_SR_IF)));
    if((temp & OC_I2C_MASTER_SR_RxACK) || (temp & OC_I2C_MASTER_SR_AL))
    {
      i2c_dev->bad_cycle_term++;
      continue;
    }
    printf("ADDR\n");
    // write register address
    IOWR_OC_I2C_MASTER_TXR(i2c_dev->base, reg);
    IOWR_OC_I2C_MASTER_CR(i2c_dev->base, OC_I2C_MASTER_CR_WR | OC_I2C_MASTER_SR_IF);
    do
    {
      temp = IORD_OC_I2C_MASTER_SR(i2c_dev->base);
    } while((temp & OC_I2C_MASTER_SR_TIP) || (!(temp & OC_I2C_MASTER_SR_IF)));
    if((temp & OC_I2C_MASTER_SR_RxACK) || (temp & OC_I2C_MASTER_SR_AL))
    {
      i2c_dev->bad_cycle_term++;
      continue;
    }
    printf("REG\n");
    // write data
    IOWR_OC_I2C_MASTER_TXR(i2c_dev->base, data);
    IOWR_OC_I2C_MASTER_CR(i2c_dev->base, OC_I2C_MASTER_CR_STO | OC_I2C_MASTER_CR_WR | OC_I2C_MASTER_SR_IF);
    do
    {
      temp = IORD_OC_I2C_MASTER_SR(i2c_dev->base);
    } while((temp & OC_I2C_MASTER_SR_TIP) || (!(temp & OC_I2C_MASTER_SR_IF)));
    if((temp & OC_I2C_MASTER_SR_RxACK) || (temp & OC_I2C_MASTER_SR_AL))
    {
      i2c_dev->bad_cycle_term++;
      continue;
    }
    printf("DATA\n");
    break;
  }
}
//===================================================================================
unsigned char oc_i2c_master_read(oc_i2c_master_dev *i2c_dev, unsigned char address)
{
  unsigned char temp;

  do
  {
    temp = IORD_OC_I2C_MASTER_SR(i2c_dev->base);
  } while((temp & OC_I2C_MASTER_SR_TIP));

  while(1)
  {
    temp = IORD_OC_I2C_MASTER_SR(i2c_dev->base);
    while(temp & OC_I2C_MASTER_SR_BUSY)
    {
      i2c_dev->busy_on_entry++;

      temp = IORD_OC_I2C_MASTER_SR(i2c_dev->base);
      if(temp & OC_I2C_MASTER_SR_BUSY)
      {
        IOWR_OC_I2C_MASTER_CR(i2c_dev->base, OC_I2C_MASTER_CR_STO | OC_I2C_MASTER_SR_IF);

        do
        {
          temp = IORD_OC_I2C_MASTER_SR(i2c_dev->base);
        } while((temp & OC_I2C_MASTER_SR_TIP) || (!(temp & OC_I2C_MASTER_SR_IF)));
      }
    }

    // write address for reading
    IOWR_OC_I2C_MASTER_TXR(i2c_dev->base, (address<<1) | 1);
    IOWR_OC_I2C_MASTER_CR(i2c_dev->base, OC_I2C_MASTER_CR_STA | OC_I2C_MASTER_CR_WR | OC_I2C_MASTER_SR_IF);
    do
    {
      temp = IORD_OC_I2C_MASTER_SR(i2c_dev->base);
    } while((temp & OC_I2C_MASTER_SR_TIP) || (!(temp & OC_I2C_MASTER_SR_IF)));
    if((temp & OC_I2C_MASTER_SR_RxACK) || (temp & OC_I2C_MASTER_SR_AL))
    {
      i2c_dev->bad_cycle_term++;
      continue;
    }

    // read data
    IOWR_OC_I2C_MASTER_CR(i2c_dev->base, OC_I2C_MASTER_CR_RD | OC_I2C_MASTER_CR_ACK | OC_I2C_MASTER_CR_STO | OC_I2C_MASTER_SR_IF);
    do
    {
      temp = IORD_OC_I2C_MASTER_SR(i2c_dev->base);
    } while((temp & OC_I2C_MASTER_SR_TIP) || (!(temp & OC_I2C_MASTER_SR_IF)));
    if(!(temp & OC_I2C_MASTER_SR_RxACK) || (temp & OC_I2C_MASTER_SR_AL))
    {
      i2c_dev->bad_cycle_term++;
      continue;
    }

    break;
  }

  return IORD_OC_I2C_MASTER_RXR(i2c_dev->base);
}

unsigned char oc_i2c_master_read_reg(oc_i2c_master_dev *i2c_dev, unsigned char address, unsigned char reg)
{
  unsigned char temp;

  do
  {
    temp = IORD_OC_I2C_MASTER_SR(i2c_dev->base);
  } while((temp & OC_I2C_MASTER_SR_TIP));

  while(1)
  {
    temp = IORD_OC_I2C_MASTER_SR(i2c_dev->base);
    while(temp & OC_I2C_MASTER_SR_BUSY)
    {
      i2c_dev->busy_on_entry++;

      temp = IORD_OC_I2C_MASTER_SR(i2c_dev->base);
      if(temp & OC_I2C_MASTER_SR_BUSY)
      {
        IOWR_OC_I2C_MASTER_CR(i2c_dev->base, OC_I2C_MASTER_CR_STO | OC_I2C_MASTER_SR_IF);

        do
        {
          temp = IORD_OC_I2C_MASTER_SR(i2c_dev->base);
        } while((temp & OC_I2C_MASTER_SR_TIP) || (!(temp & OC_I2C_MASTER_SR_IF)));
      }
    }

    // write address
    IOWR_OC_I2C_MASTER_TXR(i2c_dev->base, address<<1);
    IOWR_OC_I2C_MASTER_CR(i2c_dev->base, OC_I2C_MASTER_CR_STA | OC_I2C_MASTER_CR_WR | OC_I2C_MASTER_SR_IF);
    do
    {
      temp = IORD_OC_I2C_MASTER_SR(i2c_dev->base);
    } while((temp & OC_I2C_MASTER_SR_TIP) || (!(temp & OC_I2C_MASTER_SR_IF)));
    if((temp & OC_I2C_MASTER_SR_RxACK) || (temp & OC_I2C_MASTER_SR_AL))
    {
      i2c_dev->bad_cycle_term++;
      continue;
    }

    // write register address
    IOWR_OC_I2C_MASTER_TXR(i2c_dev->base, reg);
    IOWR_OC_I2C_MASTER_CR(i2c_dev->base, OC_I2C_MASTER_CR_WR |OC_I2C_MASTER_SR_IF);
    do
    {
      temp = IORD_OC_I2C_MASTER_SR(i2c_dev->base);
    } while((temp & OC_I2C_MASTER_SR_TIP) || (!(temp & OC_I2C_MASTER_SR_IF)));
    if((temp & OC_I2C_MASTER_SR_RxACK) || (temp & OC_I2C_MASTER_SR_AL))
    {
      i2c_dev->bad_cycle_term++;
      continue;
    }

    // write address for reading
    IOWR_OC_I2C_MASTER_TXR(i2c_dev->base, (address<<1) | 1);
    IOWR_OC_I2C_MASTER_CR(i2c_dev->base, OC_I2C_MASTER_CR_STA | OC_I2C_MASTER_CR_WR | OC_I2C_MASTER_SR_IF);
    do
    {
      temp = IORD_OC_I2C_MASTER_SR(i2c_dev->base);
    } while((temp & OC_I2C_MASTER_SR_TIP) || (!(temp & OC_I2C_MASTER_SR_IF)));
    if((temp & OC_I2C_MASTER_SR_RxACK) || (temp & OC_I2C_MASTER_SR_AL))
    {
      i2c_dev->bad_cycle_term++;
      continue;
    }

    // read data
    IOWR_OC_I2C_MASTER_CR(i2c_dev->base, OC_I2C_MASTER_CR_RD | OC_I2C_MASTER_CR_ACK | OC_I2C_MASTER_CR_STO | OC_I2C_MASTER_SR_IF);
    do
    {
      temp = IORD_OC_I2C_MASTER_SR(i2c_dev->base);
    } while((temp & OC_I2C_MASTER_SR_TIP) || (!(temp & OC_I2C_MASTER_SR_IF)));
    if(!(temp & OC_I2C_MASTER_SR_RxACK) || (temp & OC_I2C_MASTER_SR_AL))
    {
      i2c_dev->bad_cycle_term++;
      continue;
    }

    break;
  }

  return IORD_OC_I2C_MASTER_RXR(i2c_dev->base);
}
