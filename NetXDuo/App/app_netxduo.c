/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_netxduo.c
  * @author  MCD Application Team
  * @brief   NetXDuo applicative file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_netxduo.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include   "nx_stm32_eth_config.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* Define the ThreadX , NetX and FileX object control blocks. */

/* Define Threadx global data structures. */
TX_THREAD AppMainThread;
TX_THREAD AppServerThread;
TX_THREAD AppLinkThread;

void LedThread_Entry(ULONG thread_input);

TX_SEMAPHORE Semaphore;
TX_QUEUE  MsgQueueOne;

/* Define NetX global data structures. */

NX_PACKET_POOL EthPool;
NX_PACKET_POOL WebServerPool;

NX_IP  IpInstance;
NX_DHCP DHCPClient;

ULONG IpAddress;
ULONG NetMask;
ULONG free_bytes;

/* App memory pointer. */
CHAR   *pointer;

NX_WEB_HTTP_SERVER HTTPServer;

/* Set nx_server_pool start address to 0x20060100 */
#if defined ( __ICCARM__ ) /* IAR Compiler */
#pragma location = 0x20060200
#elif defined ( __CC_ARM ) /* MDK ARM Compiler */
__attribute__((section(".NxServerPoolSection")))
#elif defined ( __GNUC__ ) /* GNU Compiler */
__attribute__((section(".NxServerPoolSection")))
#endif
static uint8_t nx_server_pool[SERVER_POOL_SIZE];

/* Define FileX global data structures. */

/* the server reads the content from the uSD, a FX_MEDIA instance is required */
FX_MEDIA                SDMedia;

/* Buffer for FileX FX_MEDIA sector cache. this should be 32-Bytes aligned to avoid
   cache maintenance issues */
ALIGN_32BYTES (uint32_t DataBuffer[512]);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* HTTP server thread entry */
static void  App_Main_Thread_Entry(ULONG thread_input);
static void  nx_server_thread_entry(ULONG thread_input);
static VOID App_Link_Thread_Entry(ULONG thread_input);

/* DHCP state change notify callback */
static VOID ip_address_change_notify_callback(NX_IP *ip_instance, VOID *ptr);

/* Server callback when a new request from a client is triggered */
static UINT webserver_request_notify_callback(NX_WEB_HTTP_SERVER *server_ptr, UINT request_type, CHAR *resource, NX_PACKET *packet_ptr);
/* USER CODE END PFP */
/**
  * @brief  Application NetXDuo Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT MX_NetXDuo_Init(VOID *memory_ptr)
{
  UINT ret = NX_SUCCESS;
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;

  /* USER CODE BEGIN MX_NetXDuo_MEM_POOL */
  /* USER CODE END MX_NetXDuo_MEM_POOL */

  /* USER CODE BEGIN MX_NetXDuo_Init */

  printf("Nx_Webserver application started..\n");

  /* Initialize the NetXDuo system. */
  nx_system_initialize();

  /* Allocate the Ethernet packet pool. */
  ret = tx_byte_allocate(byte_pool, (VOID **) &pointer, NX_PACKET_POOL_SIZE, TX_NO_WAIT);

  /* Check ethernet packet pool memory allocation. */
  if (ret != NX_SUCCESS)
  {
    printf("Packed pool memory allocation failed : 0x%02x\n", ret);
    Error_Handler();
  }

  /* Create a packet pool. */
  ret = nx_packet_pool_create(&EthPool, "NetX Main Packet Pool", PACKET_PAYLOAD_SIZE, pointer, NX_PACKET_POOL_SIZE);

  /* Check for packet pool creation status. */
  if (ret != NX_SUCCESS)
  {
    printf("Packed creation failed : 0x%02x\n", ret);
    Error_Handler();
  }

  /* Allocate the server packet pool. */
  ret = tx_byte_allocate(byte_pool, (VOID **) &pointer, SERVER_POOL_SIZE, TX_NO_WAIT);

  /* Check server packet pool memory allocation. */
  if (ret != TX_SUCCESS)
  {
    printf("Packed pool memory allocation failed : 0x%02x\n", ret);
    Error_Handler();
  }

  /* Create the server packet pool. */
  ret = nx_packet_pool_create(&WebServerPool, "HTTP Server Packet Pool", SERVER_PACKET_SIZE, nx_server_pool, SERVER_POOL_SIZE);

  /* Check for server pool creation status. */
  if (ret != NX_SUCCESS)
  {
    printf("Server pool creation failed : 0x%02x\n", ret);
    Error_Handler();
  }

  /* Allocate the NetX IP Instance memory. */
  ret = tx_byte_allocate(byte_pool, (VOID **) &pointer, 2 * DEFAULT_MEMORY_SIZE, TX_NO_WAIT);

  /* Check NetX IP Instance memory allocation. */
  if (ret != TX_SUCCESS)
  {
    printf("NetX IP Instance memory allocation failed : 0x%02x\n", ret);
    Error_Handler();
  }

  /* Create an IP instance by linking the nx_driver_St32f7xx driver */
  ret = nx_ip_create(&IpInstance, "NetX IP Instance 0", NULL_IP_ADDRESS, NULL_IP_ADDRESS, &EthPool, nx_stm32_eth_driver,
                      pointer, 2 * DEFAULT_MEMORY_SIZE, DEFAULT_PRIORITY);

  if (ret != NX_SUCCESS)
  {
    printf("IP Instance creation failed : 0x%02x\n",ret);
    Error_Handler();
  }

  /* Allocate the ARP cache. */
  ret = tx_byte_allocate(byte_pool, (VOID **) &pointer, ARP_CACHE_SIZE, TX_NO_WAIT);

  /* Check ARP cache memory allocation. */
  if (ret != TX_SUCCESS)
  {
    printf("ARP cache memory allocation failed : 0x%02x\n", ret);
    Error_Handler();
  }

  /* Enable ARP and supply ARP cache memory for IP Instance 0. */
  ret =  nx_arp_enable(&IpInstance, (void *) pointer, ARP_CACHE_SIZE);

  if (ret != NX_SUCCESS)
  {
    printf("ARP Enable for IP error : 0x%02x\n", ret);
    Error_Handler();
  }

   /* Enable the ICMP support to be able to ping the board */
  ret = nx_icmp_enable(&IpInstance);

  if (ret != NX_SUCCESS)
  {
    printf("ICMP enable for IP error : 0x%02x\n", ret);
    Error_Handler();
  }

  /* Enable UDP support needed  by DHCP client */
  ret =  nx_udp_enable(&IpInstance);

  if (ret != NX_SUCCESS)
  {
    printf("UDP enable for IP error : 0x%02x\n", ret);
    Error_Handler();
  }

  /* Enable the TCP protocol */
  ret =  nx_tcp_enable(&IpInstance);

  if (ret!=NX_SUCCESS)
  {
    printf("TCP enable for IP error : 0x%02x\n", ret);
    Error_Handler();
  }

  /* Allocate the server stack. */
  ret = tx_byte_allocate(byte_pool, (VOID **) &pointer, SERVER_STACK, TX_NO_WAIT);

  /* Check server stack memory allocation. */
  if (ret != TX_SUCCESS)
  {
    printf("Server stack memory allocation failed : 0x%02x\n", ret);
    Error_Handler();
  }

  /* Create the HTTP Server. */
 ret = nx_web_http_server_create(&HTTPServer, "WEB HTTP Server", &IpInstance, CONNECTION_PORT,&SDMedia, pointer,
                                  SERVER_STACK, &WebServerPool, NX_NULL, webserver_request_notify_callback);

  if (ret != NX_SUCCESS)
  {
     printf("HTTP Server creation failed: 0x%02x\n", ret);
     Error_Handler();
  }

  /* Allocate the main thread. */
  ret = tx_byte_allocate(byte_pool, (VOID **) &pointer, 2 * DEFAULT_MEMORY_SIZE, TX_NO_WAIT);

  /* Check main thread memory allocation. */
  if (ret != TX_SUCCESS)
  {
    printf("Main thread memory allocation failed : 0x%02x\n", ret);
    Error_Handler();
  }

  /* Create the main thread */
  ret = tx_thread_create(&AppMainThread, "App Main thread", App_Main_Thread_Entry, 0, pointer, 2 * DEFAULT_MEMORY_SIZE,
                         DEFAULT_MAIN_PRIORITY, DEFAULT_MAIN_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START);

  if (ret != TX_SUCCESS)
  {
    return NX_NOT_ENABLED;
  }

  /* Allocate the TCP server thread stack. */
  ret = tx_byte_allocate(byte_pool, (VOID **) &pointer, 2 * DEFAULT_MEMORY_SIZE, TX_NO_WAIT);

  /* Check server thread memory allocation. */
  if (ret != TX_SUCCESS)
  {
    printf("Server thread memory allocation failed : 0x%02x\n", ret);
    Error_Handler();
  }

  /* create the web server thread */
  ret = tx_thread_create(&AppServerThread, "App Server Thread", nx_server_thread_entry, 0, pointer, 2 * DEFAULT_MEMORY_SIZE,
                         DEFAULT_PRIORITY, DEFAULT_PRIORITY, TX_NO_TIME_SLICE, TX_DONT_START);

  if (ret != TX_SUCCESS)
  {
    return NX_NOT_ENABLED;
  }

  /* Allocate the memory for Link thread   */
  if (tx_byte_allocate(byte_pool, (VOID **) &pointer,2 *  DEFAULT_MEMORY_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }

  /* create the Link thread */
  ret = tx_thread_create(&AppLinkThread, "App Link Thread", App_Link_Thread_Entry, 0, pointer, 2 * DEFAULT_MEMORY_SIZE,
                         LINK_PRIORITY, LINK_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START);

  if (ret != TX_SUCCESS)
  {
    return NX_NOT_ENABLED;
  }

  /* Create the DHCP instance. */
  ret = nx_dhcp_create(&DHCPClient, &IpInstance, "dhcp_client");
  if (ret != NX_SUCCESS)
  {
    printf("DHCP Instance creation failed : 0x%02x\n", ret);
  }

  ret = nx_ip_address_change_notify(&IpInstance, ip_address_change_notify_callback, NULL);
  if (ret != NX_SUCCESS)
  {
    Error_Handler();
  }

  ret = tx_semaphore_create(&Semaphore, "App Semaphore", 0);
  if (ret != TX_SUCCESS)
  {
    printf("Semaphore creation failed : 0x%02x\n", ret);
    Error_Handler();
  }

  /* USER CODE END MX_NetXDuo_Init */

  return ret;
}

/* USER CODE BEGIN 1 */
/**
* @brief  ip address change callback
* @param  ip_instance : NX_IP instance registered for this callback
* @param   ptr : VOID * optional data pointer
* @retval None
*/
static VOID ip_address_change_notify_callback(NX_IP *ip_instance, VOID *ptr)
{
  /* as soon as the IP address is ready, the semaphore is released to let the server start */
  tx_semaphore_put(&Semaphore);
}


static VOID App_Main_Thread_Entry(ULONG thread_input)
{
  UINT ret;

  ret = nx_ip_address_change_notify(&IpInstance, ip_address_change_notify_callback, NULL);
  if (ret != NX_SUCCESS)
  {
    Error_Handler();
  }

  ret = nx_dhcp_start(&DHCPClient);
  if (ret != NX_SUCCESS)
  {
    Error_Handler();
  }

  /* wait until an IP address is ready */
  if(tx_semaphore_get(&Semaphore, TX_WAIT_FOREVER) != TX_SUCCESS)
  {
    Error_Handler();
  }
  /* get IP address */
  ret = nx_ip_address_get(&IpInstance, &IpAddress, &NetMask);

  PRINT_IP_ADDRESS(IpAddress);

  if (ret != TX_SUCCESS)
  {
    Error_Handler();
  }

  /* the network is correctly initialized, start the WEB server thread */
  tx_thread_resume(&AppServerThread);

  /* this thread is not needed any more, we relinquish it */
  tx_thread_relinquish();

  return;
}

UINT webserver_request_notify_callback(NX_WEB_HTTP_SERVER *server_ptr, UINT request_type, CHAR *resource, NX_PACKET *packet_ptr)
{

  CHAR temp_string[30] = {'\0'};
  CHAR data[512] = {'\0'};
  UINT string_length;
  NX_PACKET *resp_packet_ptr;
  UINT status;
  ULONG total_bytes_sent;
  ULONG total_bytes_received;
  ULONG connections;
  ULONG disconnections;
  ULONG main_thread_count;
  ULONG server_thread_count;
  CHAR *main_thread_name;
  CHAR *server_thread_name;

  /* Get the requested data from packet */
  if (strcmp(resource, "/GetNXData") == 0)
  {
    nx_tcp_info_get(&IpInstance, NULL, &total_bytes_sent, NULL, &total_bytes_received, NULL,  NULL, NULL, &connections, &disconnections, NULL, NULL);
    sprintf (data, "%lu,%lu,%lu,%lu",total_bytes_received, total_bytes_sent, connections, disconnections);
  }
  else if (strcmp(resource, "/GetNetInfo") == 0)
  {
   sprintf(data, "%lu.%lu.%lu.%lu,%d", (IpAddress >> 24) & 0xff, (IpAddress >> 16) & 0xff, (IpAddress >> 8) & 0xff, IpAddress& 0xff, CONNECTION_PORT);
  }
  else if (strcmp(resource, "/GetTxCount") == 0)
  {
    tx_thread_info_get(&AppMainThread, &main_thread_name, NULL, &main_thread_count, NULL, NULL, NULL, NULL, NULL);
    tx_thread_info_get(&AppServerThread, &server_thread_name, NULL, &server_thread_count, NULL, NULL, NULL, NULL, NULL);
    sprintf (data, "%s,%lu ,%s,%lu", main_thread_name, main_thread_count, server_thread_name, server_thread_count);

  }
    else if (strcmp(resource, "/GetNXPacket") == 0)
  {
    sprintf (data, "%lu", EthPool.nx_packet_pool_available);
  }
    else if (strcmp(resource, "/GetNXPacketlen") == 0)
  {
    sprintf (data, "%lu", (EthPool.nx_packet_pool_available_list)->nx_packet_length );
  }
  else
  {
    return NX_SUCCESS;
  }
  /* Derive the client request type from the client request. */
  nx_web_http_server_type_get(server_ptr, server_ptr -> nx_web_http_server_request_resource, temp_string, &string_length);

  /* Null terminate the string. */
  temp_string[string_length] = '\0';

  /* Now build a response header with server status is OK and no additional header info. */
  status = nx_web_http_server_callback_generate_response_header(server_ptr, &resp_packet_ptr, NX_WEB_HTTP_STATUS_OK,
                                                                strlen(data), temp_string, NX_NULL);

  status = _nxe_packet_data_append(resp_packet_ptr, data, strlen(data), server_ptr->nx_web_http_server_packet_pool_ptr, NX_WAIT_FOREVER);
  /* Now send the packet! */

  status = nx_web_http_server_callback_packet_send(server_ptr, resp_packet_ptr);
  if (status != NX_SUCCESS)
  {
    nx_packet_release(resp_packet_ptr);
    return status;
  }
  return(NX_WEB_HTTP_CALLBACK_COMPLETED);
}

/**
* @brief  Application thread for HTTP web server
* @param  thread_input : thread input
* @retval None
*/

static NX_WEB_HTTP_SERVER_MIME_MAP my_mime_maps[] =
{
  {"css", "text/css"},
  {"svg", "image/svg+xml"},
  {"png", "image/png"},
  {"jpg", "image/jpg"}
};

void nx_server_thread_entry(ULONG thread_input)
{
  /* HTTP WEB SERVER THREAD Entry */
  UINT    status;
  NX_PARAMETER_NOT_USED(thread_input);

  /* Open the SD disk driver. */
  status = fx_media_open(&SDMedia, "STM32_SDIO_DISK", fx_stm32_sd_driver, SD_DRIVER_INFO_POINTER, DataBuffer, sizeof(DataBuffer));

  /* Check the media opening status. */
  if (status != FX_SUCCESS)
  {
    /*Print Media Opening error. */
    printf("FX media opening failed : 0x%02x\n", status);
    /* Error, call error handler.*/
    Error_Handler();
  }
  else
  {
    /* Print Media Opening Success. */
    printf("Fx media successfully opened.\n");

    fx_media_space_available(&SDMedia, &free_bytes);
  }

  status = nx_web_http_server_mime_maps_additional_set(&HTTPServer,&my_mime_maps[0], 4);

  /* Start the WEB HTTP Server. */
  status = nx_web_http_server_start(&HTTPServer);

  /* Check the WEB HTTP Server starting status. */
  if (status != NX_SUCCESS)
  {
    /* Print HTTP WEB Server starting error. */
    printf("HTTP WEB Server Starting Failed, error: 0x%02x\n", status);
    /* Error, call error handler.*/
    Error_Handler();
  }
  else
  {
    /* Print HTTP WEB Server Starting success. */
    printf("HTTP WEB Server successfully started.\n");
  }
}



/**
* @brief  Link thread entry
* @param thread_input: ULONG thread parameter
* @retval none
*/
static VOID App_Link_Thread_Entry(ULONG thread_input)
{
  ULONG actual_status;
  UINT linkdown = 0, status;

  while(1)
  {
    /* Get Physical Link stackavailtus. */
    status = nx_ip_interface_status_check(&IpInstance, 0, NX_IP_LINK_ENABLED,
                                      &actual_status, 10);

    if(status == NX_SUCCESS)
    {
      if(linkdown == 1)
      {
        linkdown = 0;
        status = nx_ip_interface_status_check(&IpInstance, 0, NX_IP_ADDRESS_RESOLVED,
                                      &actual_status, 10);
        if(status == NX_SUCCESS)
        {
          /* The network cable is connected again. */
          printf("The network cable is connected again.\n");
          /* Print Webserver Client is available again. */
          printf("Webserver Client is available again.\n");
        }
        else
        {
          /* The network cable is connected. */
          printf("The network cable is connected.\n");
          /* Send command to Enable Nx driver. */
          nx_ip_driver_direct_command(&IpInstance, NX_LINK_ENABLE,
                                      &actual_status);
          /* Restart DHCP Client. */
          nx_dhcp_stop(&DHCPClient);
          nx_dhcp_start(&DHCPClient);
        }
      }
    }
    else
    {
      if(0 == linkdown)
      {
        linkdown = 1;
        /* The network cable is not connected. */
        printf("The network cable is not connected.\n");
      }
    }

    tx_thread_sleep(NX_ETH_CABLE_CONNECTION_CHECK_PERIOD);
  }
}

/* USER CODE END 1 */
