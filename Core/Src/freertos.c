/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include "usart.h"
#include <std_msgs/msg/int32.h>
#include <firmware_msgs/msg/mcu_pose.h>

#include <geometry_msgs/msg/twist.h>
#include "odometry.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NODE_NAME "firmware_node"

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
rclc_executor_t executor;
rcl_publisher_t publisher;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_subscription_t subscriber;

firmware_msgs__msg__McuPose odom_msg;
geometry_msgs__msg__Twist   vel_msg;

//osSemaphoreId_t OdomSemHandle;
//const osSemaphoreAttr_t OdomSem_attributes = {
//  .name = "OdomSem"
//};

osMutexId_t odom_mutex;

const osMutexAttr_t Thread_Mutex_attr = {
  "OdomMutex",                          // human readable mutex name
  osMutexRecursive | osMutexPrioInherit,    // attr_bits
  NULL,                                     // memory for control block
  0U                                        // size for control block
};

/* USER CODE END Variables */
/* Definitions for MCU_POSE */
osThreadId_t MCU_POSEHandle;
uint32_t defaultTaskBuffer[ 3000 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t MCU_POSE_attributes = {
  .name = "MCU_POSE",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityHigh7,
};
/* Definitions for MCU_VEL */
osThreadId_t MCU_VELHandle;
uint32_t MCU_VELBuffer[ 3000 ];
osStaticThreadDef_t MCU_VELControlBlock;
const osThreadAttr_t MCU_VEL_attributes = {
  .name = "MCU_VEL",
  .cb_mem = &MCU_VELControlBlock,
  .cb_size = sizeof(MCU_VELControlBlock),
  .stack_mem = &MCU_VELBuffer[0],
  .stack_size = sizeof(MCU_VELBuffer),
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for UPDATE_IMU_VEL */
osThreadId_t UPDATE_IMU_VELHandle;
uint32_t UPDATE_IMU_VELBuffer[ 256 ];
osStaticThreadDef_t UPDATE_IMU_VELControlBlock;
const osThreadAttr_t UPDATE_IMU_VEL_attributes = {
  .name = "UPDATE_IMU_VEL",
  .cb_mem = &UPDATE_IMU_VELControlBlock,
  .cb_size = sizeof(UPDATE_IMU_VELControlBlock),
  .stack_mem = &UPDATE_IMU_VELBuffer[0],
  .stack_size = sizeof(UPDATE_IMU_VELBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

void vel_setter_callback(const void *msgin) {
	const geometry_msgs__msg__Twist *cmdVel = (const geometry_msgs__msg__Twist *) msgin;

	update_vel_wheels(cmdVel);
}
void setup_node(void);
void destroy_node(void);
/* USER CODE END FunctionPrototypes */

void odom_getter(void *argument);
void vel_setter(void *argument);
void update_imu_vel(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
//	odom_mutex = osMutexNew(&Thread_Mutex_attr);

//	 OdomSemHandle = osSemaphoreNew(2, 2, &OdomSem_attributes);

	odom_mutex = osMutexNew(&Thread_Mutex_attr);
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of MCU_POSE */
  MCU_POSEHandle = osThreadNew(odom_getter, NULL, &MCU_POSE_attributes);
  /* creation of MCU_VEL */
  MCU_VELHandle = osThreadNew(vel_setter, NULL, &MCU_VEL_attributes);
  /* creation of UPDATE_IMU_VEL */
  UPDATE_IMU_VELHandle = osThreadNew(update_imu_vel, NULL, &UPDATE_IMU_VEL_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_odom_getter */
/**
* @brief Function implementing the MCU_POSE thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_odom_getter */
void odom_getter(void *argument)
{

  /* USER CODE BEGIN odom_getter */
  /* Infinite loop */
  for(;;)
  {
	if (osMutexAcquire(odom_mutex, osWaitForever) == osOK) {
		get_odom_msg(&odom_msg);
		rcl_publish(&publisher, &odom_msg, NULL);
		osMutexRelease(odom_mutex);
	}
    osDelay(20);
  }
  /* USER CODE END odom_getter */
}

/* USER CODE BEGIN Header_vel_setter */
/**
  * @brief  Function implementing the MCU_VEL thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_vel_setter */
void vel_setter(void *argument)
{
	setup_node();
  /* USER CODE BEGIN vel_setter */

  /* Infinite loop */
  for(;;)
  {
	rclc_executor_spin(&executor);
    osDelay(10);
  }
  /* USER CODE END vel_setter */
}

/* USER CODE BEGIN Header_update_imu_vel */
/**
* @brief Function implementing the UPDATE_IMU_VEL thread.
* @param argument: Not used
* @retval None;
*/
/* USER CODE END Header_update_imu_vel */
void update_imu_vel(void *argument)
{
  /* USER CODE BEGIN update_imu_vel */
  /* Infinite loop */
  for(;;)
  {
//	if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15) == 0) {
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
//	} else {
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
//	}
//	set_vel_wheels();
	update_yaw();

	if (osMutexAcquire(odom_mutex, osWaitForever) == osOK) {
		update_step();
		osMutexRelease(odom_mutex);
	}
  }
  /* USER CODE END update_imu_vel */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void setup_node() {
	  rmw_uros_set_custom_transport(
	    true,
	    (void *) &huart6,
	    cubemx_transport_open,
	    cubemx_transport_close,
	    cubemx_transport_write,
	    cubemx_transport_read);

	  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
	  freeRTOS_allocator.allocate = microros_allocate;
	  freeRTOS_allocator.deallocate = microros_deallocate;
	  freeRTOS_allocator.reallocate = microros_reallocate;
	  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

	  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
	      printf("Error on default allocators (line %d)\n", __LINE__);
	  }

	  // micro-ROS app
	  allocator = rcl_get_default_allocator();

	  //create init_options
	  rclc_support_init(&support, 0, NULL, &allocator);

	  // create node
	  rclc_node_init_default(&node, NODE_NAME, "", &support);

	  // create publisher
	  rclc_publisher_init_best_effort(
	    &publisher,
	    &node,
	    ROSIDL_GET_MSG_TYPE_SUPPORT(firmware_msgs, msg, McuPose),
	    "mcu_pose");

	  rclc_subscription_init_best_effort(
	  		&subscriber,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
	  		"mcu_vel"
		);

	  rclc_executor_init(&executor, &support.context, 1, &allocator);
	  rclc_executor_add_subscription(
			  &executor,
			  &subscriber,
			  &vel_msg,
	    	  &vel_setter_callback,
			  ON_NEW_DATA);

}

void destroy_node()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&publisher, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void error_loop() {
  while (1) {

  }
}

/* USER CODE END Application */

