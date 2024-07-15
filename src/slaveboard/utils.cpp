#include <slaveboard/utils.h>


void checkResetCause() {
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
        Serial.println("System reset by Independent Watchdog Timer (IWDG).");
    }
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST)) {
        Serial.println("System reset by Window Watchdog Timer (WWDG).");
    }
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST)) {
        Serial.println("System reset by Power-on Reset.");
    }
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)) {
        Serial.println("System reset by Software Reset.");
    }
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)) {
        Serial.println("System reset by NRST pin.");
    }
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST)) {
        Serial.println("System reset by Low Power Reset.");
    }
    // if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST)) {
    //     Serial.println("System reset by Brown-out Reset (BOR).");
    // }
    // Clear all reset flags
    __HAL_RCC_CLEAR_RESET_FLAGS();
}

void monitorStackUsage(TaskHandle_t* xHandleRotating, TaskHandle_t* xReflectanceHandle, TaskHandle_t* xHandleFollowing, TaskHandle_t* xMasterHandle)
{
    if (VERBOSITY_LEVEL < 3) {
        return;
    }

    size_t freeHeap = xPortGetFreeHeapSize();
    Serial.println("Free Heap: " + String(freeHeap));

    UBaseType_t uxHighWaterMark;

    // For the rotate task
    uxHighWaterMark = uxTaskGetStackHighWaterMark(*xHandleRotating);
    Serial.print("TapeRotate stack high water mark: ");
    Serial.println(uxHighWaterMark);


    // For the reflectance task
    uxHighWaterMark = uxTaskGetStackHighWaterMark(*xReflectanceHandle);
    Serial.print("ReflectancePolling stack high water mark: ");
    Serial.println(uxHighWaterMark);


    // For the follow task
    uxHighWaterMark = uxTaskGetStackHighWaterMark(*xHandleFollowing);
    Serial.print("TapeFollow stack high water mark: ");
    Serial.println(uxHighWaterMark);


    // For the follow task
    uxHighWaterMark = uxTaskGetStackHighWaterMark(*xMasterHandle);
    Serial.print("Master stack high water mark: ");
    Serial.println(uxHighWaterMark);
}

extern "C"
{
  void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
  {
    // This function will be called if a stack overflow is detected
    Serial.print("Stack overflow in task: ");
    Serial.println(pcTaskName);
    // Enter an infinite loop or reset the system
    for (;;)
      ;
  }
}