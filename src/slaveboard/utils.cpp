#include <slaveboard/utils.h>

void checkResetCause()
{
  if (RCC->CSR & RCC_CSR_IWDGRSTF)
  {
    Serial.println("System reset by Independent Watchdog Timer (IWDG).");
  }
  if (RCC->CSR & RCC_CSR_WWDGRSTF)
  {
    Serial.println("System reset by Window Watchdog Timer (WWDG).");
  }
  if (RCC->CSR & RCC_CSR_PORRSTF)
  {
    Serial.println("System reset by Power-on Reset.");
  }
  if (RCC->CSR & RCC_CSR_SFTRSTF)
  {
    Serial.println("System reset by Software Reset.");
  }
  if (RCC->CSR & RCC_CSR_PINRSTF)
  {
    Serial.println("System reset by NRST pin.");
  }
  if (RCC->CSR & RCC_CSR_LPWRRSTF)
  {
    Serial.println("System reset by Low Power Reset.");
  }
  RCC->CSR |= RCC_CSR_RMVF;
}

void monitorStackUsage(TaskHandle_t* xHandleRotating, TaskHandle_t* xReflectanceHandle, TaskHandle_t* xHandleFollowing, TaskHandle_t* xMasterHandle)
{
    UBaseType_t uxHighWaterMark;

    // For the rotate task
    uxHighWaterMark = uxTaskGetStackHighWaterMark(*xHandleRotating);
    if (VERBOSITY_LEVEL >= 3) {
    Serial.print("TapeRotate stack high water mark: ");
    Serial.println(uxHighWaterMark);
    }

    // For the reflectance task
    uxHighWaterMark = uxTaskGetStackHighWaterMark(*xReflectanceHandle);
    if (VERBOSITY_LEVEL >= 3) {
        Serial.print("ReflectancePolling stack high water mark: ");
        Serial.println(uxHighWaterMark);
    }

    // For the follow task
    uxHighWaterMark = uxTaskGetStackHighWaterMark(*xHandleFollowing);
    if (VERBOSITY_LEVEL >= 3) {
        Serial.print("TapeFollow stack high water mark: ");
        Serial.println(uxHighWaterMark);
    }

    // For the follow task
    uxHighWaterMark = uxTaskGetStackHighWaterMark(*xMasterHandle);
    if (VERBOSITY_LEVEL >= 3) {
        Serial.print("Master stack high water mark: ");
        Serial.println(uxHighWaterMark);
    }
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

void log_status(const char* status_message) {
    if (VERBOSITY_LEVEL >= STATUS_MESSAGES) {
        Serial.println(status_message);
    }    
}

void log_error(const char* error_message) {
    if (VERBOSITY_LEVEL >= ERRORS_ONLY) {
        Serial.println(error_message);
    }    
}

void log_message(const char* message) {
    if (VERBOSITY_LEVEL >= MOST_VERBOSE) {
        Serial.println(message);
    }    
} 