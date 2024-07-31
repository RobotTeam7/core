#include <motion/utils.h>


void checkResetCause() {
    // if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
    //     Serial.println("System reset by Independent Watchdog Timer (IWDG).");
    // }
    // if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST)) {
    //     Serial.println("System reset by Window Watchdog Timer (WWDG).");
    // }
    // if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST)) {
    //     Serial.println("System reset by Power-on Reset.");
    // }
    // if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)) {
    //     Serial.println("System reset by Software Reset.");
    // }
    // if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)) {
    //     Serial.println("System reset by NRST pin.");
    // }
    // if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST)) {
    //     Serial.println("System reset by Low Power Reset.");
    // }
    // // if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST)) {
    // //     Serial.println("System reset by Brown-out Reset (BOR).");
    // // }
    // // Clear all reset flags
    // __HAL_RCC_CLEAR_RESET_FLAGS();
}

void monitorStackUsage(TaskHandle_t* xHandleRotating, TaskHandle_t* xReflectanceHandle, TaskHandle_t* xHandleFollowing, TaskHandle_t* xMasterHandle, TaskHandle_t* xStationTrackingHandle)
{
    if (VERBOSITY_LEVEL < 3) {
        return;
    }

    UBaseType_t uxHighWaterMark;

    size_t freeHeap = xPortGetFreeHeapSize();
    Serial.print("Free Heap: ");
    Serial.println((unsigned long)freeHeap, DEC);

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

    // For the station tracking task
    uxHighWaterMark = uxTaskGetStackHighWaterMark(*xStationTrackingHandle);
    Serial.print("Station tracking stack high water mark: ");
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

int get_last_station_server(int side_station, int y_direction) {
    if(y_direction == 1) {
       return side_station;
    } else if(y_direction == -1) {
        if(side_station == 1) {
            return 1;
        } else if(side_station == 2) {
            return 4;
        }else {
            log_error("there are only two stations on the bottom row of chef");
            return 0;
        }
    }else {
        log_error("y value must be 1 or -1");
        return 0;
    }
}


int get_last_station_chef(int side_station, int y_direction) {
    if(y_direction == 1) {
        switch(side_station) {
            case(1):
                return 2;
            case(2):
                return 4;
            case(3):
                return 6;
            default:
                log_error("there are only 3 top stations in chef row!");
                return 0;
        }
    } else if(y_direction == -1) {
        switch(side_station) {
            case(1):
                return 1;
            case(2):
                return 3;
            case(3):
                return 5;
            case(4):
                return 7;
            default:
                log_error("there are only 4 bottom stations in chef row");
                return 0;
        }
    } else {
        log_error("y value must be 1 or -1");
        return 0;
    }
}

int get_last_side_station_server(int last_station, int y_direction) {
    if(y_direction == 1) {
       return last_station;
    } else if(y_direction == -1) {
        switch(last_station) {
            case 1:
                return 1;
            case 4:
                return 4;
            default:
                log_error("invalid last_station, staion passed isn't a bottom station");
                return 0;
        }
    }else {
        log_error("y value must be 1 or -1");
        return 0;
    }
}

int get_last_side_station_chef(int last_station, int y_direction) {
    if(y_direction == 1) {
        switch(last_station) {
            case 2:
                return 1;
            case 4:
                return 2;
            case 6:
                return 3;
            default:
                log_error("invalid last_station, staion passed isn't a top station");
                return 0;
        }
    } else if(y_direction == -1) {
        switch(last_station) {
            case 1:
                return 1;
            case 3:
                return 2;
            case 5:
                return 3;
            case 7:
                return 4;
            default:
                log_error("nvalid last_station, staion passed isn't a bottom station");
                return 0;
        }
    } else {
        log_error("y value must be 1 or -1");
        return 0;
    }
}
