
  /* 初始化代码 */
    key_init();
    
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    for(uint8_t i = 0; i < KEY_MAX_NUM; i++) {
        key_event_t event = key_get_event((key_id_t)i);
        if(event != KEY_EVENT_NONE) {
            switch(event) {
                case KEY_EVENT_SHORT_PRESS:
                    // 处理短按事件
                    HAL_GPIO_TogglePin(LED_03_GPIO_Port, LED_03_Pin);
                    break;
                case KEY_EVENT_LONG_PRESS:
                    // 处理长按事件
                    
                    break;
                case KEY_EVENT_DOUBLE_CLICK:
                    // 处理双击事件
                    
                    break;
                default:
                    break;
            }
        }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      
      //HAL_Delay(10);
  }
  /* USER CODE END 3 */