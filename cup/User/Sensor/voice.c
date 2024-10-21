#include "voice.h"
#include "main.h"
void sound_init(){
    HAL_GPIO_WritePin(sound1_GPIO_Port,sound1_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(sound2_GPIO_Port,sound3_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(sound3_GPIO_Port,sound3_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(sound4_GPIO_Port,sound4_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(sound5_GPIO_Port,sound5_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(sound6_GPIO_Port,sound6_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(sound7_GPIO_Port,sound7_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(sound8_GPIO_Port,sound8_Pin,GPIO_PIN_SET);
}

void sound_play(int a){
    if(a==1){ //播放第一首曲 “准备完毕” 00001.mp3
        HAL_GPIO_WritePin(sound1_GPIO_Port,sound1_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(sound2_GPIO_Port,sound3_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(sound3_GPIO_Port,sound3_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(sound4_GPIO_Port,sound4_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(sound5_GPIO_Port,sound5_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(sound6_GPIO_Port,sound6_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(sound7_GPIO_Port,sound7_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(sound8_GPIO_Port,sound8_Pin,GPIO_PIN_SET);
        HAL_Delay(30);
        HAL_GPIO_WritePin(sound1_GPIO_Port,sound1_Pin,GPIO_PIN_SET);
    }
    else if(a==2){//播放第二首曲目 “到达指定景点” 00002.mp3
        HAL_GPIO_WritePin(sound1_GPIO_Port,sound1_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(sound2_GPIO_Port,sound3_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(sound3_GPIO_Port,sound3_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(sound4_GPIO_Port,sound4_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(sound5_GPIO_Port,sound5_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(sound6_GPIO_Port,sound6_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(sound7_GPIO_Port,sound7_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(sound8_GPIO_Port,sound8_Pin,GPIO_PIN_SET);
        HAL_Delay(30);
        HAL_GPIO_WritePin(sound2_GPIO_Port,sound3_Pin,GPIO_PIN_SET);
    }
    else if(a == 3){//"到达二号平台"
        HAL_GPIO_WritePin(sound1_GPIO_Port,sound1_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(sound2_GPIO_Port,sound3_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(sound3_GPIO_Port,sound3_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(sound4_GPIO_Port,sound4_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(sound5_GPIO_Port,sound5_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(sound6_GPIO_Port,sound6_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(sound7_GPIO_Port,sound7_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(sound8_GPIO_Port,sound8_Pin,GPIO_PIN_SET);
        HAL_Delay(30);        
        HAL_GPIO_WritePin(sound3_GPIO_Port,sound3_Pin,GPIO_PIN_SET);

    }
    else if(a == 4){//"到达三号平台"
        HAL_GPIO_WritePin(sound1_GPIO_Port,sound1_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(sound2_GPIO_Port,sound3_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(sound3_GPIO_Port,sound3_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(sound4_GPIO_Port,sound4_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(sound5_GPIO_Port,sound5_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(sound6_GPIO_Port,sound6_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(sound7_GPIO_Port,sound7_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(sound8_GPIO_Port,sound8_Pin,GPIO_PIN_SET);  
        HAL_Delay(30);        
        HAL_GPIO_WritePin(sound4_GPIO_Port,sound4_Pin,GPIO_PIN_SET);      
    }
    else if(a == 5){//'到达四号'
        HAL_GPIO_WritePin(sound1_GPIO_Port,sound1_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(sound2_GPIO_Port,sound3_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(sound3_GPIO_Port,sound3_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(sound4_GPIO_Port,sound4_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(sound5_GPIO_Port,sound5_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(sound6_GPIO_Port,sound6_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(sound7_GPIO_Port,sound7_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(sound8_GPIO_Port,sound8_Pin,GPIO_PIN_SET);
        HAL_Delay(30);        
        HAL_GPIO_WritePin(sound5_GPIO_Port,sound5_Pin,GPIO_PIN_SET);
    }
    else if(a == 6){//'到达五号号'
    HAL_GPIO_WritePin(sound1_GPIO_Port,sound1_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(sound2_GPIO_Port,sound3_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(sound3_GPIO_Port,sound3_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(sound4_GPIO_Port,sound4_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(sound5_GPIO_Port,sound5_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(sound6_GPIO_Port,sound6_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(sound7_GPIO_Port,sound7_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(sound8_GPIO_Port,sound8_Pin,GPIO_PIN_SET);
    HAL_Delay(30);        
    HAL_GPIO_WritePin(sound6_GPIO_Port,sound6_Pin,GPIO_PIN_SET);
}
else if(a == 7){//'到达六号'
    HAL_GPIO_WritePin(sound1_GPIO_Port,sound1_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(sound2_GPIO_Port,sound3_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(sound3_GPIO_Port,sound3_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(sound4_GPIO_Port,sound4_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(sound5_GPIO_Port,sound5_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(sound6_GPIO_Port,sound6_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(sound7_GPIO_Port,sound7_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(sound8_GPIO_Port,sound8_Pin,GPIO_PIN_SET);
    HAL_Delay(30);        
    HAL_GPIO_WritePin(sound7_GPIO_Port,sound7_Pin,GPIO_PIN_SET);
}
else if(a == 8){//'到达八号'
    HAL_GPIO_WritePin(sound1_GPIO_Port,sound1_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(sound2_GPIO_Port,sound3_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(sound3_GPIO_Port,sound3_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(sound4_GPIO_Port,sound4_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(sound5_GPIO_Port,sound5_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(sound6_GPIO_Port,sound6_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(sound7_GPIO_Port,sound7_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(sound8_GPIO_Port,sound8_Pin,GPIO_PIN_RESET);
    HAL_Delay(30);        
    HAL_GPIO_WritePin(sound8_GPIO_Port,sound8_Pin,GPIO_PIN_SET);
}
}
