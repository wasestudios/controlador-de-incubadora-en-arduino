
#include <LiquidCrystal_I2C.h>
#include <RTClib.h>
#include <EEPROM.h>
#include <DHT.h>


//Definimos los pines de entrada y salida 
byte PIN_BTN_DOWN = 2;
byte PIN_BTN_SETUP = 3;
byte PIN_BTN_UP = 4;
byte PIN_DS18B20 = 0;//no utilizado
byte PIN_DHT22 = 5;
byte PIN_BUZZER = 6;
byte PIN_ACTU_0 = 8;
byte PIN_ACTU_1 = 9;
byte PIN_ACTU_2 = 10;
byte PIN_ACTU_3 = 11;
byte PIN_TRAY_ECHO = A2;
byte PIN_TRAY_TRIG = A3;
//--------------------------------

//Variables que se utilizan para formatear los tipos de datos al ser llamados de la EEPROM
float DATA_FLOAT;
int DATA_INT;
//--------------------------------

//Inicializamos lcd,reloj,dht22
LiquidCrystal_I2C lcd(0x27,20,4);  //0x3F
RTC_DS3231 rtc;
DHT dht(PIN_DHT22, DHT22);
//--------------------------------

//variables para configurar la fecha del sistema
byte rtc_day=0;
byte rtc_month=0;
int  rtc_year=2022;
byte rtc_hour=0;
byte rtc_minute=0;
//--------------------------------

//Variables de configuracion general y las que se editan en el menu de configuracion
byte   INCUBATION_INIT; //inicia  detiene el periodo de incubacion
unsigned long MAIN_DATE_EPOCH,MAIN_TIME_CONTROL,MAIN_DATE_ROTATION,maximumRunningTime,daysToActiveHatcher;
int    TIME_CALIBRATION;//aumenta o reduce segundos al tiempo en format epoch
float  TEMP_CALIBRATION;//aumenta o reduce grados a la lectura de temperatura del dht22
float  HUME_CALIBRATION;//aumenta o reduce el porcentaje de la humedad relativa a la lectura de humedad del dht22
unsigned int  TRAY_CALIBRATION[3]; //calibra las tres posiciones de la bandeja en cm segun el sensor HC-SR04
byte   DAYS_INCUBATION;//establece los dias de incubacion 
unsigned int    TIME_FOR_ROTATION;//establece el tiempo que de espera para hacer la rotacion de las bandejas
float  RANGE_TEMP[2];//establece el rango mínimo y máximo aceptable de temperatura
float  RANGE_HUME[2];//establece el rango mínimo y máximo aceptable de humedad
bool   ACTIVE_HATCHER;//Habilita o deshabilita la nacedora
byte   ACTIVE_DAYS_HATCHER;//establece  a cuantos dias desde que inició la incubación tiene que activar la nacedora
float  RANGE_HATCHER_TEMP[2];//establece el rango mínimo y máximo aceptable de temperatura cuando la nacedora esté activa
float  RANGE_HATCHER_HUME[2];//establece el rango mínimo y máximo aceptable de humedad cuando la nacedora esté activa
byte   ACTIVE_ALARM;//Habilita  desabilita la alarma
byte   PERIOD_FINALICE;//Varibale utilizada para verificar si ha finalizado el periodo de incubacion
bool   ROTATION_NOW = false;//Variable utilizada para verificar el estado de la rotación de bandejas
char   POSITION_TRAY_NOW = 'P';//Variable utilizada para saber en cual de las tres posicione se encuentra la bandeja
//--------------------------------

float SENSO_TEMP,SENSO_HUME;//variables utilizadas para la lectura del sensor DHT22

bool triggerAlarma=false;// disparador de alarma

bool triggerHatcher = true;//disparador de nacedora
bool hatcherInOperation = false;//verifica si la nacedora esta en operaciones con confundir con si esta habilitada

unsigned long timeAlarma = 0;
byte typeTone = 0;
bool movementTrayControlConfig = false;
byte flag_temp=0;
byte flag_hume=0;

//Variables utilzadas para el control de los botones
bool press_setup = false;
bool press_increment = false;
bool press_decrement = false;
byte config_status = 0;//opcion de configuracion de 0 a 2
byte config_value = 0;//opciones de menu de 0 a 7
byte config_option = 0;//opciones de accion dentro de los menu de 0 - ...
bool inSetup = false;
bool inSave = false;
byte page = 1;
//--------------------------------

byte timeControl = -1;
long timeRunning = 0;
long timeRunningForRotation = 0;
byte data[8];

//Caracteres personalizados, se guardan en la memoria flash para no cupar SRAM
static const byte ico_cursor_left[8] PROGMEM = {2,6,15,31,31,15,6,2};
static const byte ico_cursor_right[8]  PROGMEM  = { B01000,B01100,B11110,B11111,B11111,B11110,B01100,B01000};
static const byte ico_temp[8]  PROGMEM   = {B10101,B01110,B11111,B01110,B00100,B00100,B00100,B01110};
static const byte ico_hum[8]   PROGMEM   = {B01110,B11111,B11111,B11111,B00000,B10101,B00000,B01010};
static const byte ico_tray[8]  PROGMEM   = {B00100,B01110,B11111,B00000,B00000,B11111,B01110,B00100};
static const byte ico_calendar[8] PROGMEM = {B00000,B01010,B11111,B10101,B11011,B10101,B11111,B00000};
static const byte ico_cool[8] PROGMEM = {B01010,B10101,B00000,B01010,B10101,B00000,B01010,B10101};
//--------------------------------

/**Devuelve los caracteres personalizados guardados en flash
 * @byte modo es el tipo de caracter que se quiere recuperar
 */ 
void returnChar(byte modo){
  for(byte i=0; i<8; i++){
    if(modo==0) data[i] = pgm_read_byte(ico_cursor_left+i);
    if(modo==1) data[i] = pgm_read_byte(ico_cursor_right+i);
    if(modo==2) data[i] = pgm_read_byte(ico_temp+i);
    if(modo==3) data[i] = pgm_read_byte(ico_hum+i);
    if(modo==4) data[i] = pgm_read_byte(ico_tray+i);
    if(modo==5) data[i] = pgm_read_byte(ico_calendar+i);
    if(modo==6) data[i] = pgm_read_byte(ico_cool+i);
  }
}


void setup() {
  Serial.begin(9600);
  if (!rtc.begin()) Serial.println(F("Modulo RTC no encontrado !"));  
  lcd.init();
  lcd.backlight();
  dht.begin();
  
  pinMode(PIN_BTN_DOWN,INPUT_PULLUP);
  pinMode(PIN_BTN_SETUP,INPUT_PULLUP);
  pinMode(PIN_BTN_UP,INPUT_PULLUP);
  pinMode(PIN_ACTU_0,OUTPUT);
  pinMode(PIN_ACTU_1,OUTPUT);
  pinMode(PIN_ACTU_2,OUTPUT);
  pinMode(PIN_ACTU_3,OUTPUT);
  digitalWrite(PIN_ACTU_0,HIGH);
  digitalWrite(PIN_ACTU_1,HIGH);
  digitalWrite(PIN_ACTU_2,HIGH);
  digitalWrite(PIN_ACTU_3,HIGH);
  pinMode(PIN_TRAY_TRIG, OUTPUT); //pin como salida
  pinMode(PIN_TRAY_ECHO, INPUT);  //pin como entrada
  digitalWrite(PIN_TRAY_TRIG, LOW);//Inicializamos el pin con 0
  pinMode(13,OUTPUT);
  returnChar(0);
  lcd.createChar (0,data);
  returnChar(1);
  lcd.createChar (1,data);
  returnChar(2);
  lcd.createChar (2,data);
  returnChar(3);
  lcd.createChar (3,data);
  returnChar(4);
  lcd.createChar (4,data);
  returnChar(5);
  lcd.createChar (5,data);
  returnChar(6);
  lcd.createChar (6,data);
  getInitData();
  Serial.println(freeRam());//Verificar memoria ram disponible, se puede comentar
}



void loop() {
    buzzer();//controla el buzzer para no bloquear el codigo con delay

    //lectura del temperatura y humedad añadiendo la calibracion
    SENSO_TEMP   = (dht.readTemperature()+TEMP_CALIBRATION);
    SENSO_HUME   = (dht.readHumidity()+HUME_CALIBRATION);

    //si la posicion de la bandeja es indefinida que se representa con P mueve las bandejas a la posicion media
    if(POSITION_TRAY_NOW=='P')  ROTATION_NOW =true;

    //Incrementa el tiempo de funcionamiento y el tiempo para rotacion
    if(oneSecondPased() and INCUBATION_INIT){
      timeRunning++;
      timeRunningForRotation++;
    }

   

      

    {/*Seccion de control de temperatura- metodo on-off en funcion de un rango de temperatura*/
      if(SENSO_TEMP>=RANGE_TEMP[0] and SENSO_TEMP<=RANGE_TEMP[1]){
        digitalWrite(PIN_ACTU_0,HIGH);
        digitalWrite(PIN_ACTU_3,HIGH);
        flag_temp = 0;
      }

      if(SENSO_TEMP<RANGE_TEMP[0]){
        digitalWrite(PIN_ACTU_0,LOW);
        digitalWrite(PIN_ACTU_3,HIGH);
        flag_temp = 1;
      }

      if(SENSO_TEMP>RANGE_TEMP[1]){
        digitalWrite(PIN_ACTU_0,HIGH);
        /*Habilitar ventilador de extraccion*/
        digitalWrite(PIN_ACTU_3,LOW);
        flag_temp = 2;
      }

      /*Control de Humedad*/
      if(SENSO_HUME>=RANGE_HUME[0] and SENSO_HUME<=RANGE_HUME[1]){
        digitalWrite(PIN_ACTU_1,HIGH);
        if(flag_temp!=2) digitalWrite(PIN_ACTU_3,HIGH);
        flag_hume = 0;
      }

      if(SENSO_HUME<RANGE_HUME[0]){
        digitalWrite(PIN_ACTU_1,LOW);
        if(flag_temp!=2) digitalWrite(PIN_ACTU_3,HIGH);
        flag_hume = 1;
      }

      if(SENSO_HUME>RANGE_HUME[1]){
        digitalWrite(PIN_ACTU_1,HIGH);
        /*Habilitar ventilador de extraccion*/
        digitalWrite(PIN_ACTU_3,LOW);
        flag_hume=2;
      }

    }

    {//controla tiempo de finalizacion del periodo de incubacion
       if(timeRunning > maximumRunningTime){
            INCUBATION_INIT = false;
            PERIOD_FINALICE = true;
        }
    }
    

    {//Ejecucion de parametros de nacedora
      if(ACTIVE_HATCHER and triggerHatcher){
          if(timeRunning >= daysToActiveHatcher){
              triggerHatcher = false;
              hatcherInOperation = true;
              //if(POSITION_TRAY_NOW!='M') moveTrayTo('M');
              RANGE_TEMP[0] = RANGE_HATCHER_TEMP[0];
              RANGE_TEMP[1] = RANGE_HATCHER_TEMP[1];

              RANGE_HUME[0] = RANGE_HATCHER_HUME[0];
              RANGE_HUME[1] = RANGE_HATCHER_HUME[1];
          }  
      }
    }

    
    

    {/*Seccion de rotacion de bandeja*/
      if(timeRunningForRotation>=TIME_FOR_ROTATION and config_status==0 and  INCUBATION_INIT and !hatcherInOperation){
        ROTATION_NOW=true;
        setMainDateRotation(rtc.now().unixtime()+(TIME_CALIBRATION*60));
        getInitData();
      }

      if(ROTATION_NOW and config_status==0 and INCUBATION_INIT){
        if(POSITION_TRAY_NOW=='U'){
          moveTrayTo('D');
        }else if(POSITION_TRAY_NOW=='D'){
          moveTrayTo('U');
        }else if(POSITION_TRAY_NOW=='M'){
          moveTrayTo('U');
        }else{
          moveTrayTo('M');
        }
      }
    }

    
    
    /*Display menu--------------------------------------------------------------------------------------*/
    if(config_status==0){//menu principal
       displayMenu(1);
    }else if(config_status==1){//Lista menu de configuracion
      displayMenu(2);
    }else if(config_status==2){//ejecuta configuracion
      if(pressBtnUp() && !inSetup)config_option++;
      if(pressBtnDown() && !inSetup)config_option--;
      displayMenu(3);
      
      //opciones de salida y guardado
      if(pressBtnSetup()){
        
        if(config_option ==0){
            /*Sale de la configuracion de opciones de menu al menu de opciones*/
            config_status = 1;
            lcd.clear();
        }
        /*Detectar opciones mayores que uno pero con excepcion para config_value 0p1(inicio) y 0p2(calibracion de bandeja)*/
        if(config_option>1){

            if(page==1 and config_value==0){
              switch(config_option){
                case 2://inicia periodo incubacion
                   setMainDateEpoch(rtc.now().unixtime()+(TIME_CALIBRATION*60));
                   setMainDateRotation(rtc.now().unixtime()+(TIME_CALIBRATION*60)); 
                   setIncubationInit(1);
                   getInitData();
                   PERIOD_FINALICE = false;
                   triggerAlarma = true;
                   
                break;
                case 3://detiene periodo de incubacion
                   setIncubationInit(0);
                   getInitData();
                   triggerAlarma = true;
                break;
                case 4://Mueve las vandejas segun posicion
                    ROTATION_NOW = true;
                    triggerAlarma = true;
                break;
              }
              
            }else if(page==2 and config_value==0){
              movementTrayControlConfig=!movementTrayControlConfig;
              movementTrayControlConfig ? (ROTATION_NOW = true) : (ROTATION_NOW = false);
              inSave = true;
            }else{
              /*Alterna entre el modo de edicion de variables*/
              inSetup = !inSetup;
              inSave = true;
            }
            
        }

        if(inSave && config_option==1){
            inSave = false;
            /*acciones que guarda las variables en eeprom segun el valor de config_value*/
            if(page==1){
              switch(config_value){
                case 1:
                  setDaysIncutation(DAYS_INCUBATION);
                  setTimeForRotation(TIME_FOR_ROTATION/3600);
                  setRangeTemp(RANGE_TEMP[0],0);
                  setRangeTemp(RANGE_TEMP[1],1);
                  setRangeHum(RANGE_HUME[0],0);
                  setRangeHum(RANGE_HUME[1],1);
                break;

                case 2:
                  setActiveHatcher(ACTIVE_HATCHER);
                  if(!ACTIVE_HATCHER){
                    hatcherInOperation = false;
                  }else{
                    ACTIVE_HATCHER = true;
                    triggerHatcher = true;
                  }
                  
                  setHatcherDays(ACTIVE_DAYS_HATCHER);
                  setRangeHatcherTemp(RANGE_HATCHER_TEMP[0] , 0);
                  setRangeHatcherTemp(RANGE_HATCHER_TEMP[1] , 1);
                  setRangeHatcherHum(RANGE_HATCHER_HUME[0] , 0);
                  setRangeHatcherHum(RANGE_HATCHER_HUME[1] , 1);
                break;

                case 3:
                  setTempCalibration(TEMP_CALIBRATION);
                  setHumeCalibration(HUME_CALIBRATION);
                  setTimeCalibration(TIME_CALIBRATION);
                break;
              }
              
            }else if(page==2){
              switch(config_value){
                case 0://calibracion de bandeja
                  setTrayCalibration(TRAY_CALIBRATION[0],0);
                  setTrayCalibration(TRAY_CALIBRATION[1],1);
                  setTrayCalibration(TRAY_CALIBRATION[2],2);
                break;
                case 1://calibracion de fecha del sistema
                  rtc.adjust(DateTime(rtc_year,rtc_month,rtc_day,rtc_hour,rtc_minute,0));
                break;
              }
            }
            /*acciones que guarda las variables en eeprom segun el valor de config_value*/
            Serial.println("He guardado todo");
            getInitData();
            triggerAlarma = true;
        }

      }

      lcd.setCursor(19,0);
      if(inSetup){
        /*modificion de variables segun config_value y config_option*/
        if(pressBtnUp()){
          updateVariables(page,config_value,config_option, true);
        }

        if(pressBtnDown()){
          updateVariables(page,config_value,config_option, false);
        }
        
        lcd.print("*");
      }else lcd.print(" ");

        
        
    }
    /*Fin display menu---------------------------------------------------------------*/
      
 }

/**
 * Permite generar el tono de guardado y coonfiguracion de variables sin bloquear el código
 */
void buzzer(){
  if(triggerAlarma){
    if(typeTone==0){
      if(timeAlarma==0) timeAlarma = millis();
      unsigned long diff = millis()-timeAlarma;
      if(diff>=0 and diff<=100) tone(PIN_BUZZER ,2000,100);
      if(diff>=200 and diff<=250) noTone(PIN_BUZZER);
      if(diff>=250 and diff<=350) tone(PIN_BUZZER ,1500,100);
      if(diff>=350 and diff<=400) noTone(PIN_BUZZER);
      if(diff>=400 and diff<=600) tone(PIN_BUZZER ,3000,200);
      if(diff>=600){
        triggerAlarma=false;
        timeAlarma= 0;
        noTone(PIN_BUZZER);
      }
    }
  }
}

/**
 * Mueve las bandejas a una posicion indicada U-M-D
 * input char position puede ser U-M-D
 */
void moveTrayTo(char position){
  switch(position){
    case 'U':
      if(getDistancia()==TRAY_CALIBRATION[0]){
         digitalWrite(PIN_ACTU_2,HIGH);
         POSITION_TRAY_NOW='U';
         ROTATION_NOW = false;
      }else digitalWrite(PIN_ACTU_2,LOW);
    
    break;
    case 'M':
      if(getDistancia()==TRAY_CALIBRATION[1]){
         digitalWrite(PIN_ACTU_2,HIGH);
         POSITION_TRAY_NOW='M';
         ROTATION_NOW = false;
      }else digitalWrite(PIN_ACTU_2,LOW);
    
    break;
    case 'D':
      if(getDistancia()==TRAY_CALIBRATION[2]){
         digitalWrite(PIN_ACTU_2,HIGH);
         POSITION_TRAY_NOW='D';
         ROTATION_NOW = false;
      }else digitalWrite(PIN_ACTU_2,LOW);
    
    break;
  }
}

int getDistancia(){
  digitalWrite(PIN_TRAY_TRIG, HIGH);
  delayMicroseconds(10);          //Enviamos un pulso de 10us
  digitalWrite(PIN_TRAY_TRIG, LOW);
  //t = pulseIn(Echo, HIGH); //obtenemos el ancho del pulso
  return pulseIn(PIN_TRAY_ECHO, HIGH)/59;             //escalamos el tiempo a una distancia en cm
}

/**
 * Modifica las variables de configuracion segun el incremento o decremento de los botones
 */
void updateVariables(byte page, byte config_value,byte config_option, bool modo){
  if(page==1){
    switch(config_value){
      case 1://Incubacion
        switch(config_option){
          case 2://dias incubacion
            DAYS_INCUBATION = modo ? DAYS_INCUBATION+=1 : DAYS_INCUBATION-=1;
          break;
          case 3://tiemp de rotacion
            TIME_FOR_ROTATION = modo ? TIME_FOR_ROTATION+=3600 : TIME_FOR_ROTATION-=3600;
          break;
          case 4://rango tempe ini
            RANGE_TEMP[0] = modo ?  RANGE_TEMP[0]+=0.1 : RANGE_TEMP[0]-=0.1;
          break;
          case 5://rang tempe end
            RANGE_TEMP[1] = modo ?  RANGE_TEMP[1]+=0.1 : RANGE_TEMP[1]-=0.1;
          break;
          case 6://rango Hume ini
            RANGE_HUME[0] = modo ? RANGE_HUME[0]+=0.1 : RANGE_HUME[0]-=0.1; 
          break;
          case 7://rang hume end
            RANGE_HUME[1] = modo ? RANGE_HUME[1]+=0.1 : RANGE_HUME[1]-=0.1;
          break;
        }
      
      break;
      case 2://Nacedora
         switch(config_option){
          case 2://Activacion de incubadora
            ACTIVE_HATCHER = modo ? 1 : 0;
          break;
          case 3://dias desde la incubacion para activacion de nacedora
            ACTIVE_DAYS_HATCHER = modo ? ACTIVE_DAYS_HATCHER+=1 : ACTIVE_DAYS_HATCHER-=1;
          break;
          case 4://rango cambio de temperatura
            RANGE_HATCHER_TEMP[0] = modo ?  RANGE_HATCHER_TEMP[0]+=0.1 : RANGE_HATCHER_TEMP[0]-=0.1;
          break;
          case 5://rang tempe end
            RANGE_HATCHER_TEMP[1] = modo ?  RANGE_HATCHER_TEMP[1]+=0.1 : RANGE_HATCHER_TEMP[1]-=0.1;
          break;
          case 6://rango Hume ini
            RANGE_HATCHER_HUME[0] = modo ? RANGE_HATCHER_HUME[0]+=0.1 : RANGE_HATCHER_HUME[0]-=0.1; 
          break;
          case 7://rang hume end
            RANGE_HATCHER_HUME[1] = modo ? RANGE_HATCHER_HUME[1]+=0.1 : RANGE_HATCHER_HUME[1]-=0.1;
          break;
        }
      
      break;
      case 3://calibrar
        switch(config_option){
          case 2://calibracion de temperatura
            TEMP_CALIBRATION =  modo ?  TEMP_CALIBRATION+=0.1 : TEMP_CALIBRATION-=0.1;
          break;
          case 3://calibracion de humedad
            HUME_CALIBRATION =  modo ?  HUME_CALIBRATION+=0.1 : HUME_CALIBRATION-=0.1;
          break;
          case 4://calibracion de tiempo
            TIME_CALIBRATION = modo ?  TIME_CALIBRATION+=1 : TIME_CALIBRATION-=1;
          break;
        }
      break;
    }
    
  }else if(page==2){
    switch(config_value){
      case 0://calibrar bandejas
        switch(config_option){
          case 2://calibrar posicion media
            TRAY_CALIBRATION[1] = modo ?  TRAY_CALIBRATION[1]+=1 : TRAY_CALIBRATION[1]-=1;
          break;
          case 3://calibrar posicion arriba
            TRAY_CALIBRATION[0] = modo ?  TRAY_CALIBRATION[0]+=1 : TRAY_CALIBRATION[0]-=1;
          break;
          case 4://calibrar psicion abajo
            TRAY_CALIBRATION[2] = modo ?  TRAY_CALIBRATION[2]+=1 : TRAY_CALIBRATION[2]-=1;
          break;
        }
      
      break;
      case 1://ajustar hora del sistema
        switch(config_option){
          case 2://ajustar dia
            rtc_day = modo ?  rtc_day+=1 : rtc_day-=1;
          break;
          case 3://ajustar mes
            rtc_month = modo ?  rtc_month+=1 : rtc_month-=1;
          break;
          case 4://ajustar año
            rtc_year = modo ?  rtc_year+=1 : rtc_year-=1;
          break;

          case 5://ajustar hora
            rtc_hour = modo ?  rtc_hour+=1 : rtc_hour-=1;
          break;

          case 6://ajustar minuto
            rtc_minute = modo ?  rtc_minute+=1 : rtc_minute-=1;
          break;
        }
      
      break;
      case 2:
      
      break;
      case 3:
      
      break;
    }
  }else;
  
}

/**
 * Permite visualizar las opciones de menu en el display
 */

void displayMenu(byte tipo){
  switch(tipo){
    case 1://display principal
        if(INCUBATION_INIT)screenTimeIncubation(timeRunning);
        lcd.setCursor(0,1);
        lcd.write(2);
        lcd.print(SENSO_TEMP);
        lcd.print("C");
        lcd.setCursor(12,1);
         lcd.write(3);
        lcd.print(SENSO_HUME);
        lcd.print("%");
        if(INCUBATION_INIT) screenTimeForRotation(TIME_FOR_ROTATION-timeRunningForRotation);
        
        lcd.setCursor(0,3);
        if(digitalRead(PIN_ACTU_3 )==LOW){
          lcd.write(6);
          lcd.write(6);
        }else{
          lcd.print("  ");
        }

        lcd.setCursor(4,3);
        if(flag_temp>0){
          lcd.write(2);
          flag_temp==1 ? lcd.print("-") : lcd.print("+");
        }else{
          lcd.print("  ");
        }

        lcd.setCursor(8,3);
        if(flag_hume>0){
          lcd.write(3);
          flag_hume==1 ? lcd.print("-") : lcd.print("+");
        }else{
          lcd.print("  ");
        }

        if(INCUBATION_INIT){
          lcd.setCursor(12,3);
          if(hatcherInOperation){
            lcd.print("NA");
          }else{
            lcd.print("  ");
          }
        }

        if(!INCUBATION_INIT and PERIOD_FINALICE){
          lcd.setCursor(0,2);
          lcd.print("Periodo finalizado");
        }
        

        
        if(pressBtnSetup()){
          config_status = 1;
          config_value = 0;
          lcd.clear();
        }
    break;
    case 2://opciones de configuracion
        if(pressBtnUp()){
          config_value++;
          if(config_value>3){
            config_value=0;
            page++;
            if(page>2) page=1; 
            lcd.clear();   
          }
  
          lcd.setCursor(0,config_value-1);
          lcd.print(" ");
        }
  
        if(pressBtnDown()){
            config_value--;
            if(config_value==255){
              config_value=3;
              page--;
              page = page<1 ? 2 : page=1;    
              lcd.clear();
            }
            lcd.setCursor(0,config_value+1);
            lcd.print(" ");
        }
        
        lcd.setCursor(0,config_value);
        lcd.write(1);
        
        if(page==1){
          lcd.setCursor(2,0);
          lcd.print("Inicio");
          lcd.setCursor(2,1);
          lcd.print("Incubacion");
          lcd.setCursor(2,2);
          lcd.print("Nacedora");
          lcd.setCursor(2,3);
          lcd.print("Calibrar");
        }else if(page==2){
          lcd.setCursor(2,0);
          lcd.print("Calibrar bandeja");
          lcd.setCursor(2,1);
          lcd.print("Fecha del sistema");
          lcd.setCursor(2,2);
          lcd.print("Fechas grabadas");
          lcd.setCursor(2,3);
          lcd.print("Salir");
        }
  
        if(pressBtnSetup()) {
          if(config_value==3 && page==2){
            config_status = 0;page=1;lcd.clear();
          }else{
            config_status = 2;lcd.clear();
          }
        }
    break;
    case 3://opciones internas de cada menu
        if(page==1){
            switch(config_value){
                case 0://opciones Incio
                  lcd.setCursor(0,0);
                  lcd.print("INICIAR");
                  lcd.setCursor(12,0);
                  if(config_option==2){lcd.write(0);}else{lcd.print(" ");}
                  lcd.setCursor(0,1);
                  lcd.print("DETENER");
                  lcd.setCursor(12,1);
                  if(config_option==3){lcd.write(0);}else{lcd.print(" ");}
                  lcd.setCursor(0,2);
                  lcd.print("MOV BANDE");
                  lcd.setCursor(12,2);
                  if(config_option==4){lcd.write(0);}else{lcd.print(" ");}
                  lcd.setCursor(14,2);
                  lcd.write(4);
                  lcd.print(POSITION_TRAY_NOW);
                  /*----------------------*/
                  if(ROTATION_NOW){
                    switch(POSITION_TRAY_NOW){
                      case 'U':
                        moveTrayTo('M');
                      break;
                      case 'M':
                        moveTrayTo('D');
                      break;
                      case 'D':
                        moveTrayTo('U');
                      break;
                      default:
                        moveTrayTo('M');
                      break;
                    }
                  }
                  /*----------------------*/
                  if(config_option==255) config_option=4;
                  if(config_option>4) config_option=0;
                break;
                case 1://opciones  Incubacion
                  lcd.setCursor(0,0);
                  //lcd.print("D-INC:");
                  lcd.write(5);
                  if(DAYS_INCUBATION<10) lcd.print("0");
                  lcd.print(DAYS_INCUBATION);
                  lcd.print("d");
                  if(config_option==2){lcd.write(0);}else{lcd.print(" ");}
      
                  lcd.setCursor(0,1);
                  //lcd.print("T-ROT:");
                  lcd.write(4);
                  if(TIME_FOR_ROTATION<36000) lcd.print("0");
                  lcd.print(TIME_FOR_ROTATION/3600);
                  lcd.print("h");
                  if(config_option==3){lcd.write(0);}else{lcd.print(" ");}
      
                  lcd.setCursor(0,2);
                  //lcd.print("TEMP:");
                  lcd.write(2);
                  lcd.print(RANGE_TEMP[0]);
                  if(config_option==4){
                    lcd.write(0);
                  }else if(config_option==5){
                    lcd.write(1);
                  }else{lcd.print("-");}
                  lcd.print(RANGE_TEMP[1]);
                  
      
                  lcd.setCursor(0,3);
                  //lcd.print("HUME:");
                  lcd.write(3);
                  lcd.print(RANGE_HUME[0]);
                  if(config_option==6){
                    lcd.write(0);
                  }else if(config_option==7){
                    lcd.write(1);
                  }else{lcd.print("-");}
                  lcd.print(RANGE_HUME[1]);
                  
                  if(config_option==255) config_option=7;
                  if(config_option>7) config_option=0;
                break;
                case 2://opciones  Nacedora
                  lcd.setCursor(0,0);
                  lcd.print("ACTIVADO:");
                  lcd.print(ACTIVE_HATCHER ? "SI" : "NO");
                  if(config_option==2){lcd.write(0);}else{lcd.print(" ");}
      
                  lcd.setCursor(0,1);
                  //lcd.print("D-ACTIVA:");
                  lcd.write(5);
                  if(ACTIVE_DAYS_HATCHER<10) lcd.print("0");
                  lcd.print(ACTIVE_DAYS_HATCHER);
                  lcd.print("d");
                  if(config_option==3){lcd.write(0);}else{lcd.print(" ");}
      
                  lcd.setCursor(0,2);
                  //lcd.print("TEMP:");
                  lcd.write(2);
                  lcd.print(RANGE_HATCHER_TEMP[0]);
                  if(config_option==4){
                    lcd.write(0);
                  }else if(config_option==5){
                    lcd.write(1);
                  }else{lcd.print("-");}
                  lcd.print(RANGE_HATCHER_TEMP[1]);
                  
      
                  lcd.setCursor(0,3);
                  //lcd.print("HUME:");
                  lcd.write(3);
                  lcd.print(RANGE_HATCHER_HUME[0]);
                  if(config_option==6){
                    lcd.write(0);
                  }else if(config_option==7){
                    lcd.write(1);
                  }else{lcd.print("-");}
                  lcd.print(RANGE_HATCHER_HUME[1]);
                  
                  if(config_option==255) config_option=7;
                  if(config_option>7) config_option=0;
                break;
                case 3://opciones  calibrar
                  lcd.setCursor(0,0);
                  //lcd.print("TA:");
                  lcd.write(2);
                  lcd.print(SENSO_TEMP);
                  lcd.print(" A:");
                  if(TEMP_CALIBRATION<10)lcd.print("0");
                  lcd.print(TEMP_CALIBRATION);
                  lcd.setCursor(15,0);
                  if(config_option==2){lcd.write(0);}else{lcd.print(" ");}
                  
                  lcd.setCursor(0,1);
                  //lcd.print("HA:");
                  lcd.write(3);
                  lcd.print(SENSO_HUME);
                  lcd.print(" A:");
                  lcd.print(HUME_CALIBRATION);
                  lcd.setCursor(15,1);
                  if(config_option==3){lcd.write(0);}else{lcd.print(" ");}
                  
                  DateTime dd = rtc.now();
                  lcd.setCursor(0,2);
                  if(dd.day()<10) lcd.print("0");
                  lcd.print(dd.day());
                  lcd.print("-");
                  if(dd.month()<10) lcd.print("0");
                  lcd.print(dd.month());
                  lcd.print("-");
                  lcd.print(dd.year());
                  lcd.print(" ");
                  if(dd.hour()<10) lcd.print("0");
                  lcd.print(dd.hour());
                  lcd.print(":");
                  if(dd.minute()<10) lcd.print("0");
                  lcd.print(dd.minute());
      
                  lcd.setCursor(0,3);
                  lcd.print("AD:");
                  lcd.print(TIME_CALIBRATION);
                  lcd.print("min");
                  if(config_option==4){lcd.write(0);}else{lcd.print(" ");}
                  
                  if(config_option==255) config_option=4;
                  if(config_option>4) config_option=0;
                break;
            }
        }
        
        if(page==2){
            switch(config_value){
              case 0://opciones  calibrar bandeja
                if(ROTATION_NOW){
                  digitalWrite(PIN_ACTU_2,LOW);
                }else digitalWrite(PIN_ACTU_2,HIGH);
                
                lcd.setCursor(0,0);
                lcd.print("MED:");
                if(TRAY_CALIBRATION[1]<10) lcd.print("0");
                if(TRAY_CALIBRATION[1]>10 and TRAY_CALIBRATION[1]<100) lcd.print("0");
                if(TRAY_CALIBRATION[1]>100 and TRAY_CALIBRATION[1]<1000) lcd.print("0");
                lcd.print(TRAY_CALIBRATION[1]);
                lcd.print("cm");
                lcd.setCursor(17,0);
                if(config_option==2){
                  if(movementTrayControlConfig)TRAY_CALIBRATION[1] = getDistancia();
                  lcd.write(0);
                }else{lcd.print(" ");}

                lcd.setCursor(0,1);
                lcd.print("ARR:");
                if(TRAY_CALIBRATION[0]<10) lcd.print("0");
                if(TRAY_CALIBRATION[0]>10 and TRAY_CALIBRATION[0]<100) lcd.print("0");
                if(TRAY_CALIBRATION[0]>100 and TRAY_CALIBRATION[0]<1000) lcd.print("0");
                lcd.print(TRAY_CALIBRATION[0]);
                lcd.print("cm");
                lcd.setCursor(17,1);
                if(config_option==3){
                  if(movementTrayControlConfig) TRAY_CALIBRATION[0] = getDistancia();
                  lcd.write(0);
                }else{lcd.print(" ");}

                lcd.setCursor(0,2);
                lcd.print("ABA:");
                if(TRAY_CALIBRATION[2]<10) lcd.print("0");
                if(TRAY_CALIBRATION[2]>10 and TRAY_CALIBRATION[2]<100) lcd.print("0");
                if(TRAY_CALIBRATION[2]>100 and TRAY_CALIBRATION[3]<1000) lcd.print("0");
                lcd.print(TRAY_CALIBRATION[2]);
                lcd.print("cm");
                lcd.setCursor(17,2);
                if(config_option==4){
                  if(movementTrayControlConfig)TRAY_CALIBRATION[2] = getDistancia();
                  lcd.write(0);
                }else{lcd.print(" ");}
                
                if(config_option==4){lcd.write(0);}else{lcd.print(" ");}
                if(config_option==255) config_option=4;
                if(config_option>4) config_option=0;
              break;
      
              case 1://opciones  fecha del sistema
                
                lcd.setCursor(0,0);
                lcd.print("D:");
                if(rtc_day<10) lcd.print("0");
                lcd.print(rtc_day);
                if(config_option==2){lcd.write(0);}else{lcd.print(" ");}
                
                lcd.setCursor(5,0);
                lcd.print("M:");
                if(rtc_month<10) lcd.print("0");
                lcd.print(rtc_month);
                if(config_option==3){lcd.write(0);}else{lcd.print(" ");}

                lcd.setCursor(10,0);
                lcd.print("A:");
                lcd.print(rtc_year);
                if(config_option==4){lcd.write(0);}else{lcd.print(" ");}

                lcd.setCursor(0,1);
                lcd.print("hh:");
                if(rtc_hour<10) lcd.print("0");
                lcd.print(rtc_hour);
                if(config_option==5){lcd.write(0);}else{lcd.print(" ");}

                lcd.setCursor(7,1);
                lcd.print("mm:");
                if(rtc_minute<10) lcd.print("0");
                lcd.print(rtc_minute);
                if(config_option==6){lcd.write(0);}else{lcd.print(" ");}
                
                if(config_option==255) config_option=6;
                if(config_option>6) config_option=0;
              break;
              case 2://opciones  Fechas guardadas
                lcd.setCursor(0,0);
                lcd.print("Ini Incubation");
                lcd.setCursor(0,1);
                lcd.print(MAIN_DATE_EPOCH);
                lcd.setCursor(0,2);
                lcd.print("Ult Rotation");
                lcd.setCursor(0,3);
                lcd.print(MAIN_DATE_ROTATION);
                if(config_option==255) config_option=1;
                if(config_option>1) config_option=0;
              break;
            }
        }
        
        /*--------inici btn salir, guardar, configurar--------*/
        lcd.setCursor(18,2);
        if(config_option==1){
          lcd.write(1);lcd.print("G");
        }else lcd.print(" G");

        lcd.setCursor(18,3);
        if(config_option==0){
          lcd.write(1);lcd.print("S");
        }else lcd.print(" S");
        
        /*--------fin btn salir, guardar, configurar----------*/
    break;
  }
}
   
/**
 * Verrifica si ha transcurrido un segundo dentro de void loop
 */
bool oneSecondPased(){
  bool passed = false;
  DateTime fecha = rtc.now();
  if(timeControl==-1)
        timeControl = fecha.second();
        
  if(timeControl!=fecha.second()){
        timeControl = fecha.second();
        timeRunning+=1;
        passed = true;
  }
  return passed;
}

/**
 * Verifica si se ha presionado el boton setup
 */
bool pressBtnSetup(){
    bool btnPress = false;
    if(digitalRead(PIN_BTN_SETUP) == LOW && press_setup==false){
        press_setup = true;
        btnPress = true;
    }
       
    if (digitalRead(PIN_BTN_SETUP) == HIGH  && press_setup){
        press_setup = false;
    }
    return btnPress;
}
/**
 * Verifica si se ha presionado el boton de incremento
 */
bool pressBtnUp(){
    bool btnPress = false;
    if(digitalRead(PIN_BTN_UP) == LOW && press_increment==false){
        press_increment = true;
        btnPress = true;
    }
       
    if (digitalRead(PIN_BTN_UP) == HIGH  && press_increment){
        press_increment = false;
    }
    return btnPress;
}
/**
 * Verifica si se se ha presionado el botono decremento
 */
bool pressBtnDown(){
    bool btnPress = false;
    if(digitalRead(PIN_BTN_DOWN) == LOW && press_decrement==false){
        press_decrement = true;
        btnPress = true;
    }
       
    if (digitalRead(PIN_BTN_DOWN) == HIGH  && press_decrement){
        press_decrement = false;
    }
    return btnPress;
}

/**
 * Formatea el tiempo transcurrido de la incubacion
 */
void screenTimeIncubation(long iTiempo){
  long iminutos = iTiempo / 60;
  long isegundos = iTiempo - (iminutos * 60);
  long ihoras = iminutos / 60;
  iminutos = iminutos - (ihoras * 60);
  long idias = ihoras / 24;
  ihoras = ihoras - (idias * 24);

  lcd.setCursor(4,0);
  lcd.write(5);
  if(idias<10) lcd.print("0");
  lcd.print(idias);
  lcd.print("-");
  if(ihoras<10) lcd.print("0");
  lcd.print(ihoras);
  lcd.print(":");
  if(iminutos<10) lcd.print("0");
  lcd.print(iminutos);
  lcd.print(":");
  if(isegundos<10) lcd.print("0");
  lcd.print(isegundos);
}
/**
 * Formatea el tiempo transcurrido para la siguiente rotacion
 */
void screenTimeForRotation(long iTiempo){
  long iminutos = iTiempo / 60;
  long isegundos = iTiempo - (iminutos * 60);
  long ihoras = iminutos / 60;
  iminutos = iminutos - (ihoras * 60);
  long idias = ihoras / 24;
  ihoras = ihoras - (idias * 24);

  lcd.setCursor(3,2);
  lcd.write(4);
  if(ihoras<10) lcd.print("0");
  lcd.print(ihoras);
  lcd.print(":");
  if(iminutos<10) lcd.print("0");
  lcd.print(iminutos);
  lcd.print(":");
  if(isegundos<10) lcd.print("0");
  lcd.print(isegundos);
  lcd.print("  ");
  lcd.write(4);
  lcd.print(POSITION_TRAY_NOW);
}

/**
 * Inicializa todas las variables matrices
 */
void getInitData(){
  if(getIncubationInit()==255){
    setValuesFactory();
  }
  MAIN_DATE_EPOCH = getMainDateEpoch();
  MAIN_DATE_ROTATION = getMainDateRotation();
  TIME_CALIBRATION = getTimeCalibration();
  TEMP_CALIBRATION = getTempCalibration();
  TRAY_CALIBRATION[0] = getTrayCalibration(0);
  TRAY_CALIBRATION[1] = getTrayCalibration(1);
  TRAY_CALIBRATION[2] = getTrayCalibration(2);
  HUME_CALIBRATION = getHumeCalibration();
  DAYS_INCUBATION = getDaysIncutation();
  TIME_FOR_ROTATION = getTimeForRotation();
  RANGE_TEMP[0] = getRangeTemp(0);
  RANGE_TEMP[1] = getRangeTemp(1);
  RANGE_HUME[0] = getRangeHum(0);
  RANGE_HUME[1] = getRangeHum(1);
  ACTIVE_HATCHER = getActiveHatcher();
  ACTIVE_DAYS_HATCHER = getHatcherDays();
  RANGE_HATCHER_TEMP[0] = getRangeHatcherTemp(0);
  RANGE_HATCHER_TEMP[1] = getRangeHatcherTemp(1);
  RANGE_HATCHER_HUME[0] = getRangeHatcherHum(0);
  RANGE_HATCHER_HUME[1] = getRangeHatcherHum(1);
  ACTIVE_ALARM =  getActiveAlarma();
  INCUBATION_INIT = getIncubationInit();
  PERIOD_FINALICE = getPeriodoFinalice();
  MAIN_TIME_CONTROL = rtc.now().unixtime();
  maximumRunningTime = (long)DAYS_INCUBATION * 86400;
  daysToActiveHatcher = (long) ACTIVE_DAYS_HATCHER * 86400;
  timeRunning = ((MAIN_TIME_CONTROL+ (TIME_CALIBRATION*60) ) - MAIN_DATE_EPOCH);
  timeRunningForRotation = ((MAIN_TIME_CONTROL+(TIME_CALIBRATION*60) ) - MAIN_DATE_ROTATION);
  TIME_FOR_ROTATION =  TIME_FOR_ROTATION * 3600; //De horas a segundos
}

/**
 * Devuelve los valores a fábrica
 */
void setValuesFactory(){
  setMainDateEpoch(1642547497); //18-01-2021 23:10 aprox
  setMainDateRotation(1642547497); //18-01-2021 23:10 aprox
  setTimeCalibration(0);
  setTempCalibration(0.5);
  setHumeCalibration(1);
  setTrayCalibration(10,0);
  setTrayCalibration(15,1);
  setTrayCalibration(20,2);
  setDaysIncutation(22);
  setTimeForRotation(6);
  setRangeTemp(37.5,0);
  setRangeTemp(37.8,1);
  setRangeHum(59.3,0);
  setRangeHum(59.8,1);
  setActiveHatcher(0);
  setHatcherDays(18);
  setRangeHatcherTemp(36 , 0);
  setRangeHatcherTemp(36 , 1);
  setRangeHatcherHum(70 , 0);
  setRangeHatcherHum(70 , 1);
  setActiveAlarma(1);
  setIncubationInit(0);
  setPeriodoFinalice(0);
  Serial.println(F("Datos devueltos a fabrica"));
}


/**
 * Getter y setters
 */

long  getMainDateEpoch(){
    return EEPROM.get(0,MAIN_DATE_EPOCH);
}

long getMainDateRotation(){
    return EEPROM.get(4,MAIN_DATE_ROTATION);
}

int getTimeCalibration(){
    return EEPROM.get(8,TIME_CALIBRATION);
}

float getTempCalibration(){
    return EEPROM.get(12,TEMP_CALIBRATION);
}

float  getHumeCalibration(){
    return EEPROM.get(28,HUME_CALIBRATION);
}

int getTrayCalibration(byte modo){
      if(modo==0)
        return EEPROM.get(16,DATA_INT);
      
      if(modo==1)
        return EEPROM.get(20,DATA_INT);
      
      if(modo==2)
        return EEPROM.get(24,DATA_INT);
    
}

byte getDaysIncutation(){
    return EEPROM.read(32);
}

byte getTimeForRotation(){
    return EEPROM.read(33);
}

float getRangeTemp(byte modo){
      if(modo==0)
        return EEPROM.get(34,DATA_FLOAT);
      
      if(modo==1)
        return EEPROM.get(38,DATA_FLOAT);
}

float getRangeHum(byte modo){
    switch(modo){
      case 0:
        return EEPROM.get(42,DATA_FLOAT);
      break;
      case 1:
        return EEPROM.get(46,DATA_FLOAT);
      break;
    }
}

byte getActiveHatcher(){
    return EEPROM.read(50);
}

byte getHatcherDays(){
    return EEPROM.read(51);
}

float getRangeHatcherTemp(byte modo){
    switch(modo){
      case 0:
        return EEPROM.get(52,DATA_FLOAT);
      break;
      case 1:
        return EEPROM.get(56,DATA_FLOAT);
      break;
    }
}

float getRangeHatcherHum(byte modo){
    switch(modo){
      case 0:
        return EEPROM.get(60,DATA_FLOAT);
      break;
      case 1:
        return EEPROM.get(64,DATA_FLOAT);
      break;
    }
}

byte getActiveAlarma(){
    return EEPROM.read(68);
}

byte getIncubationInit(){
    return EEPROM.read(69);
}

byte getPeriodoFinalice(){
    return EEPROM.read(70);
}

//setters
void setMainDateEpoch(long tiempo){
    EEPROM.put(0,tiempo);
}

void setMainDateRotation(long tiempo){
    EEPROM.put(4,tiempo);
}

void setTimeCalibration(int tiempo){
    EEPROM.put(8,tiempo);
}

void setTempCalibration(float temp){
    EEPROM.put(12,temp);
}

void setHumeCalibration(float hum){
    EEPROM.put(28,hum);
}

void setTrayCalibration(int measure, byte modo){
      if(modo==0)
        EEPROM.put(16,measure);
      
      if(modo==1)
        EEPROM.put(20,measure);
      
      if(modo==2)
        EEPROM.put(24,measure);
}

void setDaysIncutation(byte days){
    EEPROM.update(32,days);
}

void setTimeForRotation(byte tiempo){
    EEPROM.update(33,tiempo);
}

void setRangeTemp(float temp, byte modo){
    switch(modo){
      case 0:
        EEPROM.put(34,temp);
      break;
      case 1:
        EEPROM.put(38,temp);
      break;
    }
}

void setRangeHum(float hum, byte modo){
    switch(modo){
      case 0:
        EEPROM.put(42,hum);
      break;
      case 1:
        EEPROM.put(46,hum);
      break;
    }
}

void setActiveHatcher(byte hatcher){
    EEPROM.update(50,hatcher);
}

void setHatcherDays(byte days){
    EEPROM.update(51,days);
}

void setRangeHatcherTemp(float temp, byte modo){
    switch(modo){
      case 0:
        EEPROM.put(52,temp);
      break;
      case 1:
        EEPROM.put(56,temp);
      break;
    }
}

void setRangeHatcherHum(float hum, byte modo){
    switch(modo){
      case 0:
        EEPROM.put(60,hum);
      break;
      case 1:
        EEPROM.put(64,hum);
      break;
    }
}

void setActiveAlarma(byte active){
    EEPROM.update(68,active);
}

void setIncubationInit(byte active){
    EEPROM.update(69,active);
}

void setPeriodoFinalice(byte finalice){
    EEPROM.update(70,finalice);
}


int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
