#include <ESP8266WiFi.h>
//#include <WiFiServer.h>
/*
Мост ModbusTCP-server <-> ModbusRTU-master.

При старте скетча слушается 502 порт. Поступающие ModbusTCP запросы
транслируются в Setial с добавлением контрольной суммы и отсечением заголовка MBAP Header.
Адрес ведомого берется из ModbusTCP заголовка (UnitID). Полученные ответы транслируются
в обратном направлении.

Для функционирования необходимо задать:
ssid и password своей сети WiFi;
RS_Speed в соответствии со своими потребностями;
RS_pin - выход для переключения направления приём/передача(используется в микросхемах типа ADM485)
или индикации обмена (не обязателен).

Описание ModBusTCP:
http://www.simplymodbus.ca/TCP.htm
Описание МodBusRTU:
http://www.modbus.org/docs/Modbus_Application_Protocol_V1_1b3.pdf


Примеры использования http://www.webmico.ru
*/
//----------------------------------------------------------------------
// Serial config
#define RS_Speed 460800   //скорость соединения по Serial
#define RS_pin 2          //выход для переключения направления передачи
// (startBit+ DataBit+ stopBit)*maxByte* 1sec / speedBitBerSec + timeResponsePrepare 
#define RS_Timeout  (1 + 8 + 1) * 255 * 1000 / RS_Speed + 5
//----------------------------------------------------------------------
// defaul WiFi client config

//String Statuses[] =  { "WL_IDLE_STATUS=0", "WL_NO_SSID_AVAIL=1", "WL_SCAN_COMPLETED=2", "WL_CONNECTED=3", "WL_CONNECT_FAILED=4", "WL_CONNECTION_LOST=5", "WL_DISCONNECTED=6"};
//----------------------------------------------------------------------
// defaul WiFi AP config
WiFiMode wifi_mode = WiFiMode::WIFI_STA;

String ssidAP = "DepthHub";
const char* passwordAP = "DepthHub";
IPAddress ip_static(10, 10, 10, 10);    
IPAddress ip_gateway(10,10,10,1); 
IPAddress ip_subnet(255,255,255,0); 
IPAddress ip_dns(10,10,10,1);
//----------------------------------------------------------------------


WiFiServer server(502);
WiFiClient serverClient;
uint8_t wifiCondition = 0;
uint8_t buf[512];
uint8_t rsCondition = 0;
//----------------------------------------------------------------------
// количество пакетов для отслеживания качества связи
// если ошибок более 20% - перезагружаемся
uint32_t gPkgQtyTotal=0;
uint32_t gPkgQtyError=0;
#define QUALITY_PKG_QTY 100
#define QUALITY_LIMIT 20
//----------------------------------------------------------------------
// PIN DEFINE
#define BUILTIN_LED 2     // ESP-12E module's onboard LED, used as status indicator
#define BUILTIN_LED_OFF() digitalWrite(BUILTIN_LED, HIGH)
#define BUILTIN_LED_ON()  digitalWrite(BUILTIN_LED, LOW)

void status_blink()
{
    BUILTIN_LED_ON();
    delay(50);
    BUILTIN_LED_OFF();
}

//----------------------------------------------------------------------
//----- Таблица для вычисления CRC: -----
const uint16_t Crc16Table[] PROGMEM = {
	0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
	0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
	0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
	0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
	0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
	0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
	0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
	0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
	0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
	0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
	0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
	0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
	0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
	0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
	0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
	0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
	0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
	0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
	0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
	0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
	0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
	0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
	0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
	0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
	0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
	0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
	0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
	0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
	0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
	0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
	0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
	0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};
//----------------------------------------------------------------------
uint16_t CRC_modbus;
void CRC_16(uint8_t *b, uint16_t count) {
	CRC_modbus = 0xffff;
	for (uint16_t i = 0; i<count; i++) {
		uint8_t ptr = (CRC_modbus & 0x00ff) ^ ((uint8_t)b[i] & 0x00ff);
		CRC_modbus = pgm_read_word_near(Crc16Table + ptr) ^ (CRC_modbus >> 8);
	}
	b[count] = (uint8_t)(CRC_modbus & 0xff);
	b[count + 1] = (uint8_t)(CRC_modbus >> 8);
}
//----------------------------------------------------------------------
void setup() 
{
	pinMode(BUILTIN_LED, OUTPUT);
	BUILTIN_LED_OFF();

	// SERIAL SETUP
	Serial.setTimeout(RS_Timeout);
	Serial.begin(RS_Speed, SERIAL_8N1);
	Serial.println("");
	
	Serial.println("setup serial port speed - OK");

	// WIFI SETUP
	// PIN_14 
	pinMode(14, INPUT_PULLUP);// 
	//digitalWrite(14, HIGH);

	ssidAP = "DepthHub" + WiFi.macAddress();
	
	
	
	int colon_index = ssidAP.indexOf(':');
	while (-1 != colon_index)
	{
		ssidAP.remove(colon_index,1);
		colon_index = ssidAP.indexOf(':');
	}
	Serial.println(ssidAP.c_str());
	WiFi.hostname("DepthHub");//common wifi config
  
	wifi_mode = digitalRead(14) ? WIFI_STA : WIFI_AP;

	if (WiFiMode::WIFI_STA == wifi_mode)
	{
		Serial.print("WiFiMode::WIFI_STA ");
		if (!WiFi.getAutoConnect())
			WiFi.setAutoConnect(true);
		WiFi.setAutoReconnect(true);
		int reconect_attempt = 10;
		while (--reconect_attempt && WiFi.status() != WL_CONNECTED)
		{
			Serial.print("try WiFi reconect attempt ");
			Serial.println(reconect_attempt, DEC);
			status_blink();
			delay(150);
		}//while(--reconect_attempt)

		if (WiFi.status() != WL_CONNECTED)// autoconnect failed
		{
			Serial.println("WiFi NOT reconected");
			WifiConnectReset();
		}
		else
		{
			Serial.println("WiFi Reconnected");
			Serial.print(F("IP address: "));
			Serial.println(WiFi.localIP());
			WiFi.printDiag(Serial);
			Serial.print(F("RSSI: "));
			Serial.println(WiFi.RSSI());
			Serial.print(F("BSSID: "));
			Serial.println(WiFi.BSSIDstr());
			StartIpServer();
		}

	}
	else
	{
		Serial.print("WiFiMode::WIFI_AP ");
		WifiConnectReset();
	}
	Serial.println("setup wifi common config - OK");
  
  
	Serial.println("setup serial port alt output");
	Serial.flush();
	Serial.pins(15, 13);
	pinMode(RS_pin, OUTPUT);
	digitalWrite(RS_pin, 0);

	// read & reset the incoming byte:
	while(Serial.available())
		Serial.read();

}
//----------------------------------------------------------------------
void StartIpServer()
{
  server.begin();
  server.setNoDelay(true);
  // https://ru.wikipedia.org/wiki/Алгоритм_Нейгла
  // https://arduino-esp8266.readthedocs.io/en/latest/esp8266wifi/client-class.html
  wifiCondition = 10;
}
//----------------------------------------------------------------------
void WifiConnectReset()
{
	serverClient.stop();
	server.stop();

	WiFi.disconnect(true);
	WiFi.softAPdisconnect(true);
  
	switch (wifi_mode)
	{
	default: 
	case WIFI_STA:	wifiCondition = 0; break;
	case WIFI_AP:	wifiCondition = 2; break;
	//case WIFI_AP_STA:break;
	//case WIFI_OFF: break;
	}

	rsCondition = 0;
}
//----------------------------------------------------------------------
void WifiConnectAP(){
  //Serial.println("Start WIFI_AP");
  WiFi.mode(WIFI_AP); 

  WiFi.softAP(ssidAP.c_str(), passwordAP);
  WiFi.softAPConfig(ip_static, ip_gateway, ip_subnet); 
  

  StartIpServer();
  
}
//----------------------------------------------------------------------
//Подключение к WiFi точке доступа и старт сервера
void WifiConnectSTA() 
{
  //Serial.println("Try connect WIFI_STA");
  WiFi.mode(WIFI_STA);
  //WiFi.config(ip_static, ip_gateway, ip_subnet); 
  //for(uint32_t i=0; i< sizeof(ssid)/sizeof(char); ++i)
  {
    WiFi.begin(ssidAP.c_str(), passwordAP);
    status_blink();
    if(WL_CONNECTED == WiFi.waitForConnectResult() )
    {
      StartIpServer();
      return;
    }
  }
  WifiConnectReset();

}


//----------------------------------------------------------------------
void WifiConnect()
{
  switch (wifiCondition) 
  {
    default: //WiFi отключен
      WifiConnectReset();
      break;

    case 0: // пробуем подключиться к точке доступа 
      WifiConnectSTA();
      break;

    case 2: // стартуем точку доступа 
      WifiConnectAP();
      break;
  
   case 10: //Ожидание подключения и обработка
    if(WIFI_STA==WiFi.getMode() && WL_CONNECTED!=WiFi.status() )
    {
      WifiConnectReset();
      return;
    }
    
    {
      if (server.hasClient()) //find free/disconnected spot
      {
        if (!serverClient || !serverClient.connected()) 
        {
          if (serverClient) 
            serverClient.stop();
          serverClient = server.available();
          return;
        }
        WiFiClient sClient = server.available();
        sClient.stop();
      }

      /*Если сервер запущен и поступили данные*/
      if (serverClient && serverClient.connected()) {
        if (serverClient.available()) {
          //Сперва считывается заголовок ModbusTCP, затем считывается остаток кадра в зависимости от
          //заданной в заголовке длины (-1 ,тк адрес ведомого передается в заголовке после длины):
          if (serverClient.readBytes(buf, 7) == 7) { serverClient.readBytes(&buf[7], ((buf[4] << 8) + buf[5] - 1)); }
          serverClient.flush();
          rsCondition = 1;
        }
      }
    }//if(WL_CONNECTED==WiFi.status())
    break;//case 10
  }//switch (wifiCondition)   
  
}//void WifiConnect()
//----------------------------------------------------------------------

void loop() {
  WifiConnect();


	uint16 c=0;//длина сообщения
	switch (rsCondition) {
	default: break;
	case 1:
		//Поступило сообщение:
		c = (buf[4] << 8) + buf[5];//выделяем его длину
		CRC_16(&buf[6], c);//вычиляем контрольную сумму и добавляем ее в конец
		Serial.write(&buf[6], c + 2);//Отправляем RTU запрос
		digitalWrite(RS_pin, 1);//переключение на приём
		rsCondition = 2;
		break;
	case 2:
		//Получение ответа:
		switch (buf[7]) {//вычисление ожидаемой длины ответа в зависимости от функции в запросе
		default: break;
		case 1://01 (0x01) Read Coils
		case 2://02 (0x02) Read Discrete Inputs
			c = 3 + 2 + ((buf[10] << 8) + buf[11]) / 8 + (((buf[10] << 8) + buf[11]) % 8>0 ? 1 : 0);
			break;
		case 3://03 (0x03) Read Holding Registers
		case 4://04 (0x04) Read Input Registers
			c = 3 + 2 + ((buf[10] << 8) + buf[11]) * 2;
			break;
		case 5://05 (0x05) Write Single Coil
		case 6://06 (0x06) Write Single Register
		case 15://15 (0x0F) Write Multiple Coils
		case 16://16 (0x10) Write Multiple registers
			c = 3 + 5;
			break;
		case 22://22 (0x16) Mask Write Register
			c = 3 + 7;
			break;
		case 23://23 (0x17) Read/Write Multiple registers
			c = 3 + 2 + ((buf[14] << 8) + buf[15]) * 2;
			break;
		}//switch (buf[7]) {//вычисление ожидаемой длины ответа в зависимости от функции в запросе

		/*
		if(++gPkgQtyTotal>=QUALITY_PKG_QTY)
		{
		  if( QUALITY_LIMIT < 100*gPkgQtyError/QUALITY_PKG_QTY )
			ESP.restart();
		  else
			gPkgQtyTotal=gPkgQtyError=0;
		}
		*/
    
		//uint16 rcqty = Serial.readBytes(&buf[6], c);//считывание ответа.
		c = Serial.readBytes(&buf[6], c);//считывание ответа.

		CRC_16(&buf[6], c);//проверка контрольной суммы
		if (CRC_modbus == 0 && c >= 5 /*&& rcqty==c*/) {//контрольная сумма верна и ответ был
			buf[4] = ((c - 2) >> 8);
			buf[5] = ((c - 2) & 0xff);
			serverClient.write(&buf[0], c + 6 - 2);//отсылаем ответ ModbusTCP-клиенту
		}
		else
		{
			c = Serial.readBytes(&buf[6], 255);//считывание отставшихся байт до таймаута
			gPkgQtyError++;
			delay(1);
		}
		rsCondition = 0;
		digitalWrite(RS_pin, 0);//переключение на передачу
		break;
	}//switch (rsCondition) {
}


