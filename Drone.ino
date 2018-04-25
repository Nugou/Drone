/*
 * -------------Lembrete para o que precisa ser feito ou ideias para por no drone-------
 * -----------
 * (Adicionar ideias no drone caso precise)
 * (Terminado)Montar o codigo de operaçao dos motores.
 * (Descontinuado)Montar o codigo do modulo RF 433Mhz.
 * (Descontinuado)Montar o codigo da Camera(Vga Ov7670). 
 * (Descontinuado)Montar o codigo do Modulo RF Lora1278(Melhor) -> Biblioteca pronta no github.
 * (Terminado)(Descontinuado)Montar o codigo do Modulo HC-05. -> Teste feitos com sucesso, mais ainda da para melhorar. (desistencia desse modulo, pois curta distancia).
 * Montar o codigo EEPROM para memorizar dados importante. 
 * (Terminado)Montar o codigo do MPU-6050. -> Esperando testes no drone.
 * Montar o codigo para o drone voltar a origem quando: estar com bateria fraca, perdeu conexao com o controle. 
 * (Terminado)Montar o codigo do controle de PS2.
 * Montar o codigo de funçao do drone, como dar 360°, monitoramento de area, etc.
 * Montar o codigo do modulo GPS.
 * (Terminado) Montar o codigo do modulo nrf24l01, -> necessario mais teste para implementar no drone com sucesso 
 * Extender o codigo do modulo nrf24l01 para verificação de conexão 
 */

 

/*
 * ========================================================================================================================================================================================
 * #Declarando bibliotecas e define
 */
#include <MPUX.h>
#include <Servo.h>
#include <PIDX.h>
#include <SPI.h>
#include <RF24.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#define POT_MAX 255
#define POT_MIN -255

/*
 * ========================================================================================================================================================================================
 * #Fim da Declaração de biblioteca
 */


/*
 * ========================================================================================================================================================================================
 * #Declarando variaveis globais
 */
 
int MPU_end = 0x68; //endereço do sensor MPU-6050

int pino_motor[4] = {3, 4, 5, 6}; //pino de controle dos motores
int potMotor[4] = {POT_MIN, POT_MIN, POT_MIN, POT_MIN}; //Potencia final dos motores
int potMotorTemp[4] = {0, 0, 0, 0}; //Potencia temporaria dos motores
int potencia[4] = {0, 0, 0, 0}; //Potencia pré-calculada dos motores

int pino_led = 2; //Led informativo de funcionamento

int nrf_power = 3; //Utilize: 1 -> potencia baixa // 2 -> potencia minima // 3 -> potencia alta // 4 -> potencia maxima 

int Ly = 128; //Dados do analogico da esquerda eixo Y
int Lx = 128; //Dados do analogico da esquerda eixo X
int Ry = 128; //Dados do analogico da direita eixo Y
int Rx = 128; //Dados do analogico da direita eixo X

int Ly_final = 0; //Dados do analogico da esquerda eixo Y final
int Lx_final = 0; //Dados do analogico da esquerda eixo X final
int Ry_final = 0; //Dados do analogico da direita eixo Y final
int Rx_final = 0; //Dados do analogico da direita eixo X final

int upDown = 0; //Numero de parametros no eixo z
int roll = 0; //Numero de parametros no eixo y
int param = 4; //Numeros de parametros no eixo x
int param_upDown = 1; //Multiplcador do paramentros upDown

long timeCheckerMPU = 50; //Delay de verificaçao do MPU-6050
long timeCheckerCOM = 5; //Delay de verificaçao do Modulo de comunicação geral.(Modulo BlueTooth, Modulo RF NRF24L01)
long timeCheckerGPS = 1; //Delay de verifição do MOdulo de GPS
long timeCheckerDAT = 300; //Delay de verifição do Envio de Dados

unsigned long timeDAT = timeCheckerDAT;
unsigned long timeGPS = timeCheckerGPS;
unsigned long timeMPU = timeCheckerMPU; //Tempo para leitura do módulo MPU-6050.(Nada alterar aqui)
unsigned long timeCOM = timeCheckerCOM; //Tempo para leitura do módulo de comunicação.(Nada alterar aqui)

unsigned long gps_sentido = 0; //Sentido em graus do drone

double gps_velocidade = 0; //Velocidade do drone
double gps_altitude = 0; //Altitude do drone
double gps_altitude_ori = 0; //Altitude do drone
double gps_latitude = 0; //Latitude do drone
double gps_longitude = 0; //Longitude do drone
double gps_latitude_ori = 0; //Latitude do drone
double gps_longitude_ori = 0; //Longitude do drone

double motor_Kp = 1.5; //tenta aproximar do valor setpoint (padrão: 1.0) 
double motor_Ki = 0.0; //adiciona valores para aproximar aproximação (padrão: 0.0002)
double motor_Kd = 0.5; //Tenta manter estavel os valores (padrão: 0.0056)
double motor_Setpoint = 0.0; //Setpoint inicial do PID 

double gyX, gyY, gyZ; //Dados da MPU
double gyX_final, gyY_final, gyZ_final; //Dados da MPU
double gyX_ori, gyY_ori, gyZ_ori; //Dados da MPU iniciais
double mulMPU = 0.0056; //Multiplicador para converter os dados da MPU em angulo aproximado(-90 / 90)

boolean radioNumber = true; //Tipo do nrf24l01
boolean testMode = true; //Ativa e Desativa o modo de teste
boolean btn_enabled[16]; //Estados dos botões
boolean gps_data = false; //Confirma sem todos os dados estão prontos

byte addresses[][6] = {"1Node","2Node"}; //endereço de comunicação dos nrf24l01

/*
 * ========================================================================================================================================================================================
 * #Fim da Declaração das variaveis globais
 */

 

/*
 * ========================================================================================================================================================================================
 * #Declarando objetos
 */
TinyGPSPlus gps;
SoftwareSerial serial(9,10); //Rx -- Tx
RF24 radio(7,8);
MPUX mpu(MPU_end);
Servo ServoMotor_0;
Servo ServoMotor_1;
Servo ServoMotor_2;
Servo ServoMotor_3;
PIDX pidMotor_0(motor_Kp, motor_Ki, motor_Kd, motor_Setpoint);
PIDX pidMotor_1(motor_Kp, motor_Ki, motor_Kd, motor_Setpoint);
PIDX pidMotor_2(motor_Kp, motor_Ki, motor_Kd, motor_Setpoint);
PIDX pidMotor_3(motor_Kp, motor_Ki, motor_Kd, motor_Setpoint);

/*
 * ========================================================================================================================================================================================
 * Fim da Declaração dos objetos
 */


/*
 * ========================================================================================================================================================================================
 * #Declarando Estruturas
 */

struct stDataDrone{
	int stPotMotor[4] = {POT_MIN, POT_MIN, POT_MIN, POT_MIN};
 	int stBactery = 0;
 	double stHeight = 0;
 	double stGyZ = 0;
 	double stGpsLatitude = 0;
	double stGpsLongitude = 0; 
};
typedef struct stDataDrone typeDataRf;
typeDataRf recDataRf;
typeDataRf traDataRf;

/*
 * ========================================================================================================================================================================================
 * #Fim de declarando estruturas
 */
 

/*
 * ========================================================================================================================================================================================
 * Configurações Iniciais
 */

void setup() {
	mpu.begin();
	Serial.begin(115200);
	serial.begin(9600);
	SPI.begin();
	radio.begin();
	
	/////////////////////////////////////////////////////////////////////

	ServoMotor_0.attach(pino_motor[0]);
	ServoMotor_1.attach(pino_motor[1]);
	ServoMotor_2.attach(pino_motor[2]);
	ServoMotor_3.attach(pino_motor[3]);

	ServoMotor_0.write(potMotor[0]);
	ServoMotor_1.write(potMotor[1]);
	ServoMotor_2.write(potMotor[2]);
	ServoMotor_3.write(potMotor[3]);

	/////////////////////////////////////////////////////////////////////

	if(radioNumber){
		radio.openWritingPipe(addresses[1]);
		radio.openReadingPipe(1,addresses[0]);
	}else{
		radio.openWritingPipe(addresses[0]);
		radio.openReadingPipe(1,addresses[1]);
	}
	if(nrf_power == 1){radio.setPALevel(RF24_PA_LOW);}
	else if(nrf_power == 2){radio.setPALevel(RF24_PA_MIN);}
	else if(nrf_power == 3){radio.setPALevel(RF24_PA_HIGH);}
	else{radio.setPALevel(RF24_PA_MAX);}
	
	radio.stopListening();

	pinMode(pino_led, OUTPUT);
	digitalWrite(pino_led, LOW);
	while(!btn_enabled[0] and !testMode){
		moduleNrf();
		delay(30);
	}
	digitalWrite(pino_led, HIGH);

	/////////////////////////////////////////////////

	mpu.compute();
	delay(500);
	//Calcula a posição inicial do drone para definir a diferença de inclinação
	gyX_ori = mpu.getGyX()*mulMPU;
	gyY_ori = mpu.getGyY()*mulMPU;
	gyZ_ori = mpu.getGyZ()*mulMPU;

	Serial.print("X: ");Serial.println(gyX_ori);
	Serial.print("Y: ");Serial.println(gyY_ori);
	Serial.print("Z: ");Serial.println(gyZ_ori);

	for(int i = 0; i < 16; i++){
		btn_enabled[i] = false;
	}
	/*
	char red = 'r';
	Serial.println("Enviando confirmação!!!");
	while(!radio.write(red, sizeof(char))){
		delay(300);
	}
	Serial.println("Confirmado!!!");*/
	radio.startListening();
}

/*
 * ========================================================================================================================================================================================
 * Fim das Configurações Iniciais
 */ 



/*
 * ========================================================================================================================================================================================
 * Inicio Do Loop Arduino
 */

void loop() {
	//Verificador do modulo MPU-6050(Giroscopio)
	if(timeMPU <= millis()){
		timeMPU += timeCheckerMPU;
		mpu.compute();
	}

	//Verificador do modulo nrf24l01
	if(timeCOM <= millis()){
		timeCOM += timeCheckerCOM;
		moduleNrf();
	}

	//Verificador do modulo gps
	if(timeGPS <= millis()){
		timeGPS += timeCheckerGPS;
		moduleGps();
	}

	//Verificador do Envio de Dados
	if(timeDAT <= millis()){
		timeDAT += timeCheckerDAT;
		logDisplay();
	}
	
	Lx_final = map(Lx, 0, 255, 45, -45); //Remapeia os valores dos analogico para angulo
	Ly_final = map(Ly, 0, 255, -45, 45); //Remapeia os valores dos analogico para angulo
	Rx_final = map(Rx, 0, 255, -90, 90); //Remapeia os valores dos analogico para potencia de giro
	Ry_final = map(Ry, 0, 255, 179, -179);  //Remapeia os valores dos analogico para potencia de subida e descida
	
	//Inteligencia do drone
	if(Lx_final != 0 || Ly_final != 0 || Rx_final != 0 || Ry_final != 0){
		gyX = Ly_final - gyX_ori;//-41
		gyY = Lx_final - gyY_ori;
		//gyZ = Ry_final - gyZ_ori;

		if(mpu.getGyX()*mulMPU - gyX_ori <= gyX and gyX < 0){gyX = -(mpu.getGyX()*mulMPU - gyX_ori);}
		if(mpu.getGyX()*mulMPU - gyX_ori >= gyX and gyX > 0){gyX = -(mpu.getGyX()*mulMPU - gyX_ori);}
		
		if(mpu.getGyY()*mulMPU - gyY_ori <= gyY and gyY < 0){gyY = -(mpu.getGyY()*mulMPU - gyY_ori);}
		if(mpu.getGyY()*mulMPU - gyY_ori >= gyY and gyY > 0){gyY = -(mpu.getGyY()*mulMPU - gyY_ori);}

	}else{
		gyX = -(mpu.getGyX()*mulMPU - gyX_ori);
		gyY = -(mpu.getGyY()*mulMPU - gyY_ori);
		gyZ = -(mpu.getGyZ()*mulMPU - gyZ_ori);
	}

		

	//Recebe os dados mapeados do controle
	roll = -Rx_final; //Potencia de giro no proprio eixo
	upDown = Ry_final; //Potencia de subida e descida

	//Remapeia os angulos do giroscopio para potencia dos motores
	potMotorTemp[0] = map((int)gyX, -90, 90, POT_MIN, POT_MAX);
	potMotorTemp[1] = map((int)gyY, -90, 90, POT_MIN, POT_MAX);
	potMotorTemp[2] = map((int)gyX, -90, 90, POT_MAX, POT_MIN);
	potMotorTemp[3] = map((int)gyY, -90, 90, POT_MAX, POT_MIN);

	
	//Calcula a potencia dos motores
	potencia[0] = (potMotorTemp[0] + (255 - potMotorTemp[1]) + upDown + roll) / param; //Frente Esquerda
	potencia[1] = (potMotorTemp[0] + (255 - potMotorTemp[3]) + upDown - roll) / param; //Frente Direita
	potencia[2] = (potMotorTemp[2] + (255 - potMotorTemp[1]) + upDown - roll) / param; //Tras Esquerda
	potencia[3] = (potMotorTemp[2] + (255 - potMotorTemp[3]) + upDown + roll) / param; //Tras Direita

	//Estabiliza a potencia dos motores
	potMotor[0] = -pidMotor_0.Process(potencia[0]);
	potMotor[1] = -pidMotor_1.Process(potencia[1]);
	potMotor[2] = -pidMotor_2.Process(potencia[2]);
	potMotor[3] = -pidMotor_3.Process(potencia[3]);

	for(int i = 0; i < 4; i++){
		if(potMotor[i] >= POT_MAX){
			potMotor[i] = POT_MAX;
		}
	}
	
	//Verificador de botões do controle
	btnAction();

	//Atualiza a potencia dos motores
	if(!testMode){
		ServoMotor_0.write(potMotor[0]);
		ServoMotor_1.write(potMotor[1]);
		ServoMotor_2.write(potMotor[2]);
		ServoMotor_3.write(potMotor[3]);
	}else{
		/*Serial.print("M1: ");	Serial.print(potMotor[0]);
		Serial.print(" -- M2: ");	Serial.print(potMotor[1]);
		Serial.print(" -- M3: ");	Serial.print(potMotor[2]);
		Serial.print(" -- M4: ");	Serial.print(potMotor[3]);
		Serial.print(" -- X: ");	Serial.print(gyX);
		Serial.print(" -- Y: ");	Serial.print(gyY);
		Serial.print(" -- Z: ");	Serial.print(gyZ);
		//Serial.print(" -- Lx: ");	Serial.print(Lx_final);
		//Serial.print(" -- Ly: ");	Serial.print(Ly_final);
		//Serial.print(" -- Rx: ");	Serial.print(Rx_final);
		//Serial.print(" -- Ry: ");	Serial.print(Ry_final);
		//Serial.print(" -- Roll: ");	Serial.print(roll);
		//Serial.print(" -- UpDown: ");	Serial.print(upDown);
		
		if (gps.location.isValid()){
			Serial.print(" -- Lat: ");
			Serial.print(gps.location.lat(), 6);
			Serial.print(" -- Lon: ");
		    Serial.print(gps.location.lng(), 6);
		}
		if(gps.altitude.isValid()){
			Serial.print(" -- Alt: ");
			Serial.print(gps.altitude.meters(), 2);
		}*/
		//Serial.println();
	}
}


/*
 * ========================================================================================================================================================================================
 * Fim do Loop Arduino
 */



/*
 * ========================================================================================================================================================================================
 * Log de informação para o display
 */
 
void logDisplay(){
	traDataRf.stPotMotor[0] = potMotor[0];
	traDataRf.stPotMotor[1] = potMotor[1];
	traDataRf.stPotMotor[2] = potMotor[2];
	traDataRf.stPotMotor[3] = potMotor[3];	
	traDataRf.stBactery = 0;
	traDataRf.stHeight = 0;
	traDataRf.stGyZ = gyZ;
	traDataRf.stGpsLatitude = gps.location.lat();
	traDataRf.stGpsLongitude = gps.location.lng();
	
	radio.stopListening();delay(30);
	Serial.println(traDataRf.stGpsLongitude,6);
	for(int i = 0; i < 2; i++){
		radio.write(&traDataRf, sizeof(typeDataRf));delay(30);
	}
	radio.startListening();
	
}
 
/*
 * ========================================================================================================================================================================================
 * Fim do Log de informação para o display
 */
 

/*
 * ========================================================================================================================================================================================
 * Recebe dados da controle
 */
void moduleNrf(){
	int got_time = 0; //resposta do controle
	int timeReset = 0; //time para ignorar resposta do controle
	int BtnAnswer = 0; //grava o botão do controle
	//Serial.println("Escutando");
	while(true){	
		if(radio.available()>0){
			while(radio.available()){
				radio.read( &got_time, sizeof(int) ); 
				delay(20);
			}
			Serial.print("Resposta recebida ");
			Serial.println(got_time);
		}else{
			timeReset++;
		}
		if(got_time != 0 || timeReset >= 100){
			if(got_time > 0 && got_time < 17){//Margem dos botões do controle antes dos analogicos
				btn_enabled[got_time-1] = !btn_enabled[got_time-1];
			}else if(got_time > 16){
				if(got_time >= 17000 && got_time <= 17255){ //Rx
					Rx = got_time - 17000;
				}else if(got_time >= 18000 && got_time <= 18255){//Ry
					Ry = got_time - 18000;
				}else if(got_time >= 19000 && got_time <= 19255){//Lx
					Lx = got_time - 19000;
				}else{//Ly
					Ly = got_time - 20000;
				}
			}
			break;     
		}
	}
}

/*
 * ========================================================================================================================================================================================
 * Fim do Recebimento de dados do controle
 */


/*
 * ========================================================================================================================================================================================
 * Funções de botões
 */

void btnAction(){
	if(btn_enabled[0]){// Start
		
	}
	
	if(btn_enabled[1]){// Select
		
	}

	if(btn_enabled[2]){// Up
		
	}

	if(btn_enabled[3]){// Right
		
	}

	if(btn_enabled[4]){// Down
		
	}

	if(btn_enabled[5]){// Left
		
	}

	if(btn_enabled[6]){// Triangule
		
	}

	if(btn_enabled[7]){// Circle
		
	}
	
	if(btn_enabled[8]){// Cross
		potMotor[0] = potMotor[1] = potMotor[2] = potMotor[3] = POT_MIN;
	}
	
	if(btn_enabled[9]){// Square
		
	}

	if(btn_enabled[10]){// R1
		
	}

	if(btn_enabled[11]){// R2
		
	}

	if(btn_enabled[12]){// L1
		
	}

	if(btn_enabled[13]){// L2
		
	}

	if(btn_enabled[14]){// R3
		
	}

	if(btn_enabled[15]){// L3
		
	}
}

/*
 * ========================================================================================================================================================================================
 * Fim das Funções de botões
 */



 /*
 * ========================================================================================================================================================================================
 * Modulo GPS
 */
void moduleGps(){
	//Serial.println("MOdulo GPS");
	while(serial.available() > 0){
		if(gps.encode(serial.read())){
			if (gps.location.isValid()){
				gps_latitude = gps.location.lat();
				gps_longitude = gps.location.lng();
				Serial.print("LON: ");Serial.println(gps_longitude);
			}
			if(gps.speed.isValid()){
				gps_velocidade = gps.speed.kmph();
				//Serial.print("VEL: ");Serial.println(gps_velocidade);
			}
			if(gps.altitude.isValid()){
				gps_altitude = gps.altitude.value();
			}
		}
	}
		
	//Serial.println("Fim MOdulo GPS");
}

 /*
 * ========================================================================================================================================================================================
 * Fim do Modulo GPS
 */
