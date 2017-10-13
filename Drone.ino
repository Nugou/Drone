/*
 * -------------Lembrete para o que precisa ser feito ou ideias para por no drone------------------
 * (Adicionar ideias no drone caso precise)
 * 
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

#define SIZE_FILTER 10

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
int potMotor[4] = {0, 0, 0, 0}; //Potencia final dos motores
int potMotorTemp[4] = {0, 0, 0, 0}; //Potencia temporaria dos motores
int potencia[4] = {0, 0, 0, 0}; //Potencia pré-calculada dos motores

String data = ""; //Dados recebido do joystick

long timeCheckerMPU = 50; //Delay de verificaçao do MPU-6050
long timeCheckerCOM = 5; //Delay de verificaçao do Modulo de comunicação geral.(Modulo BlueTooth, Modulo RF NRF24L01)

unsigned long timeMPU = timeCheckerMPU; //Tempo para leitura do módulo MPU-6050.(Nada alterar aqui)
unsigned long timeCOM = timeCheckerCOM; //Tempo para leitura do módulo de comunicação.(Nada alterar aqui)

double motor_Kp = 1.0; //tenta aproximar do valor setpoint (padrão: 1.0) 
double motor_Ki = 0.0002; //adiciona valores para aproximar aproximação (padrão: 0.0002)
double motor_Kd = 0.0056; //Tenta manter estavel os valores (padrão: 0.0056)
double motor_Setpoint = 0.0; //Setpoint inicial do PID 

bool radioNumber = 1; //Tipo do nrf24l01

byte addresses[][6] = {"1Node","2Node"}; //endereço de comunicação dos nrf24l01

double gyX, gyY, acY; //Dados da MPU
double gyX_ori, gyY_ori, acY_ori; //Dados da MPU
double mulMPU = 0.0056; //Multiplicador para converter os dados da MPU em angulo aproximado(-90 / 90)

int Ly = 0; //Dados do analogico da esquerda eixo Y
int Lx = 0; //Dados do analogico da esquerda eixo X
int Ry = 0; //Dados do analogico da direita eixo Y
int Rx = 0; //Dados do analogico da direita eixo X

int upDown = 0;
int roll = 0;
int param = 4;

double tempGyX[SIZE_FILTER]; //Filtro da MPU do eixo X
double tempGyY[SIZE_FILTER]; //Filtro da MPU do eixo Y
/*
 * ========================================================================================================================================================================================
 * #Fim da Declaração das variaveis globais
 */

 

/*
 * ========================================================================================================================================================================================
 * #Declarando objetos
 */
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
 * Configurações Iniciais
 */

void setup() {
	mpu.begin();
	Serial.begin(115200);
	SPI.begin();
	radio.begin();
	
	/////////////////////////////////////////////////////////////////////

	ServoMotor_0.attach(pino_motor[0]);
	ServoMotor_1.attach(pino_motor[1]);
	ServoMotor_2.attach(pino_motor[2]);
	ServoMotor_3.attach(pino_motor[3]);

	/////////////////////////////////////////////////////////////////////

	if(radioNumber){
		radio.openWritingPipe(addresses[1]);
		radio.openReadingPipe(1,addresses[0]);
	}else{
		radio.openWritingPipe(addresses[0]);
		radio.openReadingPipe(1,addresses[1]);
	}

	radio.startListening();

	/////////////////////////////////////////////////

	mpu.compute();
	delay(500);
	//Calcula a posição inicial do drone para definir a diferença de inclinação
	gyX_ori = mpu.getGyX()*mulMPU;
	gyY_ori = mpu.getGyY()*mulMPU;
	acY_ori = mpu.getAcY()*mulMPU;
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
	/*Verificador do modulo MPU-6050(Giroscopio)*/
	if(timeMPU <= millis()){
		timeMPU += timeCheckerMPU;
		mpu.compute();
	}

	/*Verificador do modulo nrf24l01*/
	if(timeCOM <= millis()){
		timeCOM += timeCheckerCOM;
		moduleNrf();
	}

	Lx = map(Lx, 0, 255, -45, 45); //Remapeia os valores dos analogico para angulo
	Ly = map(Ly, 0, 255, -45, 45); //Remapeia os valores dos analogico para angulo
	Rx = map(Rx, 0, 255, -179, 179); //Remapeia os valores dos analogico para potencia de subida e descida
	Ry = map(Ry, 0, 255, -90, 90); //Remapeia os valores dos analogico para potencia de giro

	//Recebe dados da MPU e converte para angulo somando o erro que vem do controle
	gyX = mpu.getGyX()*mulMPU + Lx - gyX_ori;
	gyY = mpu.getGyY()*mulMPU + Ly - gyY_ori;
	acY = mpu.getAcY()*mulMPU - acY_ori;

	//Filtra o dados da mpu para diminuir a variação brusca de dados errados
	gyX = filterMPUx(gyX);
	gyY = filterMPUy(gyY);

	//Recebe os dados mapeados do controle
	roll = Rx; //Potencia de giro no proprio eixo
	upDown = Ry; //Potencia de subida e descida

	//Remapeia os angulos do giroscopio para potencia dos motores
	potMotorTemp[0] = map((int)gyX, -90, 90, 0, 179);
	potMotorTemp[1] = map((int)gyY, -90, 90, 0, 179);

	
	//Calcula a potencia dos motores
	potencia[0] = (potMotorTemp[0] + (potMotorTemp[1]) + upDown + roll) / param; //Frente Esquerda
	potencia[1] = (potMotorTemp[0] + (255 - potMotorTemp[1]) + upDown - roll) / param; //Frente Direita
	potencia[2] = (255 - potMotorTemp[0] + (potMotorTemp[1]) + upDown - roll) / param; //Tras Esquerda
	potencia[3] = (255 - potMotorTemp[0] + (255 - potMotorTemp[1]) + upDown + roll) / param; //Tras Direita

	//Estabiliza a potencia dos motores
	potMotor[0] = -pidMotor_0.Process(potencia[0]);
	potMotor[1] = -pidMotor_1.Process(potencia[1]);
	potMotor[2] = -pidMotor_2.Process(potencia[2]);
	potMotor[3] = -pidMotor_3.Process(potencia[3]);

	//Atualiza a potencia dos motores
	ServoMotor_0.write(potMotor[0]);
	ServoMotor_1.write(potMotor[1]);
	ServoMotor_2.write(potMotor[2]);
	ServoMotor_3.write(potMotor[3]);
}


/*
 * ========================================================================================================================================================================================
 * Fim do Loop Arduino
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
		if(radio.available()){
			while(radio.available()){
				radio.read( &got_time, sizeof(int) ); 
			}
		}else{
			timeReset++;
		}
			
		if(got_time != 0 || timeReset == 100){
			if(got_time > 0 && got_time < 17){//Margem dos botões do controle antes dos analogicos
				//Serial.print("Resposta recebida ");
				//Serial.println(got_time);
				BtnAnswer = got_time;
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
	//Serial.println("Enviando resposta");
	radio.stopListening();
	//got_time = random(1, 1000);
	radio.write(&got_time, sizeof(int));
	radio.startListening();
}

/*
 * ========================================================================================================================================================================================
 * Fim do Recebimento de dados do controle
 */

 
/*
 * ========================================================================================================================================================================================
 * Filtro da MPU
 */
//Eixo X
double filterMPUx(int value){
	for(int i = SIZE_FILTER; i > 0; i--){
		tempGyX[i] = tempGyX[i-1];
	}
	tempGyX[0] = value;
	long arrayAll = 0;
	
	for(int i = 0; i < SIZE_FILTER; i++){
		arrayAll = arrayAll + tempGyX[i];
	}
	return arrayAll/SIZE_FILTER;
}

//Eixo Y
double filterMPUy(int value){
	for(int i = SIZE_FILTER; i > 0; i--){
		tempGyY[i] = tempGyY[i-1];
	}
	tempGyY[0] = value;
	long arrayAll = 0;
	
	for(int i = 0; i < SIZE_FILTER; i++){
		arrayAll = arrayAll + tempGyY[i];
	}
	return arrayAll/SIZE_FILTER;
}
 
/*
 * ========================================================================================================================================================================================
 * Fim do Filtro da MPU
 */
