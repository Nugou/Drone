/*
 * Lembrete para o que precisa ser feito.
 * Colocar acentos nas palavras do comentario, meu teclado esta bugado. 
 * Montar o codigo de operaçao dos motores.
 * (Descontinuado)Montar o codigo do modulo RF 433Mhz.
 * (Descontinuado)Montar o codigo da Camera(Vga Ov7670). 
 * (Descontinuado)Montar o codigo do Modulo RF Lora1278(Melhor) -> Biblioteca pronta no github.
 * (Terminado)(Descontinuado)Montar o codigo do Modulo HC-05. -> Teste feitos com sucesso, mais ainda da para melhorar. (desistencia desse modulo, pois curta distancia).
 * Montar o codigo EEPROM para memorizar dados importante. 
 * (Terminado)Montar o codigo do MPU-6050. -> Esperando testes no drone.
 * Montar o codigo para o drone voltar a origem quando: estar com bateria fraca, perdeu conexao com o controle. 
 * Montar o codigo do controle de PS2.
 * Montar o codigo de funçao do drone, como dar 360°, monitoramento de area, etc.
 * Montar o codigo do modulo GPS.
 * (Terminado) Montar o codigo do modulo nrf24l01 
 */

#include <Wire.h>
#include <Servo.h>
#include <PIDX.h>
#include <SPI.h>
#include <RF24.h>

Servo ServoMotor;

int AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; //Dados do sensor MPU-6050
double AcX_End, AcY_End, AcZ_End, GyX_End, GyY_End, GyZ_End;//Dados finais do sensor MPU_6050
int MPU=0; //endereço do sensor MPU-6050
int pino_motor[4] = {A0, A1, A3, A4}; //pino de controle dos motores
int value_motor[4]; //Valor da potencia de cada motor
int power_motor = 5; //incremento/decremento da força dos motores

String data = ""; //Dados recebido do joystick

long timeCheckerMPU = 50; //Delay de verificaçao do MPU-6050
long timeCheckerCOM = 5; //Delay de verificaçao do Modulo de comunicação geral.(Modulo BlueTooth, Modulo RF NRF24L01)

unsigned long timeMPU = timeCheckerMPU; //Tempo para leitura do módulo MPU-6050.(Nada alterar aqui)
unsigned long timeCOM = timeCheckerCOM; //Tempo para leitura do módulo de comunicação.(Nada alterar aqui)

String Rdirection = "R_Center"; //Direção do joystick Direito inicial
String Ldirection = "L_Center"; //Direção do joystick Esquerdo inicial


double mpu_minPID, mpu_maxPID; //máximo e mínimo do sensor MPU-6050
double motor_minPID, motor_maxPID; //máximo e mínimo dos motores

double mpu_Kp = 0.0056; //tenta aproximar do valor setpoint (padrao: 0.0056) 
double mpu_Ki = 0.0; //adiciona valores para aproximar aproximação (padrao: 0.001)
double mpu_Kd = 0.0; //Tenta manter estavel os valores (padrao: 0.0008)

double motor_Kp = 0.002; //tenta aproximar do valor setpoint (padrão: 0.002) 
double motor_Ki = 0.0; //adiciona valores para aproximar aproximação (padrão: 0.001)
double motor_Kd = 0.0; //Tenta manter estavel os valores (padrão: 0.0008)

double mpu_Setpoint = 0.0, mpu_Input, mpu_Output; //Nada a alterar
double motor_Setpoint = 0.0, motor_Input, motor_Output; //Nada a alterar

/*
 * Declarando objetos
 */

PIDX PID_MPU(mpu_Kp, mpu_Ki, mpu_Kd, mpu_Setpoint);
PIDX PID_MOTOR(motor_Kp, motor_Ki, motor_Kd, motor_Setpoint);

RF24 radio(7,8);

/*
 * Fim
 */

int sampleTime = timeCheckerMPU; //Tempo para habilitar a leitura no modulo MPU-6050.(Nada a alterar)

bool User_Control = false;

bool radioNumber = 1;

byte addresses[][6] = {"1Node","2Node"};


void setup() {
	Wire.begin(); 
	Serial.begin(115200);
	SPI.begin();
	radio.begin();
	
	/////////////////////////////////////////////////////////////////////

	for(int i = 0; i < 4; i++){
		ServoMotor.attach(pino_motor[i]);
	}

	/////////////////////////////////////////////////////////////////////
	
	//Pesquisa o endereço do MPU-6050
	byte error, address;
	int nDevices;
	do{     
		nDevices = 0;
		for(address = 1; address < 127; address++ ){
	        // The i2c_scanner uses the return value of
	        // the Write.endTransmisstion to see if
	        // a device did acknowledge to the address.
			Wire.beginTransmission(address);
			error = Wire.endTransmission();
     
			if (error == 0){
				Serial.print("I2C device found at address 0x");
				if (address < 16){
					Serial.print("0");
				}
				Serial.print(address,HEX);
				MPU = int(address,HEX);
				Serial.println("  !");
				nDevices++;
	        }else if (error == 4){
				Serial.print("Unknown error at address 0x");
				if (address<16){
					Serial.print("0");
				}
				Serial.println(address,HEX);
			}    
			if (nDevices == 0){
				Serial.println("No I2C devices found\n");
			}else{
				Serial.println("done\n");
				delay(5000); 
			} 
		}
	}while(nDevices == 0);

	Wire.endTransmission(true);
	delay(1000);
	Wire.begin();
	Wire.beginTransmission(MPU);
	Wire.write(0x6B); 
   
	//Inicializa o MPU-6050
	Wire.write(0); 
	Wire.endTransmission(true);

	//////////////////////////////////////////////////////////
	
	mpu_minPID = -180;
	mpu_maxPID = 180;
	mpu_Setpoint = 0;

	motor_minPID = 0;
	motor_maxPID = 255;
	motor_Setpoint = 0;

	////////////////////////////////////////////////////////

	if(radioNumber){
		radio.openWritingPipe(addresses[1]);
		radio.openReadingPipe(1,addresses[0]);
	}else{
		radio.openWritingPipe(addresses[0]);
		radio.openReadingPipe(1,addresses[1]);
	}

	radio.startListening();

	/////////////////////////////////////////////////
	
}

void loop() {
	/*Verificador do modulo MPU-6050(Giroscopio)*/
	if(timeMPU <= millis()){
		timeMPU += timeCheckerMPU;
		Wire.beginTransmission(MPU);
		Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
		Wire.endTransmission(false);
		  
		//Solicita os dados do sensor
		Wire.requestFrom(MPU,14,true);  
		 
		//Armazena o valor dos sensores nas variaveis correspondentes
		GyX=Wire.read()<<8|Wire.read(); //0x3B (GYRO_XOUT_H) & 0x3C (GYRO_XOUT_L)     
		GyY=Wire.read()<<8|Wire.read(); //0x3D (GYRO_YOUT_H) & 0x3E (GYRO_YOUT_L)
		GyZ=Wire.read()<<8|Wire.read(); //0x3F (GYRO_ZOUT_H) & 0x40 (GYRO_ZOUT_L)
		Tmp=Wire.read()<<8|Wire.read(); //0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
		AcX=Wire.read()<<8|Wire.read(); //0x43 (ACCEL_XOUT_H) & 0x44 (ACCEL_XOUT_L)
		AcY=Wire.read()<<8|Wire.read(); //0x45 (ACCEL_YOUT_H) & 0x46 (ACCEL_YOUT_L)
		AcZ=Wire.read()<<8|Wire.read(); //0x47 (ACCEL_ZOUT_H) & 0x48 (ACCEL_ZOUT_L)

		GyX_End = calculatePID_mpu(GyX);
		GyY_End = calculatePID_mpu(GyY);
		GyZ_End = calculatePID_mpu(GyZ);
	
		AcX_End = calculatePID_mpu(AcX);
		AcY_End = calculatePID_mpu(AcY);
		AcZ_End = calculatePID_mpu(AcZ);

		if(!User_Control){
			if(GyX_End > -3 && GyX_End < 3){
				//Drone esta estabilizado no eixo X
			}else{
				//Comando para estabilizar o drone no eixo X
				
			}
			if(GyY_End > -3 && GyY_End < 3){
				//Drone esta estabilizado no eixo Y
			}else{
				//Comando para estabilizar o drone no eixo Y
			}
			/*Atualmente nao necessario o eixo z 
			if(GyZ_End < -30 || GyZ_End > 30){
				
			}else{
				
			}*/
			if(AcY_End == 0){
				//Drone esta parado no ar, codigo para mante-lo no ar com uma potencia nos motores estaveis.	
			}
		}
	}

	/*Verificador do modulo nrf24l01*/
	if(timeCOM <= millis()){
		timeCOM += timeCheckerCOM;
		moduleNrf();
	}

	if((Rdirection == "R_Center") && (Ldirection == "L_Center")){
		//O drone fica parado no ar
		User_Control = false;
	}else{
		User_Control = true;
	}
	if((Rdirection == "R_Center") && (Ldirection == "L_Up")){
		//O drone vai pra cima
		for(int i = 0; i < 4; i++){
			if(value_motor[i] != 179){
				value_motor[i] += power_motor;
				if(value_motor[i] > 179){
					value_motor[i] = 179;
				}
			}
		}
	}
	if((Rdirection == "R_Center") && (Ldirection == "L_U_Right")){
		//O drone vai para cima girando para o sentido horario
	}
	if((Rdirection == "R_Center") && (Ldirection == "L_Right")){
		//O drone gira no sentido horario
	}
	if((Rdirection == "R_Center") && (Ldirection == "L_D_Right")){
		//O drone vai para baixo girando no sentido horario
	}
	if((Rdirection == "R_Center") && (Ldirection == "L_Down")){
		//O drone vai para baixo
	}
	if((Rdirection == "R_Center") && (Ldirection == "L_D_Left")){
		//O drone vai para baixo girando no sentido anti-horario
	}
	if((Rdirection == "R_Center") && (Ldirection == "L_Left")){
		//O drone gira para o sentido anti-horario
	}
	if((Rdirection == "R_Center") && (Ldirection == "L_U_Left")){
		//O drone vai para cima girando no sentido anti-horario
	}

	///////////////////////////////////////////////////////////

	if((Rdirection == "R_Up") && (Ldirection == "L_Center")){
		//O drone vai para frente
	}
	if((Rdirection == "R_Up") && (Ldirection == "L_Up")){
		//O drone vai para frente e subindo
	}
	/*
	if((Rdirection == "R_Up") && (Ldirection == "L_U_Right")){
		//O drone vai para frente, girando para a direita, e subindo
	}
	if((Rdirection == "R_Up") && (Ldirection == "L_Right")){
		//O drone vai para frente, girando para a direita
	}
	if((Rdirection == "R_Up") && (Ldirection == "L_D_Right")){
		//O drone vai para frente, girando para a direita, e descendo
	}*/
	if((Rdirection == "R_Up") && (Ldirection == "L_Down")){
		//O drone vai para frente e descendo
	}
	/*
	if((Rdirection == "R_Up") && (Ldirection == "L_D_Left")){
		//O drone vai para frente, girando para a esquerda, e descendo
	}
	if((Rdirection == "R_Up") && (Ldirection == "L_Left")){
		//O drone vai para frente girando para a esquerda
	}
	if((Rdirection == "R_Up") && (Ldirection == "L_U_Left")){
		//O drone vai para frente, girando para a esquerda, e subindo
	}*/
	
	///////////////////////////////////////////////////////////

	if((Rdirection == "R_U_Right") && (Ldirection == "L_Center")){
		//O drone vai para frente e direita
	}
	if((Rdirection == "R_U_Right") && (Ldirection == "L_Up")){
		//O drone vai para frente e direita, subindo
	}
	/*
	if((Rdirection == "R_U_Right") && (Ldirection == "L_U_Right")){
		//O drone vai para frente e direita, girando para a direita, e subindo
	}
	if((Rdirection == "R_U_Right") && (Ldirection == "L_Right")){
		
	}
	if((Rdirection == "R_U_Right") && (Ldirection == "L_D_Right")){
		
	}*/
	if((Rdirection == "R_U_Right") && (Ldirection == "L_Down")){
		//O drone vai para frente e direita, descendo
	}
	/*
	if((Rdirection == "R_U_Right") && (Ldirection == "L_D_Left")){
		
	}
	if((Rdirection == "R_U_Right") && (Ldirection == "L_Left")){
		
	}
	if((Rdirection == "R_U_Right") && (Ldirection == "L_U_Left")){
		
	}*/

	///////////////////////////////////////////////////////////

	if((Rdirection == "R_Right") && (Ldirection == "L_Center")){
		//O drone vai para direita
	}
	if((Rdirection == "R_Right") && (Ldirection == "L_Up")){
		//O drone vai para direita e subindo
	}
	/*
	if((Rdirection == "R_Right") && (Ldirection == "L_U_Right")){
		
	}
	if((Rdirection == "R_Right") && (Ldirection == "L_Right")){
		
	}
	if((Rdirection == "R_Right") && (Ldirection == "L_D_Right")){
		
	}*/
	if((Rdirection == "R_Right") && (Ldirection == "L_Down")){
		//O drone vai para direita e descendo
	}
	/*
	if((Rdirection == "R_Right") && (Ldirection == "L_D_Left")){
		
	}
	if((Rdirection == "R_Right") && (Ldirection == "L_Left")){
		
	}
	if((Rdirection == "R_Right") && (Ldirection == "L_U_Left")){
		
	}*/

	///////////////////////////////////////////////////////////

	if((Rdirection == "R_D_Right") && (Ldirection == "L_Center")){
		//O drone vai para tras e direita
	}
	if((Rdirection == "R_D_Right") && (Ldirection == "L_Up")){
		//O drone vai para tras e direita, subindo
	}
	/*
	if((Rdirection == "R_D_Right") && (Ldirection == "L_U_Right")){
		
	}
	if((Rdirection == "R_D_Right") && (Ldirection == "L_Right")){
		
	}
	if((Rdirection == "R_D_Right") && (Ldirection == "L_D_Right")){
		
	}*/
	if((Rdirection == "R_D_Right") && (Ldirection == "L_Down")){
		//O drone vai para tras e direita, descendo
	}
	/*
	if((Rdirection == "R_D_Right") && (Ldirection == "L_D_Left")){
		
	}
	if((Rdirection == "R_D_Right") && (Ldirection == "L_Left")){
		
	}
	if((Rdirection == "R_D_Right") && (Ldirection == "L_U_Left")){
		
	}*/

	///////////////////////////////////////////////////////////

	if((Rdirection == "R_Down") && (Ldirection == "L_Center")){
		//O drone vai para tras
	}
	if((Rdirection == "R_Down") && (Ldirection == "L_Up")){
		//O drone vai para tras e subindo
	}
	/*
	if((Rdirection == "R_Down") && (Ldirection == "L_U_Right")){
		
	}
	if((Rdirection == "R_Down") && (Ldirection == "L_Right")){
		
	}
	if((Rdirection == "R_Down") && (Ldirection == "L_D_Right")){
		
	}*/
	if((Rdirection == "R_Down") && (Ldirection == "L_Down")){
		//O drone vai para tras e descendo
	}/*
	if((Rdirection == "R_Down") && (Ldirection == "L_D_Left")){
		
	}
	if((Rdirection == "R_Down") && (Ldirection == "L_Left")){
		
	}
	if((Rdirection == "R_Down") && (Ldirection == "L_U_Left")){
		
	}*/

	///////////////////////////////////////////////////////////

	if((Rdirection == "R_D_Left") && (Ldirection == "L_Center")){
		//O drone vai para tras e esquerda
	}
	if((Rdirection == "R_D_Left") && (Ldirection == "L_Up")){
		//O drone vai para tras e esquerda, subindo
	}
	/*
	if((Rdirection == "R_D_Left") && (Ldirection == "L_U_Right")){
		
	}
	if((Rdirection == "R_D_Left") && (Ldirection == "L_Right")){
		
	}
	if((Rdirection == "R_D_Left") && (Ldirection == "L_D_Right")){
		
	}*/
	if((Rdirection == "R_D_Left") && (Ldirection == "L_Down")){
		//O drone vai para tras e esquerda, descendo
	}
	/*
	if((Rdirection == "R_D_Left") && (Ldirection == "L_D_Left")){
		
	}
	if((Rdirection == "R_D_Left") && (Ldirection == "L_Left")){
		
	}
	if((Rdirection == "R_D_Left") && (Ldirection == "L_U_Left")){
		
	}*/

	///////////////////////////////////////////////////////////

	if((Rdirection == "R_Left") && (Ldirection == "L_Center")){
		//O drone vai para esquerda
	}
	if((Rdirection == "R_Left") && (Ldirection == "L_Up")){
		//O drone vai para esquerda e subindo
	}
	/*
	if((Rdirection == "R_Left") && (Ldirection == "L_U_Right")){
		
	}
	if((Rdirection == "R_Left") && (Ldirection == "L_Right")){
		
	}
	if((Rdirection == "R_Left") && (Ldirection == "L_D_Right")){
		
	}*/
	if((Rdirection == "R_Left") && (Ldirection == "L_Down")){
		//O drone vai para esquerda e descendo
	}
	/*
	if((Rdirection == "R_Left") && (Ldirection == "L_D_Left")){
		
	}
	if((Rdirection == "R_Left") && (Ldirection == "L_Left")){
		
	}
	if((Rdirection == "R_Left") && (Ldirection == "L_U_Left")){
		
	}*/

	///////////////////////////////////////////////////////////

	if((Rdirection == "R_U_Left") && (Ldirection == "L_Center")){
		//O drone vai para frente e esquerda
	}
	if((Rdirection == "R_U_Left") && (Ldirection == "L_Up")){
		//O drone vai para frente e esquerda, subindo
	}
	/*
	if((Rdirection == "R_U_Left") && (Ldirection == "L_U_Right")){
		
	}
	if((Rdirection == "R_U_Left") && (Ldirection == "L_Right")){
		
	}
	if((Rdirection == "R_U_Left") && (Ldirection == "L_D_Right")){
		
	}*/
	if((Rdirection == "R_U_Left") && (Ldirection == "L_Down")){
		//O drone vai para frente e esquerda, descendo
	}
	/*
	if((Rdirection == "R_U_Left") && (Ldirection == "L_D_Left")){
		
	}
	if((Rdirection == "R_U_Left") && (Ldirection == "L_Left")){
		
	}
	if((Rdirection == "R_U_Left") && (Ldirection == "L_U_Left")){
		
	}*/
}

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
					int Rx = got_time - 17000;
				}else if(got_time >= 18000 && got_time <= 18255){//Ry
					int Ry = got_time - 18000;
				}else if(got_time >= 19000 && got_time <= 19255){//Lx
					int Lx = got_time - 19000;
				}else{//Ly
					int Ly = got_time - 20000;
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

double calculatePID_mpu(int varInt){
	mpu_Input = varInt;
	delay(sampleTime+5); //minimo 100 (para evitar erros nos calculos)
	PID_MPU.setOutputLimits(mpu_minPID, mpu_maxPID); 
	mpu_Output = PID_MPU.Process(mpu_Input);
	return mpu_Output;
}

double calculatePID_motor(int varInt){
	motor_Input = varInt;
	delay(sampleTime+5); //minimo 100 (para evitar erros nos calculos)
	PID_MOTOR.setOutputLimits(motor_minPID, motor_maxPID); 
	motor_Output = PID_MOTOR.Process(motor_Input);
	return motor_Output;
}

void autoGyY(){ //Controle autonomo de manter o drone estavel no eixo Y do gyroscopio
	
}

void autoGyX(){ //Controle autonomo de manter o drone estavel no eixo X do gyroscopio
	
}

void autoAcY(){ //Controle autonomo de manter o drone estavel no eixo Y do acelerometro
	
}




