/*
 * Sprint 8 - Max Pereira do Nascimento.c
 *
 * Created: 03/05/2021 00:47:13
 * Author : Max Nascimento
 */ 

//Definições
#define F_CPU 16000000UL
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1
//Bibliotecas
#include <avr/io.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include "nokia5110.h"

//Variáveis
volatile char vetor_Transmissao[] = {' ',' ',' ',' ',' ',' ',' ',' ',' '};
volatile char flag_Transmissao[] = {'0'};
volatile char recebido;
volatile char pressao_monitor[8] = {'0','0','0','x','0','0','0'};
//Variáveis
uint8_t flag200ms = 0;
uint8_t ciclo = 1;
uint8_t flag_respirador = 0;
uint8_t flagADMUX = 0;
uint8_t flag_Error = 1;
uint8_t FreqRespiracao = 5;
uint8_t cont_Vetor = 0;
uint8_t volAr = 8;
uint8_t seletor = 0;
uint8_t seletor2 = 1;
uint8_t bip = 0;
uint16_t FreqCardiaca = 0;
uint16_t T = 0;
uint16_t o2 = 0;
uint16_t ph = 0;
uint16_t oxigenio_servo = 0;
uint32_t o2_motor = 0;
uint32_t cont = 0;
uint32_t tempo_ms = 0;
uint32_t tempo_bip = 0;
uint32_t tempo_transmissao = 0;


//Funções
void atualizarDisplay(uint8_t *flag_disparo, uint8_t *flag_Error2);		//Função para configuração do display
void limparTransmissao();												//Usado para limpar o veotr de transmissão usado no USART


//Configura USART  utilizado para medição de pressão
ISR(USART_RX_vect){
	recebido = UDR0;												//UDR0 contém o dado recebido via USART
	if (recebido == ';'){											//Abre a transmissão
		flag_Transmissao[0] = '1';
	}
	if((tempo_transmissao <= 5000)&&(flag_Transmissao[0]=='1')){	//Continua a transmissão
		vetor_Transmissao[cont_Vetor] = recebido;
		cont_Vetor++;
		if ((cont_Vetor == 9)||(recebido ==':')){					//Verificação do formato do envio
			if((vetor_Transmissao[8]==':')&&(vetor_Transmissao[4]='x')&&(vetor_Transmissao[0] == ';')){
				flag_Error = 0;
				for (uint8_t i= 0; i<=8;i++){
					USART_Transmit(vetor_Transmissao[i]);
				}
				pressao_monitor[0] = vetor_Transmissao[1];
				pressao_monitor[1] = vetor_Transmissao[2];
				pressao_monitor[2] = vetor_Transmissao[3];
				pressao_monitor[4] = vetor_Transmissao[5];
				pressao_monitor[5] = vetor_Transmissao[6];
				pressao_monitor[6] = vetor_Transmissao[7];
				limparTransmissao();
			}
			else{
				flag_Error = 1;
				limparTransmissao();
			}
			flag_Transmissao[0] = '0';
			cont_Vetor = 0;
		}
		else{														//Avisa sobre erro no formato
			flag_Error = 1;
		}
	}
	else{															//Avisa sobre erro no tempo de transmissão ou transmissão fechada
		flag_Error = 1;
	}
}

//Mudança de estados (botão +)
ISR(INT0_vect){
	if (seletor == 4){
		if(FreqRespiracao < 30){
			FreqRespiracao =  FreqRespiracao + 1;
		}
	}
	if (seletor == 6){
		int var = o2_motor;
		if (var<100){
		OCR1B= OCR1B + 200;
			if (OCR1B < 2000){
				OCR1B = 2000;
			}
			o2_motor = (OCR1B-2000)/20;
		}
	}
	if (seletor == 8){
		if (volAr < 8){
			volAr ++;
		}
	}
}

//Mudança de estados (botão -)
ISR(INT1_vect){
	if (seletor == 4){
		if (FreqRespiracao > 5){
			FreqRespiracao =  FreqRespiracao - 1;
		}
	}
	if (seletor == 6){
		int var = o2_motor;
		if (var>0){
			OCR1B = OCR1B - 200;
			if (OCR1B < 2000){
				OCR1B = 2000;
			}
			o2_motor = (OCR1B-2000)/20;
		}
	}
	if (seletor == 8){
		if (volAr > 1){
			volAr --;
		}
	}
}

//PCINT para seletor
ISR(PCINT0_vect){
	seletor = seletor + 1;
	if (seletor == 10){
		seletor = 0;
	}
}

//PCINT para freq cardíaca
ISR(PCINT2_vect){
	static uint32_t freqCardAnterior_ms = 0;
	FreqCardiaca = 60000/((tempo_ms - freqCardAnterior_ms)*2);
	freqCardAnterior_ms = tempo_ms;
}

//Interrupção ADC
ISR(ADC_vect){
	//Medições
	//02
	if(flagADMUX == 0){
		ADMUX = 0b01000000;
		o2 = (((ADC*10*5/1023))/(0.04))/10;	//Medição O2
	}
	//Temperatura
	if(flagADMUX == 1){
		ADMUX = 0b01000001;
		T = ((ADC*10*5/1023)) + 10;			//Medição Temperatura
	}
	//pH
	if (flagADMUX == 2){
		ADMUX = 0b01000010;
		ph = ((ADC*100*5/1023)) + 700;		//Medição pH
		if ((ph >=735)&&(ph<=745)){			//Normal
			PORTD  = 0b00111101;
		}
		if (ph<735){						//Acidose
			PORTD  = 0b00011111;
		}
		if (ph>745){						//Alcalose
			PORTD  = 0b01011101;
		}
	}
	
	//Ativar sinal sonoro
	if(((T >= 35)&&(T <= 41))||(o2<=60)){
		PORTD ^= 0b10000000;				
	}
}

//Timer
ISR(TIMER0_COMPA_vect){

	tempo_ms++;		//Tempo do sistema (em ms)
	
	//verifica tempo de transmissão
	if(flag_Transmissao[0] == '1'){
		tempo_transmissao ++;
		if(tempo_transmissao == 5000){
			tempo_transmissao = 0;
			flag_Transmissao[0] = '0';
			flag_Error = 1;
			limparTransmissao();
		}
	}
	
	//Controle ADC
	if((tempo_ms % 75)==0){					//Alternador do ADC (FlagADMUX)
		flagADMUX = flagADMUX + 1;
		if (flagADMUX == 3){
			flagADMUX = 0;
		}
	}
	
	//Ativação 200ms
	if((tempo_ms % 200)==0){				//Flag 200ms
		flag200ms = 1;
	}
	
	//Sinal sonoro
	if (bip == 1){
		tempo_bip++;
		if (tempo_bip >=2000){
			PORTD ^= 0b10000000;
			bip = 0;
		}
	}
	
	//Controle do respirador
	if(volAr > 0){							//Controlador da respiração
		float f2 = 0.125*volAr;
		int f3 = f2*FreqRespiracao;
		if ((tempo_ms %  (3750/f3))==0){	//Descida do servo
			if (ciclo==0){
				OCR1A = OCR1A + 266;
				cont ++;
			
				if (OCR1A > 4000){
					ciclo = 1;
					OCR1A = 4000;
				}
				if (cont >= volAr){
					ciclo = 1;
				}
			}
			else if (ciclo == 1){			//Subida do servo
				OCR1A = OCR1A - 266;
				cont = 0;
				if (OCR1A < 2000){
					ciclo = 0;
					OCR1A = 2000;
					if(OCR1A == 2000){
						bip = 1;
						PORTD ^= 0b10000000;
						tempo_bip = 0;
					}
				
				}
			}
		}
	}
}

//Transmissão USART
void USART_Transmit(unsigned char data){
	while(!( UCSR0A & (1<<UDRE0)));//Espera a limpeza do registr. de transmissão
	UDR0 = data; //Coloca o dado no registrador e o envia
}

//Recepção USART
unsigned char USART_Receive(void){
	while(!(UCSR0A & (1<<RXC0)));	//Espera o dado
	return UDR0;					//Retorna o dado recebido
}

//Usado para limpar o veotr de transmissão usado no USART
void limparTransmissao(){
	for (uint8_t j= 0; j<=9;j++){    //Limpa o vetor de trnsmissão
		vetor_Transmissao[j] = '0';
	}
}

//Função para configuração do display
void atualizarDisplay(uint8_t *flag_disparo, uint8_t *flag_Error2){    //Função de configuração do display
	
	//Variáveis que serão usadas no display
	unsigned char FreqRespiracao_string[3];
	unsigned char FreqCardiaca_string[4];
	unsigned char Oxigenio_string[4];
	unsigned char Temperatura_string[10];
	unsigned char o2_val[20];
	unsigned char xxx[4];
	uint8_t max = o2_motor;
	unsigned char stringVolAR[5];
	unsigned char stringPH[4];
	
	if(*flag_disparo){
		//Transformação das variáveis para exibição no display
		itoa(FreqRespiracao,FreqRespiracao_string,10);
		itoa(FreqCardiaca,FreqCardiaca_string,10);
		itoa(T, Temperatura_string,10);
		itoa(o2, Oxigenio_string,10);
		itoa(max, o2_val,10);
		itoa(volAr, stringVolAR, 10);
		itoa(ph, stringPH, 10);
		nokia_lcd_clear();
		
		if (seletor == 0){									//Primeira página de funções vitais
			nokia_lcd_clear();
			nokia_lcd_set_cursor(5,1);
			nokia_lcd_write_string("Sinais Vitais",1);
			nokia_lcd_set_cursor(5,10);
			nokia_lcd_write_string(FreqCardiaca_string,1);
			nokia_lcd_set_cursor(35,10);
			nokia_lcd_write_string(" bpm",1);
			nokia_lcd_set_cursor(5, 20);
			nokia_lcd_write_string(Oxigenio_string, 1);
			nokia_lcd_set_cursor(35, 20);
			nokia_lcd_write_string(" % SpO2", 1);
			nokia_lcd_set_cursor(5, 30);
			nokia_lcd_write_string(Temperatura_string, 1);
			nokia_lcd_set_cursor(35, 30);
			nokia_lcd_write_string (" oC", 1);
			nokia_lcd_set_cursor(5, 40);
			if (*flag_Error2 == 0){
				nokia_lcd_write_string(pressao_monitor, 1);
			}
			if(*flag_Error2 == 1){
				nokia_lcd_write_string("Error", 1);
			}
			nokia_lcd_set_cursor(50, 40);
			nokia_lcd_write_string (" mmHg", 1);
			nokia_lcd_render();
		}
		
		if (seletor == 2){									//Segunda página de funções vitais
			nokia_lcd_clear();
			nokia_lcd_set_cursor(5,1);
			nokia_lcd_write_string("Sinais Vitais",1);
			nokia_lcd_set_cursor(5,15);
			nokia_lcd_write_string(stringPH,1);
			nokia_lcd_set_cursor(31,15);
			nokia_lcd_write_string(" pH *100",1);
			nokia_lcd_render();
		}
		
		if (seletor == 4){									//Seletor de parâmetros (config. frequencia de respiração)
			nokia_lcd_clear();
			nokia_lcd_set_cursor(5,1);
			nokia_lcd_write_string("Parametros",1);
			nokia_lcd_set_cursor(5,10);
			nokia_lcd_write_string("-----------",1);
			nokia_lcd_set_cursor(5,15);
			nokia_lcd_write_string(FreqRespiracao_string,1);
			nokia_lcd_set_cursor(22,15);
			nokia_lcd_write_string("* resp/min",1);
			nokia_lcd_set_cursor(5, 25);
			nokia_lcd_write_string(o2_val, 1);
			nokia_lcd_set_cursor(25, 25);
			nokia_lcd_write_string(" %O2", 1);
			nokia_lcd_set_cursor(5, 35);
			nokia_lcd_write_string(stringVolAR, 1);
			nokia_lcd_set_cursor(25, 35);
			nokia_lcd_write_string(" vol", 1);
			nokia_lcd_render();
		}
		
		if (seletor == 6){									//Seletor de parâmetros (config. O2)
			nokia_lcd_clear();
			nokia_lcd_set_cursor(5,1);
			nokia_lcd_write_string("Parametros",1);
			nokia_lcd_set_cursor(5,10);
			nokia_lcd_write_string("-----------",1);
			nokia_lcd_set_cursor(5,15);
			nokia_lcd_write_string(FreqRespiracao_string,1);
			nokia_lcd_set_cursor(22,15);
			nokia_lcd_write_string(" resp/min",1);
			nokia_lcd_set_cursor(5, 25);
			nokia_lcd_write_string(o2_val, 1);
			nokia_lcd_set_cursor(25, 25);
			nokia_lcd_write_string("* %O2", 1);
			nokia_lcd_set_cursor(5, 35);
			nokia_lcd_write_string(stringVolAR, 1);
			nokia_lcd_set_cursor(25, 35);
			nokia_lcd_write_string(" vol", 1);
			nokia_lcd_render();
		}
		
		if (seletor == 8){									//Seletor de parâmetros (config. Volume de ar)
			nokia_lcd_clear();
			nokia_lcd_set_cursor(5,1);
			nokia_lcd_write_string("Parametros",1);
			nokia_lcd_set_cursor(5,10);
			nokia_lcd_write_string("-----------",1);
			nokia_lcd_set_cursor(5,15);
			nokia_lcd_write_string(FreqRespiracao_string,1);
			nokia_lcd_set_cursor(22,15);
			nokia_lcd_write_string(" resp/min",1);
			nokia_lcd_set_cursor(5, 25);
			nokia_lcd_write_string(o2_val, 1);
			nokia_lcd_set_cursor(25, 25);
			nokia_lcd_write_string(" %O2", 1);
			nokia_lcd_set_cursor(5, 35);
			nokia_lcd_write_string(stringVolAR, 1);
			nokia_lcd_set_cursor(25, 35);
			nokia_lcd_write_string("* vol", 1);
			nokia_lcd_render();
		}
		*flag_disparo = 0;
	}
}

//Função Principal
void main(void){
	//GPIO
	DDRD   = 0b11100011;	//Definição E/S porta D
	DDRC   = 0b00111111;	//Definição E/S porta C
	DDRB   = 0b10000110;	//Definição E/S porta B
	PORTB  = 0b00000001;	//Resistores de pull-up porta B
	PORTD  = 0b00011101;	//Resistores de pull-up porta D
	
	//Interrupções por mudanças de estado
	EICRA  = 0b00001010;    //interrupções externas INT0 e INT1 na borda de descida
	EIMSK  = 0b00000011;    //Habilitada as interrupções INT0 e INT1
	PCICR  = 0b00000111;
	PCMSK0 = 0b10000001;
	PCMSK2 = 0b00010010;    //habilita interrupção individual D4 e D1
	
	//ADC
	ADMUX  = 0b01000000;	//Tensão de ref , canal 0
	ADCSRA = 0b11101111;	//habilita o AD, habilita interrupção, modo de conversão contínua, prescaler = 128
	ADCSRB = 0x00;			//modo de conversão contínua
	DIDR0  = 0b00111000;	//habilita pinos PC0, PC1 e PC2 como entradas do ADC0
	
	//Timer 0
	TCCR0A = 0b00000010;
	TCCR0B = 0b00000011;
	OCR0A  = 249;			//Interrupção em 1ms
	TIMSK0 = 0b00010010;
	
	//Transmissão
	UBRR0H = (unsigned char)(MYUBRR>>8);		//Ajusta a taxa de transmissão
	UBRR0L = (unsigned char)MYUBRR;
	UCSR0B = (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0); //Habilita o transmissor e o receptor
	UCSR0C = (0<<USBS0)|(3<<UCSZ00);			//Ajusta o formato do frame: 8 bits de dados e 1 de parada
	
	//PWM
	ICR1   = 39999;       //Período do PWM = 20ms
	TCCR1A = 0b10100010;
	TCCR1B = 0b00011010;  //Prescaler = 8
	
	OCR1A  = 2000; // 1ms
	OCR1B  = 2000; // 2ms


	sei();	//Habilita interrupções globais, ativando o bit I do SREG

	nokia_lcd_init();  //inicia LCD
	
	//Loop
	while (1){
		atualizarDisplay(&flag200ms, &flag_Error);
	}
}
