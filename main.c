/*
 * GccApplication1.c
 *
 * Created: 17/12/2021 10:38:14
 * Author : Anderson
 */ 

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/eeprom.h>
#include "LCD/SSD1306.h"
#include "LCD/Font5x8.h"

#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1
#define endereco_diametro 4
#define endereco_distancia 0
#define endereco_temperatura 10

int contador = 0, pneu = 0, tempo100us = 0, tempo_100us_anterior = 0, distancia_anterior = 0;
int diametro = 50, pedal = 0, tempo_subida = 0, delta = 0, distancia_objeto = 0;
int velocidade = 0, distancia = 0, RPM = 0, delay = 0, tempo_total = 0;
double tensao_RTD = 0, RTD = 0, temperatura = 0, tensao_bateria = 0, bateria = 0, tensao_LDR = 0, lux = 0;
char marcha = 'D', farol = 'D';

ISR(PCINT2_vect) //Interrupção externa INT0
{
	if(contador == 0) //Borda de descida
	{
		pneu++; //Contagem de voltas do pneu
		tempo_total = tempo100us - tempo_100us_anterior;
		tempo_100us_anterior = tempo100us;
		RPM = 600000 / tempo_total;
		velocidade = (1130.94 * diametro) / tempo_total;
		distancia = (3.1415 * diametro / 100000) * pneu;
		if(distancia  > distancia_anterior)
		{
			eeprom_write_dword(endereco_distancia, (uint32_t)distancia); //Gravação na EEPROM da distancia percorrida no endereço 1
			distancia_anterior = distancia;
		}
		contador++;
	}
	else
		contador--; //Se o contador for diferente de 0, a variavel sofre um decremento para a condição do if ser satisfeita, capturando sempre a borda de descida
	
	if(!(PIND & (1<<4)) && (diametro < 100)) //Botão + do diametro do pneu na borda de descida
	{
		diametro++;
		_delay_ms(100);
		eeprom_write_byte(endereco_diametro, diametro); //Gravação na EEPROM do diametro no endereço 0
	}
	if(!(PIND & (1<<5)) && (diametro > 1)) //Botão - do diametro do pneu na borda de descida
	{
		diametro--;
		_delay_ms(100);
		eeprom_write_byte(endereco_diametro, diametro); //Gravação na EEPROM do diametro no endereço 0
	}
	if(!(PIND & (1<<6))) //Escolha dos modos de operação D, R e P
		marcha = 'D';
	else
		marcha = 'R';
	
	if(!(PIND & (1<<7)))
		marcha = 'P';
}

ISR(TIMER0_COMPA_vect) //Interrupção do TC0 a cada 100us
{
	tempo100us++;
}

ISR(TIMER1_CAPT_vect) //Interrupção do TC1
{
	if(TCCR1B & (1<<ICES1)) //lê o valor de contagem da borda de subida do sinal
		tempo_subida = ICR1; //salva essa primeira contagem
	else  //lê o valor de contagem da borda de descida do sinal
		delta = (ICR1 - tempo_subida)*16; //largura do pulso
	
	TCCR1B ^= (1<<ICES1); //inverte a borda de captura
}

ISR(ADC_vect){
	
	//Conversão do sinal analogico para digital
	switch(delay){
		case 1:
			ADMUX = 0b01000000; //Realoca a leitura do ADC para o PC0
			if(((velocidade > 20) && (distancia_objeto > 300)) || ((velocidade < 20) && (distancia_objeto < 300)) || ((velocidade < 20) && (distancia_objeto > 300)))
			{
				pedal = ADC; //Guarda valor do ADC 0-1023 na variavel pedal
				OCR2B = pedal / 4; //Conversão do ADC de 0-1023 para 0-255
			}
		break;
		
		case 2:
			ADMUX = 0b01000001; //Realoca a leitura do ADC para o PC1
			tensao_bateria = ADC; //Variavél irá armazenar o valor do registrador ADC - PC1
		break;
		
		case 3:
			ADMUX = 0b01000010; //Realoca a leitura do ADC para o PC2
			tensao_RTD = ADC; //Variavél irá armazenar o valor do registrador ADC - PC2
		break;
		
		case 4:
			ADMUX = 0b01000011; //Realoca a leitura do ADC para o PC3
			tensao_LDR = ADC;
			delay = 0;
		break;
	}
	delay++;
}

ISR(USART_RX_vect)
{
	char recebido;
	recebido = UDR0;
	
	if(recebido == 'd') //Comando para verificar a temperatura e salvar na EEPROM
		USART_Transmit((unsigned char)temperatura); //Valor da temperatura é mostrado na USART
		eeprom_write_byte(endereco_temperatura, temperatura);

	if(recebido == 'l') //Comando para limpar a temperatura da EEPROM
		eeprom_write_byte(endereco_temperatura, 0xFF);
}

void USART_Transmit(unsigned char data)
{
	while(!(UCSR0A & (1<<UDRE0)));//Espera a limpeza do registr. de transmissão
	UDR0 = data; //Coloca o dado no registrador e o envia
}

unsigned char USART_Receive(void)
{
	while(!(UCSR0A & (1<<RXC0))); //Espera o dado ser recebido
	return UDR0; //Lê o dado recebido e retorna
}

void LCD(int diametro, char marcha, int RPM, char farol, int distancia, double temperatura, double bateria, int distancia_objeto)
{
	lux = (5000 / ((tensao_LDR * 5) / 1023)) - 1000; //calculo da quantidade de luz
	bateria = ((tensao_bateria * 5) / 1023) * 20; //cálculo da porcentagem da bateria
	RTD = (((tensao_RTD * 5) / 1023) * 1000) / (5 - ((tensao_RTD * 5) / 1023)); //cálculo da resistência do RTD para achar a temperatura correspondente
	temperatura = (100 * RTD - 10000) / 38.5; //cálculo da temperatura da bateria
	
	if(lux < 300)
		farol = 'L';
	else
		farol = 'D';
	
	GLCD_Clear();
	GLCD_GotoXY(30, 0);
	GLCD_PrintString("Comp. Bordo");
	GLCD_GotoXY(0, 10);
	GLCD_PrintString("---------------------");
	GLCD_GotoXY(0, 20);
	GLCD_PrintString("Diam(cm): ");
	GLCD_PrintInteger(diametro);
	GLCD_GotoXY(80, 20);
	GLCD_PrintString("M: ");
	GLCD_PrintChar(marcha);
	GLCD_GotoXY(0, 35);
	GLCD_PrintInteger(RPM);
	GLCD_PrintString("rpm");
	GLCD_GotoXY(45, 35);
	GLCD_PrintString("f: ");
	GLCD_PrintChar(farol);
	GLCD_GotoXY(80, 35);
	GLCD_PrintInteger(distancia);
	GLCD_PrintString("km");
	GLCD_GotoXY(80, 50);
	GLCD_PrintInteger(temperatura);
	GLCD_PrintString(" C");
	GLCD_GotoXY(50, 50);
	GLCD_PrintInteger(bateria);
	GLCD_PrintString(" %");
	GLCD_GotoXY(0, 50);
	GLCD_PrintInteger(distancia_objeto);
	GLCD_PrintString("cm");
	GLCD_Render();
}

void display(int velocidade)
{
	static int velocidade_anterior = 0;
	
	for (int i = 0; i < (1000 - velocidade_anterior); i++)
	{
		PORTB |= 0b00001000;
		_delay_ms(0.01);
		PORTB &= 0b11110111;
	}
	for (int i = 0; i < velocidade; i++)
	{
		PORTB |= 0b00001000;
		_delay_ms(0.01);
		PORTB &= 0b11110111;
	}
	velocidade_anterior = velocidade;
	_delay_ms(500);
}

void led_farol(char marcha, double lux)
{
	if(marcha == 'R')
		PORTB |= 0b11000000;
	else
		PORTB &= 0b00111111;
		
	if(lux < 300)
		PORTB |= 0b00000110;
	else
		PORTB &= 0b11111001;
}

int main(void)
{
	distancia = eeprom_read_dword(endereco_distancia); //Leitura do diametro que foi escrito no endereço 0
	diametro = eeprom_read_byte(endereco_diametro); //Leitura da distancia que foi escrita no endereço 4
	
	DDRB = 0b11001110; //habilita os pinos 1-7 como saída da porta B
	DDRC = 0b00110000; //habilita os pinos 4-5 como saída da porta C
	DDRD = 0b00001000; //habilita o pino 3 como saída porta D para o PWM
	PORTB = 0b00010001; //habilita o resistor de pull-up do pinos 0 da porta B
	PORTC = 0b11111111; //desabilita os resistores de pull-up de todos os pinos da porta C
	PORTD = 0b00110100; //habilita os resistores de pull-up dos pinos 4 e 5 da porta D
	
	PCICR  = 0b00000100; //Habilita a interupção das portas D: PCINTD
	PCMSK2 = 0b11110100; //Habilita a interrupção nos pinos 2, 4, 5, 6 e 7 da porta D
	
	TCCR0A = 0b00000010; //Habilita o modo CTC do TC0
	TCCR0B = 0b00000010; //Habilita o presscaler 8
	TIMSK0 = 0b00000010; //Habilita a inturrupção por iguadade comparando com OCR0A
	OCR0A = 199; //Ajusta o comparador para o TC0 contar até 199
	
	TCCR2A = 0b00100011; //habilita o PWM nao invertido
	TCCR2B = 0b00000100; //habilita o presscaler 64
	OCR2B = 128; //controle do PWM
	
	TCCR1B = (1<<ICES1) | (1<<CS12); //captura na borda de descida, presscaler = 256
	TIMSK1 = 1<<ICIE1; //habilita a interrupção por captura
	
	ADCSRA = 0b11101111; //habilita o AD presscaler 128 
	ADCSRB = 0b00000000; //conversão contínua do ADC
	DIDR0 = 0b00000000; //desabilita o pino 0, 1 e 2 da porta C como entrada digital
	
	UBRR0H = (unsigned char)(MYUBRR>>8); //Ajusta a taxa de transmissão
	UBRR0L = (unsigned char)MYUBRR;
	UCSR0B = (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0); //Habilita o transmissor e o receptor
	UCSR0C = (1<<USBS0)|(3<<UCSZ00); //Ajusta o formato do frame: 8 bits de dados e 2 de parada
	
	sei();
	
	GLCD_Setup();
	GLCD_SetFont(Font5x8, 5, 8, GLCD_Overwrite);
	GLCD_InvertScreen();
	GLCD_Clear();

	while (1)
	{	
		distancia_objeto = delta/58;
		if((velocidade > 20) && (distancia_objeto < 300)) //Condição para mudar o duty cycle do PWM para 10%
			OCR2B = 25.6;
		
		LCD(diametro, marcha, RPM, farol, distancia, temperatura, bateria, distancia_objeto);
		display(velocidade);
		led_farol(marcha, lux);
	}
}

