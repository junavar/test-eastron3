#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <syslog.h>
#include <pthread.h>

#include <sys/ioctl.h>
#include <linux/serial.h>

#include "modbus-rtu.h"





#define NUM_MODBUS  4
int num_modbus_t = NUM_MODBUS; // previsision de manejar varios dispositivos modbus
modbus_t modbus_ts[NUM_MODBUS];

int modbus_errno=0;

//unsigned char buff_send[SIZE_BUFFER] =		{ 0x01, 0x04, 0x00, 0x00, 0x00, 0x02, 0x71, 0xCB };
unsigned char *buff_send;


//TODO: usar estas dos estructuras en la recepción de las tramas
struct modbus_response_frame *modbus_response_frame;
struct modbus_exception_response_frame *modbus_exception_response_frame;

static const uint8_t table_crc_hi[] = { 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1,
		0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
		0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1,
		0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1,
		0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
		0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1,
		0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1,
		0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40 };

/* Table of CRC values for low-order byte */
static const uint8_t table_crc_lo[] = { 0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03,
		0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C,
		0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
		0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE,
		0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17,
		0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30,
		0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35,
		0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
		0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B,
		0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24,
		0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21,
		0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6,
		0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
		0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8,
		0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD,
		0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2,
		0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53,
		0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
		0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59,
		0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E,
		0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C, 0x44, 0x84, 0x85, 0x45, 0x87, 0x47,
		0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40 };

static uint16_t crc16(uint8_t *buffer, uint16_t buffer_length) {
	uint8_t crc_hi = 0xFF; /* high CRC byte initialized */
	uint8_t crc_lo = 0xFF; /* low CRC byte initialized */
	unsigned int i; /* will index into CRC lookup */

	/* pass through message buffer */
	while (buffer_length--) {
		i = crc_hi ^ *buffer++; /* calculate the CRC  */
		crc_hi = crc_lo ^ table_crc_hi[i];
		crc_lo = table_crc_lo[i];
	}

	return (crc_hi << 8 | crc_lo);
}

	/*
	 *  Pone y devuelve valor del tiempo de espera tras enviar la solicitud
	 *  Si se pasa como argumento un valor distinto de 0 pone y devuelve ese valor
	 */
int espera_a_leer(modbus_t *pctx, int espera_arg, int speed_arg){

	if (espera_arg!=0){
		pctx->wait_time=espera_arg;
	}
	else{
		switch (speed_arg){
			case 1200:
				pctx->wait_time=ESPERA_A_LEER_1200;
				break;
			case 2400:
				pctx->wait_time=ESPERA_A_LEER_2400;
				break;
			case 4800:
				pctx->wait_time=ESPERA_A_LEER_4800;
				break;
			case 9600:
				pctx->wait_time=ESPERA_A_LEER_9600;
				break;
			default:
				pctx->wait_time=ESPERA_A_LEER_DEFAULT;
		};

	}
	return pctx->wait_time;
}






/*
 * Solicita a un dispositivo modbus que devuelva un numero de registros desde uno inicial
 */

int manda_a_modbus( modbus_t *pctx, unsigned char fun, unsigned short ini, unsigned short num_reg){
	int a_escribir, escritos;
	unsigned short crc_calc;
	struct modbus_request_frame request_frame;
	request_frame.dev=pctx->slave;
	request_frame.fun=fun;
	request_frame.ini=(ini >> 8)| (ini << 8);
	request_frame.num_reg=(num_reg >> 8)| (num_reg << 8);
	a_escribir = sizeof(struct modbus_request_frame);

	/*
	 * Pone el crc. Primero lo calcula y luego hace el swap para pasar de "big to little-endian"
	 */
	crc_calc = crc16((unsigned char *) &request_frame,
			sizeof(struct modbus_request_frame) - 2);
	request_frame.crc = (crc_calc >> 8)	| (crc_calc << 8);

	escritos = write(pctx->file_descriptor, &request_frame, a_escribir);
	if (escritos != a_escribir) {
		return -1;
	}
	// espera a que todos los bytes se hayan transmitido
	tcdrain(pctx->file_descriptor);

	/*
		espera adicional antes de devolver control apa leer la respuesta.
		Este valor es funcion de la velocidad puerto serie
		Más de 3,5 veces el tiempo de caracter. (Ver doc modbus Eastron)
	*/
	usleep(pctx->wait_time); // espera en funcion de la velocidad

	return 0;

}

/*
 *
 */

int recibe_de_modbus(modbus_t *pctx, unsigned short *registros_recibidos, unsigned short num_registros_esperados, int *nregistros_recibidos)
{

	static unsigned char buffer[255];
	memset(buffer, 0,sizeof(buffer));

	int num_bytes_datos;
	num_bytes_datos = num_registros_esperados * 2;
	int a_recibir;
	a_recibir = SIZE_HEADER_RESPONSE_FRAME + num_bytes_datos + 2; //3 bytes de cabecera + datos + 3 bytes de CRC

	int recibidos;
	recibidos = read(pctx->file_descriptor, &buffer[0], a_recibir);
	if (recibidos==-1){
		return -1; //tipicamente porque el file handle está cerrado o porque que se esta en modo "no bloqueo" y no hay bytes recibidos
	}

	if ((recibidos==5) && (buffer[1]==0x84)){
		//TODO verificar CRC y enc caso de que no cumpla no enviar este error
		//syslog(LOG_ERR, " Trama de Excepción desde device %s error code: 0x%02X", pctx->dispositivo_serie, buffer[2] );
		return 1;
	}

	if (recibidos < a_recibir) {
		//syslog(LOG_ERR, " Trama incompleta recibida de device %s: %d bytes de %d esperados",	pctx->dispositivo_serie, recibidos, a_recibir);
	    //printf("\nTrama Incompleta: %d bytes de %d\n", recibidos, a_recibir);
		return 2;
	}

	/*
	* Verifica CRC trama recibida
	*/
	unsigned short crc; // obtenido del mensaje
	crc = ((unsigned short) buffer[3 + num_bytes_datos + 0] << 8)
			| (unsigned short) buffer[3 + num_bytes_datos + 1];

	unsigned short crc_calc; //crc calculado
	crc_calc = crc16(buffer, num_bytes_datos + 3);
	if (crc != crc_calc) {
		//syslog(LOG_ERR, " CRC en trama recibida de device %s de modbus", pctx->dispositivo_serie);
		return 3;
	}

	if (buffer[2]!=num_bytes_datos ){
		//syslog(LOG_ERR, " Error trama recibida de device %s: envia más parametros de los esperados", pctx->dispositivo_serie);
		return 6;
	}

	if(buffer[0]!= pctx->slave){
		//syslog(LOG_ERR, " Trama recibida NO es del dispositivo esperado");
		return 4;
	}
	if(buffer[1]!= 0x04){
		//syslog(LOG_ERR, " Trama recibida NO es respuesta de la función solicitada");
		return 5;
	}

	*nregistros_recibidos=(recibidos-5)/2; // menos 3 bytres de cabecera y  2 de CRC

	int i;
	unsigned short  *pregistro = (unsigned short *)&buffer[3];

	for (i=0; i < *nregistros_recibidos;i++){
		//se copia dando la vuelta a la bytes de cada registro de 16bits
		registros_recibidos[i]= 	(pregistro[i]  >> 8 ) | (pregistro[i] << 8);
	}

	return 0;
}


modbus_t* modbus_new_rtu(const char *device, int baud, char parity, int data_bit, int stop_bit)
{

		//TODO buscar sitio en array de modbus_t de forma correcta que permita reaperturas en caso de error
		int i;
		for(i=0;i<num_modbus_t;i++){
			if (strnlen(modbus_ts[i].dispositivo_serie, 255-1)==0){
				strncpy(modbus_ts[i].dispositivo_serie, device, 255-1);
				modbus_ts[i].baud=baud;
				modbus_ts[i].data_bit=data_bit;
				modbus_ts[i].parity=parity;
				modbus_ts[i].stop_bit=stop_bit;
				// pone retraso de la lectura tras la escritura de la solicitud en función de la velocidad
				switch (baud){
					case 1200:
						modbus_ts[i].wait_time=ESPERA_A_LEER_1200;
						break;
					case 2400:
						modbus_ts[i].wait_time=ESPERA_A_LEER_2400;
						break;
					case 4800:
						modbus_ts[i].wait_time=ESPERA_A_LEER_4800;
						break;
					case 9600:
						modbus_ts[i].wait_time=ESPERA_A_LEER_9600;
						break;
					default:
						modbus_ts[i].wait_time=ESPERA_A_LEER_DEFAULT;
				};
				break;
			}

		}

		if (i==4)				{
			return (modbus_t *)-1;
		}

		return &modbus_ts[i];
	}

/*
 * Realiza la apertura del dispositivo serie
 */
int modbus_connect(modbus_t *pctx) {

	/*
	* Se abre el puerto serie
	* El usuario bajo el que corre
	* el programa debe estar en el grupo "dialout"
	*
	* The O_NDELAY .is an obsolete name for O_NONBLOCK
	*
	* En la práctica si se pone O_NDELAY(O_NONBLOCK , las funciones open() y read() no se bloquean.
	* Algunos convertidores USB-RS485 se quedan bloqueados en la funcion open() si no se pone 0_NDELAY
	*
	* the O_NONBLOCK is same as O_NDELAY option flag.
	* But there is something worth mentioning regarding the O_NODELY option.
	* If this option is used then a read operation would return 0 in case there is no more data to read from a pipe, FIFO etc.
	* But this return value of 0 is in direct conflict with return value of 0 when end of file is encountered.
	* So it is preferable to use the O_NONBLOCK option.
	*/
	pctx->file_descriptor=open(pctx->dispositivo_serie, O_RDWR | O_NOCTTY| O_NONBLOCK);
	if (pctx->file_descriptor == -1) {
		return -1;
	}

	struct termios opciones_serial;

	if (tcgetattr(pctx->file_descriptor, &opciones_serial) < 0) {

		return -1;
	}
	/*
	* Ajustes de la configuracion serie
	*/
	switch (pctx->baud){
			case 1200:
				cfsetspeed(&opciones_serial,B1200);
				break;
			case 2400:
				cfsetspeed(&opciones_serial,B2400);
				break;
			case 4800:
				cfsetspeed(&opciones_serial,B4800);
				break;
			case 9600:
				cfsetspeed(&opciones_serial,B9600);
				break;
			default:
				cfsetspeed(&opciones_serial,B2400);
	}

	//TODO: Poner correctamente bit de datos, paridad y stop bit desde la estructura

	opciones_serial.c_cflag |= CS8;/*8 bits de datos sin paridad */
	opciones_serial.c_cflag &= ~CSTOPB; // 1 bit de stop
	opciones_serial.c_cflag |= CLOCAL; // Ignora las linea de control de modem
	opciones_serial.c_cflag |= CREAD; // Enable receiver

	cfmakeraw(&opciones_serial); /* modo raw */

	// se ajusta para que desbloquear read() si no llegan bytes en 100ms
	// Si no llegan ningun bytes a tiempo ...
	opciones_serial.c_cc[VMIN] = 0;
	opciones_serial.c_cc[VTIME] = 1;

	// Aplica la configuración
	if (tcsetattr(pctx->file_descriptor, TCSANOW, &opciones_serial) < 0) {
		return -1;
	};
	tcflush(pctx->file_descriptor, TCIOFLUSH); //¿es esto necesrio?


	// La siguiente llamada a la función fcntl() pretende que las llamadas a read() se bloquen en espera de algunos bytes o del timeout pero realmente pone a "0" más flags
	// fcntl(fd_modbus, F_SETFL, 0);
	// Es mejor ser más selectivos obteniendo los flags actuales y actuando solo sobre el bit-flag que controla este bloqueo


	int oldflags = fcntl (pctx->file_descriptor, F_GETFL, 0); //Obtiene flags actuales



	/* la siguiente función con O_NONBLOCK puesto provoca que read() NO se bloquee si no hay bytes que leer
	 * devolviendo de forma inmediata -1 con errno=11
		   //fcntl(fd_modbus, F_SETFL, oldflags |=  O_NONBLOCK);


	 * la siguiente función con O_NONBLOCK quitado hace que read() SI se bloquee en espera de algun byte
	 * devolviendo en su caso los bytes recibidos  sin error,  incluso 0 cuando se supera VTIME.
	 *
	 * si se pone con valores bajos de espera_a leer no da errno 11 y las tramas siempres llegan incompletas
	 */
	fcntl(pctx->file_descriptor, F_SETFL, oldflags &= ~O_NONBLOCK);

	return 0;
}

int modbus_set_slave(modbus_t* pctx, int slave){

	if (slave<1 ||slave>254 ){ // eastron sdm235 tien un rango 1 a 247
		return -1;
	}
	pctx->slave=slave;
	return 0;
}


int modbus_flush(modbus_t *pctx){

	// TODO verificar la implementacion de esta funcion

	if (tcflush(pctx->file_descriptor,TCIOFLUSH)!=0){
		return -1;
	}
	return 0;
}



int modbus_read_input_registers(modbus_t *pctx, int addr, int nb, uint16_t *dest){

	int res;
	res = manda_a_modbus( pctx, 0x04, (unsigned short) addr, (unsigned short) nb);
	if (res){
		return -1;
	}

	int nregistros_recibidos;
	res = recibe_de_modbus(pctx, dest,  nb, &nregistros_recibidos);
	if (res){
			// pone en esta variable el error que devuelve recibe_de_modbus()
			modbus_errno=res;
			return -1;
	}

	return nregistros_recibidos;
}

void modbus_close(modbus_t *ctx){

}

void modbus_free(modbus_t *ctx){

}





