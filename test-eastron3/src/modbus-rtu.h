/*
 * modbus-rtu.h
 *
 *  Created on: Apr 14, 2019
 *      Author: juan
 */

#ifndef MODBUS_RTU_H_
#define MODBUS_RTU_H_

typedef struct modbus_t{
	int file_descriptor;
	char dispositivo_serie[255];
	int baud;
	char parity;
	int data_bit;
	int stop_bit;
	int slave;
	int vmin; // parametro VMIN de puerto serie
	int vtime;// parametero VTIME de puerto serie
	int wait_time; // espera enter write y read en el dispistivo serie
} modbus_t;




#define MAX_SIZE_DATA_FIELD 255
#define SIZE_BUFFER 255
#define SIZE_HEADER_RESPONSE_FRAME 3
#define SIZE_HEADER_REQUEST FRAME 2
#define SIZE_CRC 2
#define SIZE_HEADER_RESPONSE_FRAME_PLUS_CRC SIZE_HEADER_RESPONSE_FRAME + SIZE_CRC

#define ESPERA_A_LEER_DEFAULT	420000  // tiempos de espera por defecto entre escritura solicitud y lectura de respuesta
#define ESPERA_A_LEER_1200 		500000
#define ESPERA_A_LEER_2400 		480000
#define ESPERA_A_LEER_4800 		300000
#define ESPERA_A_LEER_9600 		220000



struct modbus_request_frame {
	unsigned char dev;
	unsigned char fun;
	unsigned short ini; //ojo en big endian
	unsigned short num_reg; //ojo en big endian
	unsigned short crc; //ojo en big endian
};

struct modbus_response_frame {
	unsigned char dev;
	unsigned char fun;
	unsigned char lenght; // number of bytes in data field excluding 2 bytes of CRC
	unsigned char data_plus_crc[MAX_SIZE_DATA_FIELD + 2];//variable lenght value of queried command (max. 255 bytes)
															//last two bytes is the crc of all bytes in frame except crc
};

struct modbus_exception_response_frame {
	unsigned char dev;
	unsigned char fun;
	unsigned char error_code;
	unsigned short crc;
};


modbus_t* modbus_new_rtu(const char *device, int baud, char parity, int data_bit, int stop_bit);
int modbus_connect(modbus_t *pctx);
int modbus_set_slave(modbus_t* pctx, int slave);
int modbus_flush(modbus_t *pctx);
int modbus_read_input_registers(modbus_t *pctx, int addr, int nb, uint16_t *dest);
void modbus_close(modbus_t *ctx);
void modbus_free(modbus_t *ctx);


extern int modbus_errno;


#endif /* MODBUS_RTU_H_ */
