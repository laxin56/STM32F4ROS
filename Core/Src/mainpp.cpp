#include <stdio.h>
#include <mainpp.h>
#include <main.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>

ros::NodeHandle nh;
sensor_msgs::Imu imu;
ros::Publisher pub_imu("imu", &imu);

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
}


int Time = 0;

short raw_gyro;
short last_gyro;
short gyro;
long long angle;

short raw_acc[2];
short last_acc[6];
short acc[6];

long long vel[6];
long long last_vel[6];
long long dis[6];

short blad[6];

extern UART_HandleTypeDef huart2;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim10; //Licznik 10 - czestotliwosc 100Hz - 10ms
extern TIM_HandleTypeDef htim11;
extern SPI_HandleTypeDef hspi1;

uint8_t address_ctrl1 = 0;
uint8_t data_ctrl1 = 0;

#define LSM303_ACC_ADDRESS (0x19 << 1) // adres akcelerometru: 0011 001x
#define LSM303_ACC_CTRL_REG1_A 0x20 // rejestr ustawien 1
#define LSM303_ACC_CTRL_REG3_A 0x22 // rejestr ustawien 3
#define LSM303_ACC_Z_H_A 0x2D // wyzszy bajt danych osi Z
#define LSM303_ACC_Z_L_A 0x2C // nizszy bajt danych osi Z
#define LSM303_ACC_X_L_A 0x28 // nizszy bajt danych osi X
// mlodszy bajt danych osi Z z najstarszym bitem ustawionym na 1 w celu
// wymuszenia autoinkrementacji adresow rejestru w urzadzeniu docelowym
// (zeby moc odczytac wiecej danych na raz)
#define LSM303_ACC_Z_L_A_MULTI_READ (LSM303_ACC_Z_L_A | 0x80)

// mlodszy bajt danych osi X z najstarszym bitem ustawionym na 1 w celu
// wymuszenia autoinkrementacji adresow rejestru w urzadzeniu docelowym
// (zeby moc odczytac wiecej danych na raz)
#define LSM303_ACC_X_L_A_MULTI_READ (LSM303_ACC_X_L_A | 0x80)

// Maski bitowe
// CTRL_REG1_A = [ODR3][ODR2][ODR1][ODR0][LPEN][ZEN][YEN][XEN]
#define LSM303_ACC_XYZ_ENABLE 0x07 // 0000 0111
#define LSM303_ACC_100HZ 0x50 //0101 0000
#define LSM303_ACC_1HZ 0x10 //0001 0000

// CTRL_REG3_A = [CLICK][AOI1][AOI2][DRDY_1][DRDY_2][WTM][OVERRUN][---]
#define LSM303_ACC_I1_DRDY1 0x10 //0001 0000
#define LSM303_ACC_I1_DRDY2 0x08 //0000 1000

#define LSM303_ACC_RESOLUTION 2.0 // Maksymalna wartosc mierzalnego przyspieszenia [g]
// Zmienne
int8_t GYRO; //Zmienna do oczytu predkosci z zyroskopu z osi Z

uint8_t Data[6]; // Zmienna do bezposredniego odczytu danych z akcelerometru
int16_t  Xaxis = 0; // Zawiera przeksztalcona forme odczytanych danych z osi X
int16_t  Yaxis = 0; // Zawiera przeksztalcona forme odczytanych danych z osi Y
int16_t Zaxis = 0; // Zawiera przeksztalcona forme odczytanych danych z osi Z

float Xaxis_g = 0; // Zawiera przyspieszenie w osi X przekstalcone na jednostke fizyczna [g]
float Yaxis_g = 0; // Zawiera przyspieszenie w osi Y przekstalcone na jednostke fizyczna [g]
float Zaxis_g = 0; // Zawiera przyspieszenie w osi Z przekstalcone na jednostke fizyczna [g]
float axis[2];


char odom[] = "/imu";

void setup()
{
  nh.initNode();
  nh.advertise(pub_imu);
uint8_t Settings = LSM303_ACC_XYZ_ENABLE | LSM303_ACC_100HZ;

	// Wpisanie konfiguracji do rejestru akcelerometru
HAL_I2C_Mem_Write(&hi2c1, LSM303_ACC_ADDRESS, LSM303_ACC_CTRL_REG1_A, 1, &Settings, 1, 100);

//	Settings = LSM303_ACC_I1_DRDY2;
//HAL_I2C_Mem_Write(&hi2c1, LSM303_ACC_ADDRESS, LSM303_ACC_CTRL_REG3_A, 1, &Settings, 1, 100);
}

void Get_data()
{

	HAL_I2C_Mem_Read(&hi2c1, LSM303_ACC_ADDRESS, LSM303_ACC_X_L_A_MULTI_READ, 1, Data, 6, 100);
	// Konwersja odebranych bajtow danych na typ int16_t
	Xaxis = ((Data[1] << 8) | Data[0]);
	Yaxis = ((Data[3] << 8) | Data[2]);
	Zaxis = ((Data[5] << 8) | Data[4]);
	//Xaxis = Data[1];
	//Yaxis = Data[3];
	// obliczenie przyspieszen w kazdej z osi w jednostce SI [g]
		//Xaxis_g = ((float) Xaxis * LSM303_ACC_RESOLUTION) / (float) INT16_MAX;
		//Yaxis_g = ((float) Yaxis * LSM303_ACC_RESOLUTION) / (float) INT16_MAX;
		//Zaxis_g = ((float) Zaxis * LSM303_ACC_RESOLUTION) / (float) INT16_MAX;
}


void Get_gyro()
{
	uint8_t data;
	uint8_t address = 0x2D | 0x80;
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &address, 1, 50);
	HAL_SPI_Receive(&hspi1, &data, 1, 50);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

	raw_gyro = data - 128;
}

/*
 * Funkcja inicjalizująca odpowiednie parametry jak timer oraz zerujaca tablice
 */
void initialize()
{

	  //gyro init

	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
	  address_ctrl1 = 0x20;
	  HAL_SPI_Transmit(&hspi1, &address_ctrl1, 1, 50);
	  data_ctrl1 = 0x0f;
	  HAL_SPI_Transmit(&hspi1, &data_ctrl1, 1, 50);
	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

	HAL_TIM_Base_Start_IT(&htim10);  //rozpoczęcie działania timera, ticki co 1ms

	for(int i = 0; i < 6; i++)
	{
		acc[i] = 0;
		last_acc[i] = 0;
		vel[i] = 0;
		last_vel[i] = 0;
		dis[i] = 0;
		last_gyro = 0;
		gyro = 0;
		angle = 0;
	}
}

/*
 * Funkcja liczaca calki z przyspieszenia
 */
void Calculate()
{
	Get_data(); //Pobierz dane z akcelerometru
	Get_gyro();
	Time = (htim10.Instance->CNT); //Pobieranie zliczen timera co 1ms, czyli pobieranie czasu
	htim10.Instance->CNT = 0;	 //Zerowanie czasu w timerze

	raw_acc[0] = Xaxis;
	raw_acc[1] = Yaxis;

	//w petli calkujemy podwojnie osie x i y akcelerometru
	for(int i = 0; i < 2; i++)
	{
		acc[i] = raw_acc[i] - blad[i];
		//predkosc - calka i filtr i usrednianie z ostatnia predkoscia
		vel[i] += (acc[i] + (acc[i] - last_acc[i]) / 2) * Time;
		// Przemieszczenie
		dis[i] += (vel[i] + (vel[i] - last_vel[i]) / 2) * Time;

		//dis[0] -> to przemieszczenie w osi x
		//dis[1] -> to przemieszczenei w osi y

		//teraz zapamietywania predkosci i przyspieszenia
		last_acc[i] = acc[i];
		last_vel[i] = vel[i];
	}

	//Pojedyncza calka zyroskopu osi z, aby otrzymywac orientacje
	//Korekta bledu calkowania
	gyro = raw_gyro - blad[2];
	//calka z predkosci katowej
	angle += (gyro + (gyro - last_gyro) / 2) * Time;

	last_gyro = gyro;
}

void Napraw_Blad(int many)
{
	long int c[6];
	int Many = many;

	for(int i =0; i<6; i++)
	{
		c[i] = 0;
		last_acc[i] = 0;
		vel[i] = 0;
		last_vel[i] = 0;
		dis[i] = 0;
		last_gyro = 0;
		angle = 0;
	}

	while(Many >0)
	{
		Get_data();
		Get_gyro();

		c[0] += Xaxis;
		c[1] += Yaxis;
		c[2] += raw_gyro;
		Many--;

	}

    for (int i = 0; i < 3; i++){
    	blad[i] = c[i]/many;
    }
}

void Wez_pozycje()
{

	float pozycja_x = 0, pozycja_y = 0, kat = 0;
	pozycja_x = dis[0];
	pozycja_y = dis[1];
	kat = angle;

			imu.header.frame_id = odom;
			imu.linear_acceleration.x = pozycja_x;
			imu.linear_acceleration.y = pozycja_y;
			imu.linear_acceleration.z = kat;
			pub_imu.publish(&imu);
			nh.spinOnce();

}
