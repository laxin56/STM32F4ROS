/*
 * main.cpp

 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */
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

/*
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
char hello[] = "Hello world!";
*/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
}
//Komentarz do Hello World
/*
void setup(void)
{

  nh.initNode();
  nh.advertise(chatter);
}

void loop(void)
{
  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
  str_msg.data = hello;
  chatter.publish(&str_msg);
  nh.spinOnce();

  HAL_Delay(1000);
}

*/
extern I2C_HandleTypeDef hi2c1;

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
uint8_t Data[6]; // Zmienna do bezposredniego odczytu danych z akcelerometru
int16_t Xaxis = 0; // Zawiera przeksztalcona forme odczytanych danych z osi X
int16_t Yaxis = 0; // Zawiera przeksztalcona forme odczytanych danych z osi Y
int16_t Zaxis = 0; // Zawiera przeksztalcona forme odczytanych danych z osi Z

float Xaxis_g = 0; // Zawiera przyspieszenie w osi X przekstalcone na jednostke fizyczna [g]
float Yaxis_g = 0; // Zawiera przyspieszenie w osi Y przekstalcone na jednostke fizyczna [g]
float Zaxis_g = 0; // Zawiera przyspieszenie w osi Z przekstalcone na jednostke fizyczna [g]
float axis[2];

//double x = 1.0;
//double y = 0.0;
//double theta = 1.57;

//char base_link[] = "/base_link";
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

void loop()
{
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
	HAL_I2C_Mem_Read(&hi2c1, LSM303_ACC_ADDRESS, LSM303_ACC_X_L_A_MULTI_READ, 1, Data, 6, 100);
	// Konwersja odebranych bajtow danych na typ int16_t
	Xaxis = ((Data[1] << 8) | Data[0]);
	Yaxis = ((Data[3] << 8) | Data[2]);
	Zaxis = ((Data[5] << 8) | Data[4]);

	// obliczenie przyspieszen w kazdej z osi w jednostce SI [g]
	Xaxis_g = ((float) Xaxis * LSM303_ACC_RESOLUTION) / (float) INT16_MAX;
	Yaxis_g = ((float) Yaxis * LSM303_ACC_RESOLUTION) / (float) INT16_MAX;
	Zaxis_g = ((float) Zaxis * LSM303_ACC_RESOLUTION) / (float) INT16_MAX;

	axis[0] = Xaxis_g;
	axis[1] = Yaxis_g;
	axis[2] = Zaxis_g;
	//axis[0] = 1;
	//	axis[1] = 22;
	//	axis[2] = 3;
	imu.header.frame_id = odom;
	//imu.child_frame_id = base_link;
	imu.linear_acceleration.x = axis[0];
	imu.linear_acceleration.y = axis[1];
	imu.linear_acceleration.z = axis[2];
	pub_imu.publish(&imu);
	nh.spinOnce();
	HAL_Delay(400);

}
