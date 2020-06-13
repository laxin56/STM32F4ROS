/*
 * mainpp.h
 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */

#ifndef MAINPP_H_
#define MAINPP_H_

#ifdef __cplusplus
 extern "C" {
#endif


void setup(void);    // inicjalizacja

void Get_data(void); //pobieranie danych z akcelerometru

void Get_gyro(); //Pobieranie danych z zyroskopu

void initialize(void);

void Calculate(void); //obliczanie pozycji

void Napraw_Blad(int); //Funkcja prostujaca blad calkowania

void Wez_pozycje(void);


#ifdef __cplusplus
}
#endif


#endif /* MAINPP_H_ */
