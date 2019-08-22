/**
 * @file   	%<%NAME%>%.%<%EXTENSION%>%
 * @author 	%<%USER%>%
 * @version
 * @date 	%<%DATE%>%, %<%TIME%>%
 * @brief
 */


/*#### |Begin| --> Секция - "Include" ########################################*/
#include "Lib_A_VGCS_vel_gnss_correct_system.h"
/*#### |End  | <-- Секция - "Include" ########################################*/


/*#### |Begin| --> Секция - "Глобальные переменные" ##########################*/
/*#### |End  | <-- Секция - "Глобальные переменные" ##########################*/


/*#### |Begin| --> Секция - "Локальные переменные" ###########################*/
/*#### |End  | <-- Секция - "Локальные переменные" ###########################*/


/*#### |Begin| --> Секция - "Прототипы локальных функций" ####################*/
/*#### |End  | <-- Секция - "Прототипы локальных функций" ####################*/


/*#### |Begin| --> Секция - "Описание глобальных функций" ####################*/
void __VGCS_FNC_ONCE_MEMORY_LOCATION
VGCS_InitStruct(
	vgcs_data_init_s *pUKF_s)
{
	/* Сброс скалярных параметров в значения по умолчанию */
	pUKF_s->scalParams_s.alpha 	= (__VGCS_FPT__) 1.0;
	pUKF_s->scalParams_s.beta 	= (__VGCS_FPT__) 2.0;
	pUKF_s->scalParams_s.kappa 	= (__VGCS_FPT__) 0.0;
}

void __VGCS_FNC_ONCE_MEMORY_LOCATION
VGSS_Init_MatrixStructs(
	vgcs_data_s *pData_s)
{
	/* Инициализация матрицы шума Q */
	UKFMO_MatrixInit(
		&pData_s->noiseMatrix_s.Q_s.mat_s,
		VGCS_LEN_MATRIX_ROW,
		VGCS_LEN_MATRIX_COL,
		pData_s->noiseMatrix_s.Q_s.memForMatrix[0u]);
	__VGCS_CheckMatrixStructValidation(
		&pData_s->noiseMatrix_s.Q_s.mat_s);

	/* Инициализация матрицы шума R */
	UKFMO_MatrixInit(
		&pData_s->noiseMatrix_s.R_s.mat_s,
		VGCS_LEN_MATRIX_ROW,
		VGCS_LEN_MATRIX_COL,
		pData_s->noiseMatrix_s.R_s.memForMatrix[0u]);
	__VGCS_CheckMatrixStructValidation(
		&pData_s->noiseMatrix_s.R_s.mat_s);

	/* Инициализация матрицы пространства состояний */
	UKFMO_MatrixInit(
		&pData_s->state_s.mat_s,
		VGCS_LEN_STATE,
		1u,
		pData_s->state_s.memForMatrix[0u]);
	__VGCS_CheckMatrixStructValidation(
		&pData_s->state_s.mat_s);
}

void __VGCS_FNC_ONCE_MEMORY_LOCATION
VGSS_Init_All(
	vgcs_data_s 		*pData_s,
	vgcs_data_init_s 	*pInit_s)
{
	/* Инициализация всех структур матриц */
	VGSS_Init_MatrixStructs(pData_s);
}
/*#### |End  | <-- Секция - "Описание глобальных функций" ####################*/


/*#### |Begin| --> Секция - "Создание задач" #################################*/
/*#### |End  | <-- Секция - "Создание задач" #################################*/


/*#### |Begin| --> Секция - "Описание локальных функций" #####################*/
/*#### |End  | <-- Секция - "Описание локальных функций" #####################*/


/*############################################################################*/
/*############################ END OF FILE  ##################################*/
/*############################################################################*/
