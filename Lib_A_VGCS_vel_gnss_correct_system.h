/**
 * @file   	%<%NAME%>%.%<%EXTENSION%>%
 * @author 	%<%USER%>%
 * @version
 * @date 	%<%DATE%>%, %<%TIME%>%
 * @brief
 */


#ifndef LIB_A_VGCS_VEL_GNSS_CORRECT_SYSTEM_H_
#define LIB_A_VGCS_VEL_GNSS_CORRECT_SYSTEM_H_


/*#### |Begin| --> Секция - "Include" ########################################*/
/*==== |Begin| --> Секция - "C libraries" ====================================*/
/*==== |End  | <-- Секция - "C libraries" ====================================*/

/*==== |Begin| --> Секция - "RTOS libraries ==================================*/
/*==== |End  | <-- Секция - "RTOS libraries ==================================*/

/*==== |Begin| --> Секция - "MK peripheral libraries" ========================*/
/*==== |End  | <-- Секция - "MK peripheral libraries" ========================*/

/*==== |Begin| --> Секция - "Extern libraries" ===============================*/
#include "Lib_A_UKFMO_ukf_matrix_operations.h"
#include "Lib_A_UKFSIF_ukf_standart_init_fnc.h"
/*==== |End  | <-- Секция - "Extern libraries" ===============================*/
/*#### |End  | <-- Секция - "Include" ########################################*/


/*#### |Begin| --> Секция - "Определение констант" ###########################*/

/*==== |Begin| --> Секция определения типа числа с плавающей точкой ==========*/
#if !defined (__VGCS_FPT__)
	#error "Please, set __VGCS_FPT__ float or double in macros list"
#endif

#if !defined (__VGCS_FPT_SIZE__)
	#error "Please, set __VGCS_FPT_SIZE__ 4 (that mean float) or 8 (that mean double) in macros list"
#endif

#if     __VGCS_FPT_SIZE__ == 4

#elif   __VGCS_FPT_SIZE__ == 8

#else
	#error "Your compiler uses a non-standard floating point size"
#endif
/*==== |End  | <-- Секция определения типа числа с плавающей точкой ==========*/

/*==== |Begin| --> Секция - Макросы для встраиваемых функций =================*/
#if defined (__GNUC__)

	/* inline*/
	#ifndef __VGCS_INLINE
		#define __VGCS_INLINE          	inline
	#endif

	/* static inline */
	#ifndef __VGCS_STATIC_INLINE
		#define __VGCS_STATIC_INLINE   	static inline
	#endif

	/* always inline */
	#ifndef __VGCS_ALWAYS_INLINE
		#define __VGCS_ALWAYS_INLINE    inline __attribute__((always_inline)) static
	#endif

	/* force inline */
	#ifndef __VGCS_FORCE_INLINE
		#define __VGCS_FORCE_INLINE    	inline __attribute__((always_inline))
	#endif

#else
	#define __VGCS_INLINE
	#define __VGCS_STATIC_INLINE   static
	#define __VGCS_ALWAYS_INLINE
#endif
/*==== |End  | <-- Секция - Макросы для встраиваемых функций =================*/


/*==== |Begin| --> Секция - Расположение функций библиотеки в специальной
 *                          области памяти ===================================*/
#if defined (__VGCS_FNC_ONCE_MEMORY_LOCATION_NAME__)
	#if defined (__GNUC__)
		#define __VGCS_FNC_ONCE_MEMORY_LOCATION  __attribute__ ((section(__VGCS_FNC_ONCE_MEMORY_LOCATION_NAME__)))
	#else
		#error "You defined the name of the memory area for the function location, but the type of your compiler is not supported by the library. You can delete the macro definition __VGCS_FNC_ONCE_MEMORY_LOCATION_NAME__ or extend the macro definition __VGCS_FNC_ONCE_MEMORY_LOCATION for your compiler type"
	#endif
#else
	#define __VGCS_FNC_ONCE_MEMORY_LOCATION
#endif

#if defined (__VGCS_FNC_LOOP_MEMORY_LOCATION_NAME__)
	#if defined (__GNUC__)
		#define __VGCS_FNC_LOOP_MEMORY_LOCATION  __attribute__ ((section(__VGCS_FNC_LOOP_MEMORY_LOCATION_NAME__)))
	#else
		#error "You defined the name of the memory area for the function location, but the type of your compiler is not supported by the library. You can delete the macro definition __VGCS_FNC_LOOP_MEMORY_LOCATION_NAME__ or extend the macro definition __VGCS_FNC_LOOP_MEMORY_LOCATION for your compiler type"
	#endif
#else
	#define __VGCS_FNC_LOOP_MEMORY_LOCATION
#endif
/*==== |End  | <-- Секция - Расположение функций библиотеки в специальной
 *                          области памяти ===================================*/

/*-------------------------------------------------------------------------*//**
 * @brief  Перечисляемый тип, задающий расположение параметров системы в векторе
 *         пространства состояний
 */
typedef enum
{
	VGCS_VEL_X = 0u,		/*!< Оценка скорости ось "X" */
	VGCS_VEL_Y,				/*!< Оценка скорости ось "Y" */
	VGCS_VEL_Z,				/*!< Оценка скорости ось "Z" */

	VGCS_ACC_ERR_X, 		/*!< Оценка погрешности акселерометра ось "X" */
	VGCS_ACC_ERR_Y, 		/*!< Оценка погрешности акселерометра ось "Y" */
	VGCS_ACC_ERR_Z, 		/*!< Оценка погрешности акселерометра ось "Z" */

	/**
	 * @brief Длина вектора пространства состояний
	 */
	VGCS_LEN_STATE,
} vgcs_state_param_position_e;

/*-------------------------------------------------------------------------*//**
 * @brief    Количество строк матрицы сигма-точек
 */
#define VGCS_LEN_SIGMA_ROW 					(VGCS_LEN_STATE)
/*-------------------------------------------------------------------------*//**
 * @brief    Количество столбцов матрицы сигма-точек
 */
#define VGCS_LEN_SIGMA_COL 					((VGCS_LEN_STATE * 2u) + 1u)


/*-------------------------------------------------------------------------*//**
 * @brief    Длина вектора весовых коэффициентов "среднего"
 */
#define VGCS_LEN_VECT_MEAN 					(VGCS_LEN_SIGMA_COL)
/*-------------------------------------------------------------------------*//**
 * @brief    Длина вектора весовых коэффициентов "ковариации"
 */
#define VGCS_LEN_VECT_COV					(VGCS_LEN_SIGMA_COL)


/*-------------------------------------------------------------------------*//**
 * @brief    Количество строк матриц фильтра Калмана
 */
#define VGCS_LEN_MATRIX_ROW 				VGCS_LEN_STATE
/*-------------------------------------------------------------------------*//**
 * @brief    Количество столбцов матриц фильтра Калмана
 */
#define VGCS_LEN_MATRIX_COL 				VGCS_LEN_STATE

/*#### |End  | <-- Секция - "Определение констант" ###########################*/


/*#### |Begin| --> Секция - "Определение типов" ##############################*/

/*-------------------------------------------------------------------------*//**
 * @brief  Структура для матрицы размерностью 6x6
 */
typedef struct
{
	ukfmo_matrix_data_s mat_s;
	__VGCS_FPT__ memForMatrix[VGCS_LEN_MATRIX_ROW][VGCS_LEN_MATRIX_COL];
} vgcs_matrix_6x6_s;

/*-------------------------------------------------------------------------*//**
 * @brief  Структура для матрицы размерностью 6x1
 */
typedef struct
{
	ukfmo_matrix_s mat_s;
	__VGCS_FPT__ memForMatrix[VGCS_LEN_STATE][1u];
} vgcs_matrix_6_1_s;

/*-------------------------------------------------------------------------*//**
 * @brief  Структура для матрицы размерностью 6x13
 */
typedef struct
{
	ukfmo_matrix_s mat_s;
	__VGCS_FPT__ memForMatrix[VGCS_LEN_SIGMA_ROW][VGCS_LEN_SIGMA_COL];
} vgcs_matrix_6_13_s;

/*-------------------------------------------------------------------------*//**
 * @brief  Структура для хранения проекции вектора кажущегося ускорения
 *         на оси нормальной земной СК
 */
typedef struct
{
	__VGCS_FPT__ new_a[3u];
} vgcs_acc_world_frame_s;

/*-------------------------------------------------------------------------*//**
 * @brief  Структура для хранения проекции вектора скорости от GNSS модуля
 *         на оси нормальной земной СК
 */
typedef struct
{
	__VGCS_FPT__ new_a[3u];
} vgcs_velgnss_world_frame_s;

/*-------------------------------------------------------------------------*//**
 * @brief  Структура для хранения данных, поступающих от измерителей
 */
typedef struct
{
	vgcs_acc_world_frame_s 		accWorldFrame_s;
	vgcs_velgnss_world_frame_s 	velByGNSSWorldFrame_s;
} vgcs_meas_data_s;

/*-------------------------------------------------------------------------*//**
 * @brief  Структура для хранения матриц шумов "процесса" и "измерения"
 */
typedef struct
{
	/*------------------------------------------------------------------------*//**
	 * @brief 	Process noise covariance matrix
	 */
	vgcs_matrix_6x6_s Q_s;

	/*------------------------------------------------------------------------*//**
	 * @brief 	Measurement noise covariance matrix
	 */
	vgcs_matrix_6x6_s R_s;
} vgca_noise_matrix_s;

/*-------------------------------------------------------------------------*//**
 * @brief Структура для хранения всех данных, необходимых для работы UKF
 */
typedef struct
{
	/*------------------------------------------------------------------------*//**
	 * @brief    Данные, полученные от бортовых измерителей
	 */
	vgcs_meas_data_s 	meas_s;

	/*------------------------------------------------------------------------*//**
	 * @brief Вектор пространства состояний
	 */
	vgcs_matrix_6_1_s 	state_s;

	/*------------------------------------------------------------------------*//**
	 * @brief Матрицы шумов
	 */
	vgca_noise_matrix_s noiseMatrix_s;

} vgcs_data_s;

/*-------------------------------------------------------------------------*//**
 * @brief Структура для инициализации UKF
 */
typedef struct
{
	ukfsif_scaling_param_s scalParams_s;
} vgcs_data_init_s;
/*#### |End  | <-- Секция - "Определение типов" ##############################*/


/*#### |Begin| --> Секция - "Определение глобальных переменных" ##############*/
/*#### |End  | <-- Секция - "Определение глобальных переменных" ##############*/


/*#### |Begin| --> Секция - "Прототипы глобальных функций" ###################*/
extern void __VGCS_FNC_ONCE_MEMORY_LOCATION
VGCS_InitStruct(
	vgcs_data_init_s *pUKF_s);

extern void __VGCS_FNC_ONCE_MEMORY_LOCATION
VGSS_Init_All(
	vgcs_data_s 		*pData_s,
	vgcs_data_init_s 	*pInit_s);
/*#### |End  | <-- Секция - "Прототипы глобальных функций" ###################*/


/*#### |Begin| --> Секция - "Определение макросов" ###########################*/
#if defined (__UKFMO_CHEKING_ENABLE__)

#define __VGCS_CheckMatrixStructValidation(x) \
	__UKFMO_CheckMatrixStructValidationGeneric(x, (VGCS_LEN_SIGMA_COL), (VGCS_LEN_SIGMA_COL))
#else

/*-------------------------------------------------------------------------*//**
 * @author    Mickle Isaev
 * @date      22-авг-2019
 *
 * @brief    Макрос проверяет валидность структуры матрицы, если матрица
 *           не валидна, то макрос зацикливает программу
 *
 * @param[in]	x: 	Указатель на структуру матрицы
 *
 * @return   None
 */
#define __VGCS_CheckMatrixStructValidation(x)
#endif
/*#### |End  | <-- Секция - "Определение макросов" ###########################*/


/*#### |Begin| --> Секция - "Include - подмодули" ############################*/
/*#### |End  | <-- Секция - "Include - подмодули" ############################*/

#endif	/* LIB_A_VGCS_VEL_GNSS_CORRECT_SYSTEM_H_ */

/*############################################################################*/
/*################################ END OF FILE ###############################*/
/*############################################################################*/
