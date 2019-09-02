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

#define  vgcs_fnc_status_e 		ukfmo_fnc_status_e

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

typedef struct
{
	ukfmo_matrix_s mat_s;
	__VGCS_FPT__ memForMatrix[VGCS_LEN_SIGMA_COL][1u];
} vgcs_matrix_13_1_s;

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
	__VGCS_FPT__ 				dt;
} vgcs_meas_data_s;

/*-------------------------------------------------------------------------*//**
 * @brief  Структура для хранения матриц шумов "процесса" и "измерения"
 */
typedef struct
{
	/*------------------------------------------------------------------------*//**
	 * @brief 	Process noise covariance matrix
	 */
	vgcs_matrix_6x6_s QMat_s;

	/*------------------------------------------------------------------------*//**
	 * @brief 	Measurement noise covariance matrix
	 */
	vgcs_matrix_6x6_s RMat_s;
} vgca_noise_matrix_s;

typedef struct
{
	/*------------------------------------------------------------------------*//**
	 * @brief Корень квадратный из суммы "Lаmbda" и "VGCS_LEN_STATE"
	 */
	__VGCS_FPT__ sqrtLamLen;
} vgcs_scalar_params_s;

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
	vgcs_matrix_6x6_s 	stateMat_s;

	/*------------------------------------------------------------------------*//**
	 * @brief Матрицы шумов
	 */
	vgca_noise_matrix_s noiseMatrix_s;

	/*------------------------------------------------------------------------*//**
	 * @brief Матрицы ковариаций ("P_k-1")
	 */
//	vgcs_matrix_6x6_s 	covMat_s;

	/*------------------------------------------------------------------------*//**
	 * @brief Корень квадратный от матрицы ковариаций ("sqrt P")
	 */
	vgcs_matrix_6x6_s 	sqrtP_apriori_s;

	/*------------------------------------------------------------------------*//**
	 * @brief Матрица распределения сигма-точек (chi k-1)
	 */
	vgcs_matrix_6_13_s 	chiSigmaMat_s;

	/*------------------------------------------------------------------------*//**
	 * @brief Матрица сигма-точек (после функции преобразования) (chi k|k-1)
	 */
	vgcs_matrix_6_13_s 	chiSigmaPostMat_s;

	vgcs_scalar_params_s scalar_s;

	vgcs_matrix_6_1_s x_apriori_s;

	vgcs_matrix_6_1_s x_posteriori_s;

	vgcs_matrix_13_1_s muMean_s;
	vgcs_matrix_13_1_s muCovar_s;

	vgcs_matrix_6_1_s chi_apriory_minus_x_apriory_s;
	vgcs_matrix_6_1_s chi_apriory_minus_x_apriory_Transpose_s;
	vgcs_matrix_6x6_s resultOfMult2Matrix_s;

	vgcs_matrix_6x6_s P_apriori_s;

	vgcs_matrix_6x6_s P_predict_s;

	vgcs_matrix_6_13_s psi_apriori_s;

	vgcs_matrix_6_1_s y_apriori_s;

	vgcs_matrix_6x6_s Pyy_s;

	vgcs_matrix_6x6_s PyyInv_s;

	vgcs_matrix_6x6_s Pxy_s;

	vgcs_matrix_6x6_s K_s;

	vgcs_matrix_6x6_s K_Transpose_s;

	vgcs_matrix_6_1_s psi_priory_MINUS_y_priory;

	vgcs_matrix_6_1_s y_predict_s;

	vgcs_matrix_6_1_s innovation_s;
} vgcs_data_s;

/*-------------------------------------------------------------------------*//**
 * @brief Структура для инициализации UKF
 */
typedef struct
{
	ukfsif_scaling_param_s scalParams_s;
} vgcs_data_init_s;

#define __VGCS_GET_ADDR_MATRIX_STRUCT_R_k(BASEADDR)	\
	(&BASEADDR->noiseMatrix_s.RMat_s.mat_s)
#define __VGCS_GET_ADDR_MATRIX_MEMORY_R_k(BASEADDR)	\
	(BASEADDR->noiseMatrix_s.RMat_s.memForMatrix[0u])

#define __VGCS_GET_ADDR_MATRIX_STRUCT_Q_k(BASEADDR)	\
	(&BASEADDR->noiseMatrix_s.QMat_s.mat_s)
#define __VGCS_GET_ADDR_MATRIX_MEMORY_Q_k(BASEADDR)	\
	(BASEADDR->noiseMatrix_s.QMat_s.memForMatrix[0u])

/*-------------------------------------------------------------------------*//**
 * @brief    Calculate error covariance matrix square root 
 */
#define VGCS_GET_ADDR_MATRIX_STRUCT_P_k(BASEADDR)


#define VGCS_GET_ADDR_MATRIX_MEMORY_P_k(BASEADDR)

/*-------------------------------------------------------------------------*//**
 * @brief   Calculate error covariance matrix square root  
 */
#define VGCS_GET_ADDR_MATRIX_STRUCT_P_k_k1_SQRT(BASEADDR)

#define VGCS_GET_ADDR_MATRIX_MEMORY_P_k_k1_SQRT(BASEADDR)

/*-------------------------------------------------------------------------*//**
 * @brief    Calculate the sigma-points 
 */
#define VGCS_GET_ADDR_MATRIX_chi_k1(BASEADDR)

/*-------------------------------------------------------------------------*//**
 * @brief    Propagate each sigma-point through prediction 
 */
#define VGCS_GET_ADDR_MARTIX_chi_k_k1(BASEADDR)

/*-------------------------------------------------------------------------*//**
 * @brief    Calculate mean of predicted state
 */
#define VGCS_GET_ADDR_MARTIX_x_k_k1(BASEADDR)

/*-------------------------------------------------------------------------*//**
 * @brief    Calculate covariance of predicted state 
 */
#define VGCS_GET_ADDR_MARTIX_P_k_k1(BASEADDR)

/*-------------------------------------------------------------------------*//**
 * @brief    Propagate each sigma-point through observation 
 */
#define VGCS_GET_ADDR_MARTIX_psi_k_k1(BASEADDR)

/*-------------------------------------------------------------------------*//**
 * @brief    Calculate mean of predicted output 
 */
#define VGCS_GET_ADDR_MARTIX_y_k_k1(BASEADDR)

/*-------------------------------------------------------------------------*//**
 * @brief    Calculate covariance of predicted output 
 */
#define VGCS_GET_ADDR_MARTIX_Pyy(BASEADDR)

/*-------------------------------------------------------------------------*//**
 * @brief    Calculate cross-covariance of state and output  
 */
#define VGCS_GET_ADDR_MARTIX_Pxy(BASEADDR)

/*-------------------------------------------------------------------------*//**
 * @brief    Calculate Kalman gain 
 */
#define VGCS_GET_ADDR_MARTIX_K_k(BASEADDR)

/*-------------------------------------------------------------------------*//**
 * @brief    Update state estimate 
 */
#define VGCS_GET_ADDR_MARTIX_x_k(xBASEADDR)

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

extern void __VGCS_FNC_ONCE_MEMORY_LOCATION
VGSS_Init_MatrixStructs(
	vgcs_data_s 		*pData_s,
	ukfsif_all_data_s 	*pMatrixPointers_s);

__VGCS_ALWAYS_INLINE void
VGCS_UpdateAccInWorldFrame(
	vgcs_data_s 		*pData_s,
	__VGCS_FPT__ 		*pAcc)
{
	pData_s->meas_s.accWorldFrame_s.new_a[0u] = *pAcc++;
	pData_s->meas_s.accWorldFrame_s.new_a[1u] = *pAcc++;
	pData_s->meas_s.accWorldFrame_s.new_a[2u] = *pAcc;
}

__VGCS_ALWAYS_INLINE void
VGCS_UpdateSpeedByGNSS(
	vgcs_data_s 	*pData_s,
	__VGCS_FPT__ 	*pVel)
{
	pData_s->meas_s.velByGNSSWorldFrame_s.new_a[0u] = *pVel++;
	pData_s->meas_s.velByGNSSWorldFrame_s.new_a[1u] = *pVel++;
	pData_s->meas_s.velByGNSSWorldFrame_s.new_a[2u] = *pVel;
}
/*#### |End  | <-- Секция - "Прототипы глобальных функций" ###################*/


/*#### |Begin| --> Секция - "Определение макросов" ###########################*/


#if defined (__UKFMO_CHEKING_ENABLE__)

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
__VGCS_ALWAYS_INLINE ukfmo_matrix_s*
__VGCS_CheckMatrixStructValidation(
	ukfmo_matrix_s *pData)
{
	/* Вызов макроса для проверки параметров структуры */
	__UKFMO_CheckMatrixStructValidationGeneric(
		pData,
		(VGCS_LEN_SIGMA_COL),
		(VGCS_LEN_SIGMA_COL));

	return (pData);
}
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
