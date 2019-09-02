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
static vgcs_fnc_status_e
VGCS_Step2_ProragateEachSigmaPointsThroughPrediction(
	vgcs_data_s *pData_s);
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

	/* @TODO Сброс периода интегрирования */
}

void __VGCS_FNC_ONCE_MEMORY_LOCATION
VGSS_Init_MatrixStructs(
	vgcs_data_s *pData_s)
{
	/* Инициализация матрицы шума Q */
	UKFMO_MatrixInit(
		__VGCS_GET_ADDR_MATRIX_STRUCT_Q_k(pData_s),
		VGCS_LEN_MATRIX_ROW,
		VGCS_LEN_MATRIX_COL,
		__VGCS_GET_ADDR_MATRIX_MEMORY_Q_k(pData_s)
	);
	__VGCS_CheckMatrixStructValidation(
		__VGCS_GET_ADDR_MATRIX_STRUCT_Q_k(pData_s));

	/* Инициализация матрицы шума R */
	UKFMO_MatrixInit(
		__VGCS_GET_ADDR_MATRIX_STRUCT_R_k(pData_s),
		VGCS_LEN_MATRIX_ROW,
		VGCS_LEN_MATRIX_COL,
		__VGCS_GET_ADDR_MATRIX_MEMORY_R_k(pData_s)
	);
	__VGCS_CheckMatrixStructValidation(
		__VGCS_GET_ADDR_MATRIX_STRUCT_R_k(pData_s));

	/* Инициализация матрицы пространства состояний */
	UKFMO_MatrixInit(
		&pData_s->stateMat_s.mat_s, 			/* !< Указатель на структуру матрицы */
		VGCS_LEN_MATRIX_ROW, 					/* !< Количество строк */
		VGCS_LEN_MATRIX_COL,					/* !< Количество столбцов */
		pData_s->stateMat_s.memForMatrix[0u] 	/* !< Указатель на область памяти для хранения матрицы */
	);
	__VGCS_CheckMatrixStructValidation(
		&pData_s->stateMat_s.mat_s);

	/* Инициализация матрицы ковариации */
	UKFMO_MatrixInit(
		&pData_s->covMat_s.mat_s,
		VGCS_LEN_MATRIX_ROW,
		VGCS_LEN_MATRIX_COL,
		pData_s->covMat_s.memForMatrix[0u]
	);
	__VGCS_CheckMatrixStructValidation(
		&pData_s->covMat_s.mat_s);

	/* Инициализация матрицы сигма-точек */
	UKFMO_MatrixInit(
		&pData_s->chiSigmaMat_s.mat_s,
		VGCS_LEN_SIGMA_ROW,
		VGCS_LEN_SIGMA_COL,
		pData_s->chiSigmaMat_s.memForMatrix[0u]
	);
	__VGCS_CheckMatrixStructValidation(
		&pData_s->chiSigmaMat_s.mat_s);

	/* Инициализация матрицы сигма-точек (после функции преобразования) */
	UKFMO_MatrixInit(
		&pData_s->chiSigmaPostMat_s.mat_s,
		VGCS_LEN_SIGMA_ROW,
		VGCS_LEN_SIGMA_COL,
		pData_s->chiSigmaPostMat_s.memForMatrix[0u]
	);
	__VGCS_CheckMatrixStructValidation(
		&pData_s->chiSigmaPostMat_s.mat_s);

	/* Инициализация матрицы квадратного корня от матрицы ковариации "P" */
	UKFMO_MatrixInit(
		&pData_s->sqrtCovMat_s.mat_s,
		VGCS_LEN_MATRIX_ROW,
		VGCS_LEN_MATRIX_COL,
		pData_s->sqrtCovMat_s.memForMatrix[0u]
	);
	__VGCS_CheckMatrixStructValidation(
		&pData_s->sqrtCovMat_s.mat_s);

}

void __VGCS_FNC_ONCE_MEMORY_LOCATION
VGSS_Init_All(
	vgcs_data_s 		*pData_s,
	vgcs_data_init_s 	*pInit_s)
{
	/* Инициализация всех структур матриц */
	VGSS_Init_MatrixStructs(pData_s);

	/* @TODO вычисление корня квадратного из (lambda + len) и запись в поле структуры */

	/* @TODO Установка периода интегрирования */
}
/*#### |End  | <-- Секция - "Описание глобальных функций" ####################*/


/*#### |Begin| --> Секция - "Создание задач" #################################*/
/*#### |End  | <-- Секция - "Создание задач" #################################*/


/*#### |Begin| --> Секция - "Описание локальных функций" #####################*/

vgcs_fnc_status_e
VGCS_Step1_GeterateTheSigmaPoints(
	vgcs_data_s *pData_s)
{
	/* #### Calculate error covariance matrix square root #### */

	/* Копирование матрицы P в матрицу SQRT_P */
	memcpy(
		(void*) &pData_s->sqrtCovMat_s	.memForMatrix[0u][0u],
		(void*) &pData_s->covMat_s		.memForMatrix[0u][0u],
		VGCS_LEN_MATRIX_ROW * VGCS_LEN_MATRIX_COL);

	#if defined (__UKFMO_CHEKING_ENABLE__)
	ukfmo_fnc_status_e matOperationStatus_e;
	#endif

	/* Проверка матрицы */
	__VGCS_CheckMatrixStructValidation(
		&pData_s->sqrtCovMat_s.mat_s);

	/* Нижнее разложение Холецкого */
	#if defined (__UKFMO_CHEKING_ENABLE__)
	matOperationStatus_e =
	#endif
		UKFMO_GetCholeskyLow(
			&pData_s->sqrtCovMat_s.mat_s);

	/* Calculate the sigma-points */
	UKFSIF_CalculateTheSigmaPoints_2L1(
		&pData_s->stateMat_s	.memForMatrix[0u][0u],
		&pData_s->chiSigmaMat_s	.memForMatrix[0u][0u],
		&pData_s->sqrtCovMat_s	.memForMatrix[0u][0u],
		pData_s->scalar_s		.sqrtLamLen,
		VGCS_LEN_SIGMA_ROW);

	#if defined (__UKFMO_CHEKING_ENABLE__)
	return (matOperationStatus_e);
	#else
	return (UKFMO_OK);
	#endif
}

vgcs_fnc_status_e
VGCS_Step2_PredictionTransformation(
	vgcs_data_s *pData_s)
{
	VGCS_Step2_ProragateEachSigmaPointsThroughPrediction(
		pData_s);

//	VGCS_Step2_CalculateMeanOfPredictedState(
//		pData_s);
//
//	VGCS_Step2_CalculateCovarianceOfPredictedState(
//		pData_s);
}

static vgcs_fnc_status_e
VGCS_Step2_ProragateEachSigmaPointsThroughPrediction(
	vgcs_data_s *pData_s)
{
	size_t i;
	__VGCS_FPT__ deltaVel_a[3u];

	for (i = 0u;
		 i < ((size_t) VGCS_LEN_SIGMA_COL);
		 i++)
	{
		/* Интегрирование вектора ускорений для получения вектора скорости (методом нулевого порядка )
		 * с учетом смещения нуля акселерометра */
		deltaVel_a[VGCS_VEL_X] =
			(pData_s->meas_s.accWorldFrame_s.new_a[VGCS_VEL_X] - pData_s->chiSigmaMat_s.memForMatrix[VGCS_ACC_ERR_X][i]) * pData_s->meas_s.dt;
		deltaVel_a[VGCS_VEL_Y] =
			(pData_s->meas_s.accWorldFrame_s.new_a[VGCS_VEL_Y] - pData_s->chiSigmaMat_s.memForMatrix[VGCS_ACC_ERR_Y][i]) * pData_s->meas_s.dt;
		deltaVel_a[VGCS_VEL_Z] =
			(pData_s->meas_s.accWorldFrame_s.new_a[VGCS_VEL_Z] - pData_s->chiSigmaMat_s.memForMatrix[VGCS_ACC_ERR_Z][i]) * pData_s->meas_s.dt;

		/* Сложение приращения скорости с предыдущим значением скорости и
		 * запись в матрицу (chi k|k-1) */
		pData_s->chiSigmaPostMat_s.memForMatrix[VGCS_VEL_X][i] =
			pData_s->chiSigmaMat_s.memForMatrix[VGCS_VEL_X][i] + deltaVel_a[VGCS_VEL_X];

		pData_s->chiSigmaPostMat_s.memForMatrix[VGCS_VEL_Y][i] =
			pData_s->chiSigmaMat_s.memForMatrix[VGCS_VEL_Y][i] + deltaVel_a[VGCS_VEL_Y];

		pData_s->chiSigmaPostMat_s.memForMatrix[VGCS_VEL_Z][i] =
			pData_s->chiSigmaMat_s.memForMatrix[VGCS_VEL_Z][i] + deltaVel_a[VGCS_VEL_Z];

		/* Копирование смещений нуля в матрицу (chi k|k-1) */
		pData_s->chiSigmaPostMat_s.memForMatrix[VGCS_ACC_ERR_X][i] =
			pData_s->chiSigmaMat_s.memForMatrix[VGCS_ACC_ERR_X][i];

		pData_s->chiSigmaPostMat_s.memForMatrix[VGCS_ACC_ERR_Y][i] =
			pData_s->chiSigmaMat_s.memForMatrix[VGCS_ACC_ERR_Y][i];

		pData_s->chiSigmaPostMat_s.memForMatrix[VGCS_ACC_ERR_Z][i] =
			pData_s->chiSigmaMat_s.memForMatrix[VGCS_ACC_ERR_Z][i];
	}

	return (UKFMO_OK);
}

//vgcs_fnc_status_e
//VGCS_Step2_CalculateMeanOfPredictedState(
//	vgcs_data_s *pData_s)
//{
//
//}
//
//vgcs_fnc_status_e
//VGCS_Step2_CalculateCovarianceOfPredictedState(
//	vgcs_data_s *pData_s)
//{
//
//}
/*#### |End  | <-- Секция - "Описание локальных функций" #####################*/


/*############################################################################*/
/*############################ END OF FILE  ##################################*/
/*############################################################################*/
