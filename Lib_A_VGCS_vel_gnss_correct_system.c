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
static void __VGCS_FNC_ONCE_MEMORY_LOCATION
VGCS_Init_NoiseMatrix(
	ukfmo_matrix_s 	*pNoiseMat,
	__VGCS_FPT__ 	*pNoiseMatDiag);

static vgcs_fnc_status_e __VGCS_FNC_LOOP_MEMORY_LOCATION
VGCS_Step2_ProragateEachSigmaPointsThroughPrediction(
	vgcs_data_s *pData_s);

static vgcs_fnc_status_e __VGCS_FNC_LOOP_MEMORY_LOCATION
VGCS_Step1_CalculateErrorCovarianceMatrixSquareRoot(
	vgcs_data_s *pData_s);

static vgcs_fnc_status_e __VGCS_FNC_LOOP_MEMORY_LOCATION
VGCS_Step1_GeterateTheSigmaPoints(
	vgcs_data_s *pData_s);

static vgcs_fnc_status_e __VGCS_FNC_LOOP_MEMORY_LOCATION
VGCS_Step2_CalculateMeanOfPredictedState(
	vgcs_data_s *pData_s);

static vgcs_fnc_status_e __VGCS_FNC_LOOP_MEMORY_LOCATION
VGCS_Step2_CalculateCovarianceOfPredictedState(
	vgcs_data_s *pData_s);

static vgcs_fnc_status_e __VGCS_FNC_LOOP_MEMORY_LOCATION
VGCS_Step3_PropagateEachSigmaPointThroughObservation(
	vgcs_data_s *pData_s);

static vgcs_fnc_status_e __VGCS_FNC_LOOP_MEMORY_LOCATION
VGCS_Step3_CalculateMeanOfPredictedOutput(
	vgcs_data_s *pData_s);

static vgcs_fnc_status_e __VGCS_FNC_LOOP_MEMORY_LOCATION
VGCS_Step3_CalculateCovarianceOfPredictedOutput(
	vgcs_data_s *pData_s);

static vgcs_fnc_status_e __VGCS_FNC_LOOP_MEMORY_LOCATION
VGCS_Step3_CalculateCrossCovarOfStateAndOut(
	vgcs_data_s *pData_s);

static vgcs_fnc_status_e __VGCS_FNC_LOOP_MEMORY_LOCATION
VGCS_Step4_CalcKalmanGain(
	vgcs_data_s *pData_s);

static vgcs_fnc_status_e __VGCS_FNC_LOOP_MEMORY_LOCATION
VGCS_Step4_UpdateStateEstimate(
	vgcs_data_s *pData_s);

static vgcs_fnc_status_e __VGCS_FNC_LOOP_MEMORY_LOCATION
VGCS_Step4_UpdateErrorCovariance(
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
	pUKF_s->dt = (__VGCS_FPT__) 0.0;

	/* Сброс в нуль: */
	size_t i;
	for (i = 0u; i < VGCS_LEN_STATE; i++)
	{
		pUKF_s->Q_mat_a[i] = (__VGCS_FPT__) 0.0;
		pUKF_s->R_mat_a[i] = (__VGCS_FPT__) 0.0;
		pUKF_s->state_a[i] = (__VGCS_FPT__) 0.0;
	}
}

void __VGCS_FNC_ONCE_MEMORY_LOCATION
VGSS_Init_MatrixStructs(
	vgcs_data_s 		*pData_s,
	ukfsif_all_data_s 	*pMatrixPointers_s)
{
	/* Объявление структуры для инициализации указателей на матрицы */
	ukfsif_all_data_init_s    initMatrixPointers_s;
	UKFIS_StructInit		(&initMatrixPointers_s);

	/* Инициализация матрицы шума Q */
	UKFMO_MatrixInit(
		&pData_s->noiseMatrix_s.QMat_s.mat_s,
		VGCS_LEN_MATRIX_ROW,
		VGCS_LEN_MATRIX_COL,
		pData_s->noiseMatrix_s.QMat_s.memForMatrix[0u]
	);
	__UKFMO_CheckMatrixSize(
		(ukfmo_matrix_s*)&pData_s->noiseMatrix_s.QMat_s.mat_s,
		sizeof(pData_s->noiseMatrix_s.QMat_s.memForMatrix));
	initMatrixPointers_s.pMatrix_s_a[UKFSIF_INIT_Q] =
		__VGCS_CheckMatrixStructValidation(
			&pData_s->noiseMatrix_s.QMat_s.mat_s);

	/* Инициализация матрицы шума R */
	UKFMO_MatrixInit(
		&pData_s->noiseMatrix_s.RMat_s.mat_s,
		VGCS_LEN_MATRIX_ROW,
		VGCS_LEN_MATRIX_COL,
		pData_s->noiseMatrix_s.RMat_s.memForMatrix[0u]
	);
	__UKFMO_CheckMatrixSize(
		(ukfmo_matrix_s*)&pData_s->noiseMatrix_s.RMat_s.mat_s,
		sizeof(pData_s->noiseMatrix_s.RMat_s.memForMatrix));
	initMatrixPointers_s.pMatrix_s_a[UKFSIF_INIT_R] =
		__VGCS_CheckMatrixStructValidation(
			&pData_s->noiseMatrix_s.RMat_s.mat_s);

	/* Инициализация матрицы пространства состояний */
	UKFMO_MatrixInit(
		&pData_s->stateMat_s.mat_s, 			/* !< Указатель на структуру матрицы */
		VGCS_LEN_MATRIX_ROW, 					/* !< Количество строк */
		VGCS_LEN_MATRIX_COL,					/* !< Количество столбцов */
		pData_s->stateMat_s.memForMatrix[0u] 	/* !< Указатель на область памяти для хранения матрицы */
	);
	__UKFMO_CheckMatrixSize(
		(ukfmo_matrix_s*)&pData_s->stateMat_s.mat_s,
		sizeof (pData_s->stateMat_s.memForMatrix));
	initMatrixPointers_s.pMatrix_s_a[UKFSIF_INIT_x_LxL] =
		__VGCS_CheckMatrixStructValidation(
			&pData_s->stateMat_s.mat_s);

	/* Инициализация вектора пространства состояний */
	UKFMO_MatrixInit(
		&pData_s->x_apriori_s.mat_s,
		VGCS_LEN_STATE,
		1u,
		pData_s->x_apriori_s.memForMatrix[0u]);
	__UKFMO_CheckMatrixSize(
		(ukfmo_matrix_s*)&pData_s->x_apriori_s.mat_s,
		sizeof(pData_s->x_apriori_s.memForMatrix));
	initMatrixPointers_s.pMatrix_s_a[UKFSIF_INIT_x_apriori] =
		__VGCS_CheckMatrixStructValidation(
			&pData_s->x_apriori_s.mat_s);

	UKFMO_MatrixInit(
		&pData_s->x_posteriori_s.mat_s,
		VGCS_LEN_STATE,
		1u,
		pData_s->x_posteriori_s.memForMatrix[0u]);
	__UKFMO_CheckMatrixSize(
		(ukfmo_matrix_s*)&pData_s->x_posteriori_s.mat_s,
		sizeof(pData_s->x_posteriori_s.memForMatrix));
	initMatrixPointers_s.pMatrix_s_a[UKFSIF_INIT_x_posteriori] =
		__VGCS_CheckMatrixStructValidation(
			&pData_s->x_posteriori_s.mat_s);

	/* Инициализация матрицы сигма-точек */
	UKFMO_MatrixInit(
		&pData_s->chiSigmaMat_s.mat_s,
		VGCS_LEN_SIGMA_ROW,
		VGCS_LEN_SIGMA_COL,
		pData_s->chiSigmaMat_s.memForMatrix[0u]
	);
	__UKFMO_CheckMatrixSize(
		(ukfmo_matrix_s*)&pData_s->chiSigmaMat_s.mat_s,
		sizeof(pData_s->chiSigmaMat_s.memForMatrix));
	initMatrixPointers_s.pMatrix_s_a[UKFSIF_INIT_chi_predict] =
		__VGCS_CheckMatrixStructValidation(
			&pData_s->chiSigmaMat_s.mat_s);

	/* Инициализация матрицы сигма-точек (после функции преобразования) */
	UKFMO_MatrixInit(
		&pData_s->chiSigmaPostMat_s.mat_s,
		VGCS_LEN_SIGMA_ROW,
		VGCS_LEN_SIGMA_COL,
		pData_s->chiSigmaPostMat_s.memForMatrix[0u]
	);
	__UKFMO_CheckMatrixSize(
		(ukfmo_matrix_s*)	&pData_s->chiSigmaPostMat_s.mat_s,
		sizeof				(pData_s->chiSigmaPostMat_s.memForMatrix));
	initMatrixPointers_s.pMatrix_s_a[UKFSIF_INIT_chi_apriori] =
		__VGCS_CheckMatrixStructValidation(
			&pData_s->chiSigmaPostMat_s.mat_s);

	/* Инициализация матрицы квадратного корня от матрицы ковариации "P" */
	UKFMO_MatrixInit(
		&pData_s->sqrtP_apriori_s.mat_s,
		VGCS_LEN_MATRIX_ROW,
		VGCS_LEN_MATRIX_COL,
		pData_s->sqrtP_apriori_s.memForMatrix[0u]
	);
	__UKFMO_CheckMatrixSize(
		(ukfmo_matrix_s*)&pData_s->sqrtP_apriori_s.mat_s,
		sizeof(pData_s->sqrtP_apriori_s.memForMatrix));
	initMatrixPointers_s.pMatrix_s_a[UKFSIF_INIT_P_sqrt] =
		__VGCS_CheckMatrixStructValidation(
			&pData_s->sqrtP_apriori_s.mat_s);

	UKFMO_MatrixInit(
		&pData_s->muMean_s.mat_s,
		VGCS_LEN_SIGMA_COL,
		1u,
		pData_s->muMean_s.memForMatrix[0u]
	);
	__UKFMO_CheckMatrixSize(
		(ukfmo_matrix_s*)&pData_s->muMean_s.mat_s,
		sizeof(pData_s->muMean_s.memForMatrix));
	initMatrixPointers_s.pMatrix_s_a[UKFSIF_INIT_muMean] =
		__VGCS_CheckMatrixStructValidation(
			&pData_s->muMean_s.mat_s);

	UKFMO_MatrixInit(
		&pData_s->muCovar_s.mat_s,
		VGCS_LEN_SIGMA_COL,
		1u,
		pData_s->muCovar_s.memForMatrix[0u]
	);
	__UKFMO_CheckMatrixSize(
		(ukfmo_matrix_s*)&pData_s->muCovar_s.mat_s,
		sizeof(pData_s->muCovar_s.memForMatrix));
	initMatrixPointers_s.pMatrix_s_a[UKFSIF_INIT_muCovar] =
		__VGCS_CheckMatrixStructValidation(
			&pData_s->muCovar_s.mat_s);

	UKFMO_MatrixInit(
		&pData_s->chi_apriory_minus_x_apriory_s.mat_s,
		VGCS_LEN_STATE,
		1u,
		pData_s->chi_apriory_minus_x_apriory_s.memForMatrix[0u]
	);
	__UKFMO_CheckMatrixSize(
		(ukfmo_matrix_s*)&pData_s->chi_apriory_minus_x_apriory_s.mat_s,
		sizeof (pData_s->chi_apriory_minus_x_apriory_s.memForMatrix));
	initMatrixPointers_s.pMatrix_s_a[UKFSIF_INIT_chi_priory_MINUS_x_priory] =
		__VGCS_CheckMatrixStructValidation(
			&pData_s->chi_apriory_minus_x_apriory_s.mat_s);

	UKFMO_MatrixInit(
		&pData_s->chi_apriory_minus_x_apriory_Transpose_s.mat_s,
		1u,
		VGCS_LEN_STATE,
		pData_s->chi_apriory_minus_x_apriory_Transpose_s.memForMatrix[0u]
	);
	__UKFMO_CheckMatrixSize(
		(ukfmo_matrix_s*) &pData_s->chi_apriory_minus_x_apriory_Transpose_s.mat_s,
		sizeof(pData_s->chi_apriory_minus_x_apriory_Transpose_s.memForMatrix));
	initMatrixPointers_s.pMatrix_s_a[UKFSIF_INIT_chi_priory_MINUS_x_priory_TRANSPOSE] =
		__VGCS_CheckMatrixStructValidation(
			&pData_s->chi_apriory_minus_x_apriory_Transpose_s.mat_s);

	UKFMO_MatrixInit(
		&pData_s->resultOfMult2Matrix_s.mat_s,
		VGCS_LEN_STATE,
		VGCS_LEN_STATE,
		pData_s->resultOfMult2Matrix_s.memForMatrix[0u]
	);
	__UKFMO_CheckMatrixSize(
		&pData_s->resultOfMult2Matrix_s.mat_s,
		sizeof(pData_s->resultOfMult2Matrix_s.memForMatrix));
	initMatrixPointers_s.pMatrix_s_a[UKFSIF_INIT_result_of_mult_2_matrix] =
		__VGCS_CheckMatrixStructValidation(
			&pData_s->resultOfMult2Matrix_s.mat_s);

	UKFMO_MatrixInit(
		&pData_s->P_apriori_s.mat_s,
		VGCS_LEN_MATRIX_ROW,
		VGCS_LEN_MATRIX_COL,
		pData_s->P_apriori_s.memForMatrix[0u]
	);
	__UKFMO_CheckMatrixSize(
		&pData_s->P_apriori_s.mat_s,
		sizeof(pData_s->P_apriori_s.memForMatrix));
	initMatrixPointers_s.pMatrix_s_a[UKFSIF_INIT_P_apriory] =
		__VGCS_CheckMatrixStructValidation(
			&pData_s->P_apriori_s.mat_s);

	UKFMO_MatrixInit(
		&pData_s->psi_apriori_s.mat_s,
		VGCS_LEN_SIGMA_ROW,
		VGCS_LEN_SIGMA_COL,
		pData_s->psi_apriori_s.memForMatrix[0u]
	);
	__UKFMO_CheckMatrixSize(
		&pData_s->psi_apriori_s.mat_s,
		sizeof(pData_s->psi_apriori_s.memForMatrix));
	initMatrixPointers_s.pMatrix_s_a[UKFSIF_INIT_psi_apriori] =
		__VGCS_CheckMatrixStructValidation(
			&pData_s->psi_apriori_s.mat_s);

	UKFMO_MatrixInit(
		&pData_s->y_apriori_s.mat_s,
		VGCS_LEN_STATE,
		1u,
		pData_s->y_apriori_s.memForMatrix[0u]
	);
	__UKFMO_CheckMatrixSize(
		&pData_s->y_apriori_s.mat_s,
		sizeof(pData_s->y_apriori_s.memForMatrix));
	initMatrixPointers_s.pMatrix_s_a[UKFSIF_INIT_y_apriori] =
		__VGCS_CheckMatrixStructValidation(
			&pData_s->y_apriori_s.mat_s);

	UKFMO_MatrixInit(
		&pData_s->Pyy_s.mat_s,
		VGCS_LEN_MATRIX_ROW,
		VGCS_LEN_MATRIX_COL,
		pData_s->Pyy_s.memForMatrix[0u]
	);
	__UKFMO_CheckMatrixSize(
		&pData_s->Pyy_s.mat_s,
		sizeof(pData_s->Pyy_s.memForMatrix));
	initMatrixPointers_s.pMatrix_s_a[UKFSIF_INIT_Pyy] =
		__VGCS_CheckMatrixStructValidation(
			&pData_s->Pyy_s.mat_s);

	UKFMO_MatrixInit(
		&pData_s->psi_priory_MINUS_y_priory.mat_s,
		VGCS_LEN_STATE,
		1u,
		pData_s->psi_priory_MINUS_y_priory.memForMatrix[0u]
	);
	__UKFMO_CheckMatrixSize(
		&pData_s->psi_priory_MINUS_y_priory.mat_s,
		sizeof(pData_s->psi_priory_MINUS_y_priory.memForMatrix));
	initMatrixPointers_s.pMatrix_s_a[UKFSIF_INIT_STEP2_psi_priory_MINUS_y_priory] =
		__VGCS_CheckMatrixStructValidation(
			&pData_s->psi_priory_MINUS_y_priory.mat_s);

	UKFMO_MatrixInit(
		&pData_s->Pxy_s.mat_s,
		VGCS_LEN_MATRIX_ROW,
		VGCS_LEN_MATRIX_COL,
		pData_s->Pxy_s.memForMatrix[0u]
	);
	__UKFMO_CheckMatrixSize(
		&pData_s->Pxy_s.mat_s,
		sizeof(pData_s->Pxy_s.memForMatrix));
	initMatrixPointers_s.pMatrix_s_a[UKFSIF_INIT_Pxy] =
		__VGCS_CheckMatrixStructValidation(
			&pData_s->Pxy_s.mat_s);

	UKFMO_MatrixInit(
		&pData_s->PyyInv_s.mat_s,
		VGCS_LEN_MATRIX_ROW,
		VGCS_LEN_MATRIX_COL,
		pData_s->PyyInv_s.memForMatrix[0u]);
	__UKFMO_CheckMatrixSize(
		&pData_s->PyyInv_s.mat_s,
		sizeof(pData_s->PyyInv_s.memForMatrix));
	initMatrixPointers_s.pMatrix_s_a[UKFSIF_INIT_Pyy_INV] =
		__VGCS_CheckMatrixStructValidation(
			&pData_s->PyyInv_s.mat_s);

	UKFMO_MatrixInit(
		&pData_s->K_s.mat_s,
		VGCS_LEN_MATRIX_ROW,
		VGCS_LEN_MATRIX_COL,
		pData_s->K_s.memForMatrix[0u]);
	__UKFMO_CheckMatrixSize(
		&pData_s->K_s.mat_s,
		sizeof(pData_s->K_s.memForMatrix));
	initMatrixPointers_s.pMatrix_s_a[UKFSIF_INIT_K] =
		__VGCS_CheckMatrixStructValidation(
			&pData_s->K_s.mat_s);

	UKFMO_MatrixInit(
		&pData_s->y_posteriori_s.mat_s,
		VGCS_LEN_MATRIX_ROW,
		1u,
		pData_s->y_posteriori_s.memForMatrix[0u]);
	__UKFMO_CheckMatrixSize(
		&pData_s->y_posteriori_s.mat_s,
		sizeof(pData_s->y_posteriori_s.memForMatrix));
	initMatrixPointers_s.pMatrix_s_a[UKFSIF_INIT_meas] =
		__VGCS_CheckMatrixStructValidation(
			&pData_s->y_posteriori_s.mat_s);

	UKFMO_MatrixInit(
		&pData_s->innovation_s.mat_s,
		VGCS_LEN_MATRIX_ROW,
		1u,
		pData_s->innovation_s.memForMatrix[0u]);
	__UKFMO_CheckMatrixSize(
		&pData_s->innovation_s.mat_s,
		sizeof(pData_s->innovation_s.memForMatrix));
	initMatrixPointers_s.pMatrix_s_a[UKFSIF_INIT_innovation] =
		__VGCS_CheckMatrixStructValidation(
			&pData_s->innovation_s.mat_s);

	UKFMO_MatrixInit(
		&pData_s->P_predict_s.mat_s,
		VGCS_LEN_MATRIX_ROW,
		VGCS_LEN_MATRIX_COL,
		pData_s->P_predict_s.memForMatrix[0u]);
	__UKFMO_CheckMatrixSize(
		&pData_s->P_predict_s.mat_s,
		sizeof(pData_s->P_predict_s.memForMatrix));
	initMatrixPointers_s.pMatrix_s_a[UKFSIF_INIT_P] =
		__VGCS_CheckMatrixStructValidation(
			&pData_s->P_predict_s.mat_s);

	UKFMO_MatrixInit(
		&pData_s->K_Transpose_s.mat_s,
		VGCS_LEN_MATRIX_ROW,
		VGCS_LEN_MATRIX_COL,
		pData_s->K_Transpose_s.memForMatrix[0u]);
	__UKFMO_CheckMatrixSize(
		&pData_s->K_Transpose_s.mat_s,
		sizeof(pData_s->K_Transpose_s.memForMatrix));
	initMatrixPointers_s.pMatrix_s_a[UKFSIF_INIT_K_TRANSPOSE] =
		__VGCS_CheckMatrixStructValidation(
			&pData_s->K_Transpose_s.mat_s);

	UKFMO_MatrixInit(
		&pData_s->x_predict_temp_s.mat_s,
		VGCS_LEN_MATRIX_ROW,
		VGCS_LEN_MATRIX_COL,
		pData_s->x_predict_temp_s.memForMatrix[0u]);
	__UKFMO_CheckMatrixSize(
		&pData_s->x_predict_temp_s.mat_s,
		sizeof(pData_s->x_predict_temp_s.memForMatrix));
	initMatrixPointers_s.pMatrix_s_a[UKFSIF_INIT_x_LxL_TEMP] =
		__VGCS_CheckMatrixStructValidation(
			&pData_s->x_predict_temp_s.mat_s);

	UKFMO_MatrixInit(
		&pData_s->x_predict_temp_ones_s.mat_s,
		1u,
		VGCS_LEN_MATRIX_COL,
		pData_s->x_predict_temp_ones_s.memForMatrix[0u]);
	__UKFMO_CheckMatrixSize(
		&pData_s->x_predict_temp_ones_s.mat_s,
		sizeof(pData_s->x_predict_temp_ones_s.memForMatrix));
	initMatrixPointers_s.pMatrix_s_a[UKFSIF_INIT_x_1xL_ones_TEMP] =
		__VGCS_CheckMatrixStructValidation(
			&pData_s->x_predict_temp_ones_s.mat_s);


	/* Копирование указателей на структуры матриц (Эта функция должна быть
	 * вызвана в конце) */
	UKFSIF_Init_SetMatrixPointers(
		pMatrixPointers_s,
		&initMatrixPointers_s,
		(uint16_t) VGCS_LEN_STATE);
}

static void __VGCS_FNC_ONCE_MEMORY_LOCATION
VGCS_Init_NoiseMatrix(
	ukfmo_matrix_s 	*pNoiseMat,
	__VGCS_FPT__ 	*pNoiseMatDiag)
{
	/* Сброс матрицы шумов в нуль */
	UKFMO_MatrixZeros(pNoiseMat);

	size_t i;
	for (i = 0u; i < pNoiseMat->numCols; i++)
	{
		if (*pNoiseMatDiag == ((__VGCS_FPT__) 0.0))
		{
			/* Если попоали сюда, значит диагональ матрицы шума не
			 * инициализирована */
			while (1);
		}
		pNoiseMat->pData[__UKFMO_GetIndexInOneFromTwoDim(pNoiseMat, i, i)] =
			*pNoiseMatDiag++;
	}
}

/*-------------------------------------------------------------------------*//**
 * @author    Mickle Isaev
 * @date      03-сен-2019
 *
 * @brief    Функция выполняет инициализацию всех параметров, необходимых
 *           для работы UKF
 *
 * @param[out] 	*pData_s:	Указатель на структуру данных, в которой содержаться
 * 							параметры, необходимые для работы UKF
 * @param[in]   *pInit_s:   Указатель на структуру, которая содержит
 * 							параметры для инициализации UKF
 *
 * @return  None
 */
void __VGCS_FNC_ONCE_MEMORY_LOCATION
VGSS_Init_All(
	vgcs_data_s 		*pData_s,
	vgcs_data_init_s 	*pInit_s)
{
	/* Инициализация всех структур матриц */
	VGSS_Init_MatrixStructs(
		pData_s,
		&pData_s->ukfsifMatrixPointers_s);

	/* Вычисление корня квадратного из (lambda + len) и запись в поле структуры */
	pData_s->scalar_s.sqrtLamLen =
		__VGCS_sqrt(
			UKFSIF_GetLambda(
				VGCS_LEN_STATE,
				pInit_s->scalParams_s.alpha,
				pInit_s->scalParams_s.kappa) + VGCS_LEN_STATE);

	/* Установка периода интегрирования */
	#if defined (__UKFMO_CHEKING_ENABLE__)
	if (pInit_s->dt == (__VGCS_FPT__)0.0)
	{
		__UKFMO_ALL_INTERRUPTS_DIS();
		while (1);
	}
	#endif

	/* Обновление периода интегрирования */
	__VGCS_UpdateDt(pData_s, pInit_s->dt);

	/* Инициализация вектора muMean */
	UKFSIF_InitWeightVectorMean(
		&pInit_s->scalParams_s,
		pData_s->muMean_s.memForMatrix[0u],
		VGCS_LEN_STATE);

	/* Инициализация вектора muCov */
	UKFSIF_InitWeightVectorCov(
		&pInit_s->scalParams_s,
		pData_s->muCovar_s.memForMatrix[0u],
		VGCS_LEN_STATE);

	/* Заполнение матрицы Q */
	VGCS_Init_NoiseMatrix(
		&pData_s->noiseMatrix_s.QMat_s.mat_s,
		pInit_s->Q_mat_a);

	/* Заполнение матрицы R */
	VGCS_Init_NoiseMatrix(
		&pData_s->noiseMatrix_s.RMat_s.mat_s,
		pInit_s->R_mat_a);

	/* Заполнение матрицы P */
	UKFMO_MatrixIdentity(
		&pData_s->P_predict_s.mat_s);

	/* Инициализация вектора пространства состояний */
	size_t i;
	for (i = 0u; i < __UKFMO_GetRowNumb(&pData_s->x_posteriori_s.mat_s); i++)
	{
		/* Получить индекс ячейки массива */
		size_t idx =
			__UKFMO_GetIndexInOneFromTwoDim(&pData_s->x_posteriori_s.mat_s, i, 0u);
		pData_s->x_posteriori_s.mat_s.pData[idx]
			= pInit_s->state_a[i];
	}

}

/*-------------------------------------------------------------------------*//**
 * @author    Mickle Isaev
 * @date      03-сен-2019
 *
 * @brief    Функция выполняет все шаги, предусмотренные алгоритмом UKF
 *
 * @param[in,out] 	*pData_s:	Указатель на структуру данных, в которой содержаться
 * 								параметры, необходимые для работы UKF
 *
 * @return 	Статус функции
 *                 @see "vgcs_fnc_status_e"
 */
vgcs_fnc_status_e __VGCS_FNC_LOOP_MEMORY_LOCATION
VGCS_UKF_UpdateVectState(
	vgcs_data_s *pData_s)
{
	#if defined (__UKFMO_CHEKING_ENABLE__)
	ukfmo_fnc_status_e matOperationStatus_e = UKFMO_OK;
	#endif

	/* Step 1 ################################################################ */
	/* Были ли получены новые данные от акселерометра */
	if (__VGCS_IsFlagAccDataUpdateSet() == 1u)
	{
		/* Сброс флага */
		__VGCS_ReSetFlagAccDataUpdate();


		/* Calculate error covariance matrix square root */
		#if defined (__UKFMO_CHEKING_ENABLE__)
		matOperationStatus_e =
		#endif
			VGCS_Step1_CalculateErrorCovarianceMatrixSquareRoot(
				pData_s);

		/* Calculate the sigma-points */
		#if defined (__UKFMO_CHEKING_ENABLE__)
		matOperationStatus_e =
		#endif
			VGCS_Step1_GeterateTheSigmaPoints(
				pData_s);

		/* Step 2 ################################################################ */
		/* Propagate each sigma-point through prediction */
		#if defined (__UKFMO_CHEKING_ENABLE__)
		matOperationStatus_e =
		#endif
			VGCS_Step2_ProragateEachSigmaPointsThroughPrediction(
				pData_s);

		/* Calculate mean of predicted state */
		#if defined (__UKFMO_CHEKING_ENABLE__)
		matOperationStatus_e =
		#endif
			VGCS_Step2_CalculateMeanOfPredictedState(
				pData_s);

		/* Calculate covariance of predicted state  */
		#if defined (__UKFMO_CHEKING_ENABLE__)
		matOperationStatus_e =
		#endif
			VGCS_Step2_CalculateCovarianceOfPredictedState(
				pData_s);
	}

	/* Step 3 ################################################################ */
	/* Если были приняты новые данные от GNSS модуля */
	if (__VGCS_IsFlagGNSSDataUpdateSet() == 1u)
	{
		/* Сброс флага */
		__VGCS_ReSetFlagGNSSDataUpdate();


		/* Propagate each sigma-point through observation */
		#if defined (__UKFMO_CHEKING_ENABLE__)
		matOperationStatus_e =
		#endif
			VGCS_Step3_PropagateEachSigmaPointThroughObservation(
				pData_s);

		/* Calculate mean of predicted output */
		#if defined (__UKFMO_CHEKING_ENABLE__)
		matOperationStatus_e =
		#endif
			VGCS_Step3_CalculateMeanOfPredictedOutput(
				pData_s);

		/* Calculate covariance of predicted output */
		#if defined (__UKFMO_CHEKING_ENABLE__)
		matOperationStatus_e =
		#endif
			VGCS_Step3_CalculateCovarianceOfPredictedOutput(
				pData_s);

		/* Calculate cross-covariance of state and output */
		#if defined (__UKFMO_CHEKING_ENABLE__)
		matOperationStatus_e =
		#endif
			VGCS_Step3_CalculateCrossCovarOfStateAndOut(
				pData_s);

		/* Step 4 ################################################################ */
		/* Calculate Kalman gain */
		#if defined (__UKFMO_CHEKING_ENABLE__)
		matOperationStatus_e =
		#endif
			VGCS_Step4_CalcKalmanGain(
				pData_s);

		/* Update state estimate */
		#if defined (__UKFMO_CHEKING_ENABLE__)
		matOperationStatus_e =
		#endif
			VGCS_Step4_UpdateStateEstimate(
				pData_s);

		/* Update error covariance */
		#if defined (__UKFMO_CHEKING_ENABLE__)
		matOperationStatus_e =
		#endif
			VGCS_Step4_UpdateErrorCovariance(
				pData_s);
	}

	#if defined (__UKFMO_CHEKING_ENABLE__)
	return (matOperationStatus_e);
	#else
	return (UKFMO_OK);
	#endif
}

/*#### |End  | <-- Секция - "Описание глобальных функций" ####################*/


/*#### |Begin| --> Секция - "Создание задач" #################################*/
/*#### |End  | <-- Секция - "Создание задач" #################################*/


/*#### |Begin| --> Секция - "Описание локальных функций" #####################*/

static vgcs_fnc_status_e __VGCS_FNC_LOOP_MEMORY_LOCATION
VGCS_Step1_CalculateErrorCovarianceMatrixSquareRoot(
	vgcs_data_s *pData_s)
{
	#if defined (__UKFMO_CHEKING_ENABLE__)
	ukfmo_fnc_status_e matOperationStatus_e;
	#endif

	/* Копирование матрицы P в матрицу SQRT_P */
	#if defined (__UKFMO_CHEKING_ENABLE__)
	matOperationStatus_e =
	#endif
		UKFMO_CopyMatrix(
			__VGCS_CheckMatrixStructValidation(
				&pData_s->sqrtP_apriori_s.mat_s),
			__VGCS_CheckMatrixStructValidation(
				&pData_s->P_predict_s.mat_s));

	#if defined (__UKFMO_CHEKING_ENABLE__)
	matOperationStatus_e =
	#endif
		UKFMO_GetCholeskyLow(
			__VGCS_CheckMatrixStructValidation(
				&pData_s->sqrtP_apriori_s.mat_s));

	#if defined (__UKFMO_CHEKING_ENABLE__)
	return (matOperationStatus_e);
	#else
	return (UKFMO_OK);
	#endif
}

static vgcs_fnc_status_e __VGCS_FNC_LOOP_MEMORY_LOCATION
VGCS_Step1_GeterateTheSigmaPoints(
	vgcs_data_s *pData_s)
{
	/* Calculate the sigma-points */
	UKFSIF_Step1_CalculateTheSigmaPoints(
		&pData_s->ukfsifMatrixPointers_s.calcTheSigmaPoints_s,
		pData_s->scalar_s.sqrtLamLen);

	return (UKFMO_OK);
}

static vgcs_fnc_status_e __VGCS_FNC_LOOP_MEMORY_LOCATION
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

static vgcs_fnc_status_e __VGCS_FNC_LOOP_MEMORY_LOCATION
VGCS_Step2_CalculateMeanOfPredictedState(
	vgcs_data_s *pData_s)
{
	UKFSIF_Step2_CalculateMeanOfPredictedState(
		&pData_s->ukfsifMatrixPointers_s.calcMeanOfPredictState_s);

	return (UKFMO_OK);
}

static vgcs_fnc_status_e __VGCS_FNC_LOOP_MEMORY_LOCATION
VGCS_Step2_CalculateCovarianceOfPredictedState(
	vgcs_data_s *pData_s)
{
	UKFSIF_Step2_CalculateCovarianceOfPredictedState(
		&pData_s->ukfsifMatrixPointers_s.calcCovarOfPredictState_s);

	return (UKFMO_OK);
}

static vgcs_fnc_status_e __VGCS_FNC_LOOP_MEMORY_LOCATION
VGCS_Step3_PropagateEachSigmaPointThroughObservation(
	vgcs_data_s *pData_s)
{
	#if defined (__UKFMO_CHEKING_ENABLE__)
	ukfmo_fnc_status_e matOperationStatus_e;
	#endif

	/* Т.к. матрица psi_k|k-1 полностью соответствует матрице chi_k|k-1, то
	 * выполним копирование матрицы без преобразования */
	#if defined (__UKFMO_CHEKING_ENABLE__)
	matOperationStatus_e =
	#endif
		UKFMO_CopyMatrix(
			__VGCS_CheckMatrixStructValidation(&pData_s->psi_apriori_s.mat_s),
			__VGCS_CheckMatrixStructValidation(&pData_s->chiSigmaMat_s.mat_s));

	#if defined (__UKFMO_CHEKING_ENABLE__)
	return (matOperationStatus_e);
	#else
	return (UKFMO_OK);
	#endif
}

static vgcs_fnc_status_e __VGCS_FNC_LOOP_MEMORY_LOCATION
VGCS_Step3_CalculateMeanOfPredictedOutput(
	vgcs_data_s *pData_s)
{
	UKFSIF_Step3_CalculateMeanOfPredictedOutput(
		&pData_s->ukfsifMatrixPointers_s.caclMeanOfPredictOut_s);

	return (UKFMO_OK);
}

static vgcs_fnc_status_e __VGCS_FNC_LOOP_MEMORY_LOCATION
VGCS_Step3_CalculateCovarianceOfPredictedOutput(
	vgcs_data_s *pData_s)
{
	UKFSIF_Step3_CalculateCovarianceOfPredictedOutput(
		&pData_s->ukfsifMatrixPointers_s.caclCovarOfPredictOut_s);

	return (UKFMO_OK);
}

static vgcs_fnc_status_e __VGCS_FNC_LOOP_MEMORY_LOCATION
VGCS_Step3_CalculateCrossCovarOfStateAndOut(
	vgcs_data_s *pData_s)
{
	UKFSIF_Step3_CalculateCrossCovarOfStateAndOut(
		&pData_s->ukfsifMatrixPointers_s.calcCrossCovarOfStateAndOut_s);

	return (UKFMO_OK);
}

static vgcs_fnc_status_e __VGCS_FNC_LOOP_MEMORY_LOCATION
VGCS_Step4_CalcKalmanGain(
	vgcs_data_s *pData_s)
{
	UKFSIF_Step4_CalcKalmanGain(
		&pData_s->ukfsifMatrixPointers_s.calcKalmanGain_s);

	return (UKFMO_OK);
}

static vgcs_fnc_status_e __VGCS_FNC_LOOP_MEMORY_LOCATION
VGCS_Step4_UpdateStateEstimate(
	vgcs_data_s *pData_s)
{
	UKFSIF_Step4_UpdateStateEstimate(
		&pData_s->ukfsifMatrixPointers_s.updateState_s);

	return (UKFMO_OK);
}

static vgcs_fnc_status_e __VGCS_FNC_LOOP_MEMORY_LOCATION
VGCS_Step4_UpdateErrorCovariance(
	vgcs_data_s *pData_s)
{
	UKFSIF_Step4_UpdateErrorCovariance(
		&pData_s->ukfsifMatrixPointers_s.updateErrCov_s);

	return (UKFMO_OK);
}
/*#### |End  | <-- Секция - "Описание локальных функций" #####################*/


/*############################################################################*/
/*############################ END OF FILE  ##################################*/
/*############################################################################*/
