#include "Madgwick.h"

#define deltaT 0.01f
#define GYRO_LSB 65.5f
#define ACCEL_LSB 8192.0f
#define MAG_LSB 1090.0f
#define DEG2RAD    (3.14159265359f/180.0f)
#define gravity 9.81f
#define Ga2miuT 100.0f
//#define gyroError 0.005f*sqrt(50.0f)
#define beta_max 1.5f
#define beta_min 0.1f
#define zeta 0.005f
//#define beta sqrt(3.0f/4.0f)*gyroError
//#define zeta sqrt(3.0f/4.0f)*gyroGrift

// mag_bias correction from hard and soft iron error
float mag_harderr[3] = {134.6028, -246.5224, 41.6337};
float mag_softerr[3][3] = {
    { 1.0000, -0.00015,  0.00031},
    { -0.00015,  1.0000, -0.00003},
    {0.00031,  -0.00003,  1.0000}
};

/**
 * @brief Madgwick姿态融合算法更新函数
 * 
 * @param accdata 加速度计数据，包含X、Y、Z三轴加速度
 * @param gyrodata 陀螺仪数据，包含X、Y、Z三轴角速度
 * @param magdata 磁力计数据，包含X、Y、Z三轴磁场强度
 * @param Q 四元数输出，包含q1、q2、q3、q4四个分量
 * @param b_x 参考磁场在X方向的分量
 * @param b_z 参考磁场在Z方向的分量
 * 
 * @note Madgwick算法实现步骤：
 *       1. 归一化加速度计和磁力计数据
 *       2. 陀螺仪数据单位转换（度/秒 转 弧度/秒）
 *       3. 计算目标函数
 *       4. 计算雅可比矩阵
 *       5. 计算并归一化梯度
 *       6. 计算陀螺仪四元数微分
 *       7. 更新并归一化四元数
 *       8. 计算下一时刻的参考磁场
 */
void MadgwickUpdate(MPU6050_AccDataTypeDef *accdata, MPU6050_GyroDataTypeDef *gyrodata, HMC5883L_DataTypeDef *magdata, SEQTypeDef *Q)
{
    const float acc_bias = 0.1f;
    const float mag_bias = 0.3f;
    bool useAcc = true, useMag = true;

    static float b_x = 50.0f, b_z = 0.0f;
    static float b_normx = 1.0f, b_normz = 0.0f;
    static float wBias_x = 0.0f, wBias_y = 0.0f, wBias_z = 0.0f;
    static float beta = 0.0f;
    float q1 = Q->q1, q2 = Q->q2, q3 = Q->q3, q4 = Q->q4;
    float a_x = accdata->Acc_X, a_y = accdata->Acc_Y, a_z = accdata->Acc_Z;
    float w_x = gyrodata->Gyro_X, w_y = gyrodata->Gyro_Y, w_z = gyrodata->Gyro_Z;
    float m_x = magdata->Mag_X, m_y = magdata->Mag_Y, m_z = magdata->Mag_Z;
    float norm;

    // acc to g
    a_x /= ACCEL_LSB;
    a_y /= ACCEL_LSB;
    a_z /= ACCEL_LSB;
    a_x *= gravity;
    a_y *= gravity;
    a_z *= gravity;

    // check if acc is usable
    norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
    if(fabs(norm - gravity) / gravity > acc_bias)  useAcc = false;
    else useAcc = true;
    
	// normalise acc data
	a_x /= norm;
	a_y /= norm;
	a_z /= norm;

//    m_x -= mag_harderr[0];
//    m_y -= mag_harderr[1];
//    m_z -= mag_harderr[2];

    // remove hard iron error
    float mx_cal = m_x - mag_harderr[0];
    float my_cal = m_y - mag_harderr[1];
    float mz_cal = m_z - mag_harderr[2];
    // remove soft iron error
    m_x = mag_softerr[0][0] * mx_cal + mag_softerr[0][1] * my_cal + mag_softerr[0][2] * mz_cal;
    m_y = mag_softerr[1][0] * mx_cal + mag_softerr[1][1] * my_cal + mag_softerr[1][2] * mz_cal;
    m_z = mag_softerr[2][0] * mx_cal + mag_softerr[2][1] * my_cal + mag_softerr[2][2] * mz_cal;

//	float X_raw = m_x;
//	float Y_raw = m_y;
//	float Z_raw = m_z;

//    m_x = (X_raw - 3.5) * 0.9615;
//    m_y = (Y_raw + 106.5) * 1.1106;
//    m_z = (Z_raw + 11.5) * 0.9438;

    // mag Ga to miuT
    m_x /= MAG_LSB;
    m_y /= MAG_LSB;
    m_z /= MAG_LSB;
    m_x *= Ga2miuT;
    m_y *= Ga2miuT;
    m_z *= Ga2miuT;

    // check if mag is usable
    float b_norm = sqrt(b_x * b_x + b_z * b_z);
    norm = sqrt(m_x * m_x + m_y * m_y + m_z * m_z);
    if(fabs(norm - b_norm) / b_norm > mag_bias)  useMag = false;
    else useMag = true;


	// normalise mag data
	float m_normx = m_x / norm;
	float m_normy = m_y / norm;
	float m_normz = m_z / norm;


    // gyro °/s to rad/s
    w_x /= GYRO_LSB;
    w_y /= GYRO_LSB;
    w_z /= GYRO_LSB;
    w_x *= DEG2RAD;
    w_y *= DEG2RAD;
    w_z *= DEG2RAD;

    // use gradient descent
    if(useAcc || useMag){
        float n_1 = 0, n_2 = 0, n_3 = 0, n_4 = 0;

        // compute the objective function
        float f_1 = 2.0f * (q2 * q4 - q1 * q3) - a_x;
        float f_2 = 2.0f * (q1 * q2 + q3 * q4) - a_y;
        float f_3 = 2.0f * (0.5f - q2 * q2 - q3 * q3) - a_z;
        float f_4 = 2.0f * b_normx * (0.5f - q3 * q3 - q4 * q4) + 2.0f * b_normz * (q2 * q4 - q1 * q3) - m_normx;
        float f_5 = 2.0f * b_normx * (q2 * q3 - q1 * q4) + 2.0f * b_normz * (q1 * q2 + q3 * q4) - m_normy;
        float f_6 = 2.0f * b_normx * (q1 * q3 + q2 * q4) + 2.0f * b_normz * (0.5f - q2 * q2 - q3 * q3) - m_normz;

        // compute the Jacobian matrix
        float j_11 = -2.0f * q3;
        float j_12 = 2.0f * q4;
        float j_13 = -2.0f * q1;
        float j_14 = 2.0f * q2;
        float j_21 = 2.0f * q2;
        float j_22 = 2.0f * q1;
        float j_23 = 2.0f * q4;
        float j_24 = 2.0f * q3;
        float j_32 = -4.0f * q2;
        float j_33 = -4.0f * q3;
        float j_41 = -2.0f * b_normz * q3;
        float j_42 = 2.0f * b_normz * q4;
        float j_43 = -2.0f * b_normz * q1 - 4.0f * b_normx * q3;
        float j_44 = 2.0f * b_normz * q2 - 4.0f * b_normx * q4;
        float j_51 = -2.0f * b_normx * q4 + 2.0f * b_normz * q2;
        float j_52 = 2.0f * b_normx * q3 + 2.0f * b_normz * q1;
        float j_53 = 2.0f * b_normx * q2 + 2.0f * b_normz * q4;
        float j_54 = -2.0f * b_normx * q1 + 2.0f * b_normz * q3;
        float j_61 = 2.0f * b_normx * q3;
        float j_62 = 2.0f * b_normx * q4 - 4.0f * b_normz * q2;
        float j_63 = 2.0f * b_normx * q1 - 4.0f * b_normz * q3;
        float j_64 = 2.0f * b_normx * q2;

        // compute and normalize the gradient JTf
        if(useAcc && useMag){
            // gradient from acc and mag
            n_1 = (j_11 * f_1) + (j_21 * f_2) + (j_41 * f_4) + (j_51 * f_5) + (j_61 * f_6);
            n_2 = (j_12 * f_1) + (j_22 * f_2) + (j_32 * f_3) + (j_42 * f_4) + (j_52 * f_5) + (j_62 * f_6);
            n_3 = (j_13 * f_1) + (j_23 * f_2) + (j_33 * f_3) + (j_43 * f_4) + (j_53 * f_5) + (j_63 * f_6);
            n_4 = (j_14 * f_1) + (j_24 * f_2) + (j_44 * f_4) + (j_54 * f_5) + (j_64 * f_6);
        }else{
            // gradient from acc only
            n_1 = (j_11 * f_1) + (j_21 * f_2);
            n_2 = (j_12 * f_1) + (j_22 * f_2) + (j_32 * f_3);
            n_3 = (j_13 * f_1) + (j_23 * f_2) + (j_33 * f_3);
            n_4 = (j_14 * f_1) + (j_24 * f_2);
        }
        norm = sqrt(n_1 * n_1 + n_2 * n_2 + n_3 * n_3 + n_4 * n_4);
        
        // 根据运动情况动态调整beta
        if(norm < 0.05f){
            beta = beta_min;
        }
        else{
            beta = beta_max;
        }

    //	printf("Graid:%f  beta:%f\r\n", norm, beta);
        
        n_1 /= norm;
        n_2 /= norm;
        n_3 /= norm;
        n_4 /= norm;
        
        if(useAcc && useMag){
            // compute the dynamic bias of gyro
            wBias_x += 2 * (-q2 * n_1 + q1 * n_2 + q4 * n_3 - q3 * n_4) * deltaT;
            wBias_y += 2 * (-q3 * n_1 - q4 * n_2 + q1 * n_3 + q2 * n_4) * deltaT;
            wBias_z += 2 * (-q4 * n_1 + q3 * n_2 - q2 * n_3 + q1 * n_4) * deltaT;

            w_x -= zeta * wBias_x;
            w_y -= zeta * wBias_y;
            w_z -= zeta * wBias_z;
        }
        
        // compute the quaternion rate by gyro
        float qDot1 = 0.5f * (-q2 * w_x - q3 * w_y - q4 * w_z);
        float qDot2 = 0.5f * (q1 * w_x + q3 * w_z - q4 * w_y);
        float qDot3 = 0.5f * (q1 * w_y - q2 * w_z + q4 * w_x);
        float qDot4 = 0.5f * (q1 * w_z + q2 * w_y - q3 * w_x);

        // update and normalize the quaternion
        q1 += deltaT * (qDot1 - beta * n_1);
        q2 += deltaT * (qDot2 - beta * n_2);
        q3 += deltaT * (qDot3 - beta * n_3);
        q4 += deltaT * (qDot4 - beta * n_4);
    }
    // only use gyro
    else{
        float qDot1 = 0.5f * (-q2 * w_x - q3 * w_y - q4 * w_z);
        float qDot2 = 0.5f * (q1 * w_x + q3 * w_z - q4 * w_y);
        float qDot3 = 0.5f * (q1 * w_y - q2 * w_z + q4 * w_x);
        float qDot4 = 0.5f * (q1 * w_z + q2 * w_y - q3 * w_x);

        q1 += deltaT * qDot1;
        q2 += deltaT * qDot2;
        q3 += deltaT * qDot3;
        q4 += deltaT * qDot4;
    }

    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    Q->q1 = q1 / norm;
    Q->q2 = q2 / norm;
    Q->q3 = q3 / norm;
    Q->q4 = q4 / norm;

    //compute magnetometer flux in the earth frame for next time
    float x = 2.0f * m_x * (q1 * q1 + q2 * q2 - 0.5f) + 2.0f * m_y * (q2 * q3 - q1 * q4) + 2.0f * m_z * (q2 * q4 + q1 * q3);
    float y = 2.0f * m_x * (q2 * q3 + q1 * q4) + 2.0f * m_y * (q1 * q1 + q3 * q3 - 0.5f) + 2.0f * m_z * (q3 * q4 - q1 * q2);
    float z = 2.0f * m_x * (q2 * q4 - q1 * q3) + 2.0f * m_y * (q3 * q4 + q1 * q2) + 2.0f * m_z * (q1 * q1 + q4 * q4 - 0.5f);
	b_x = sqrt(x * x + y * y);
	b_z = z;

    norm = sqrt(b_x * b_x + b_z * b_z);
    b_normx = b_x / norm;
    b_normz = b_z / norm;
    
//    printf("mx:%f  my:%f  mz:%f\r\n", m_x, m_y, m_z);
//    printf("bx:%f  bz:%f\r\n", b_x, b_z);
}

/**
 * @brief 将四元数转换为欧拉角
 * @param Q 输入的四元数结构体指针，包含q1、q2、q3、q4四个分量
 * @param roll 输出参数，指向存储横滚角的变量指针
 * @param pitch 输出参数，指向存储俯仰角的变量指针
 * @param yaw 输出参数，指向存储偏航角的变量指针
 * @note 使用标准的四元数到欧拉角转换公式
 *       roll: 绕X轴旋转角度
 *       pitch: 绕Y轴旋转角度
 *       yaw: 绕Z轴旋转角度
 *       所有角度单位为弧度
 */
void Quaternion2Euler(SEQTypeDef *Q, float *roll, float *pitch, float *yaw)
{
    *roll = atan2(2.0f * (Q->q1 * Q->q2 + Q->q3 * Q->q4), 1.0f - 2.0f * (Q->q2 * Q->q2 + Q->q3 * Q->q3));
    *pitch = asin(2.0f * (Q->q1 * Q->q3 - Q->q4 * Q->q2));
    *yaw = atan2(2.0f * (Q->q1 * Q->q4 + Q->q2 * Q->q3), 1.0f - 2.0f * (Q->q3 * Q->q3 + Q->q4 * Q->q4));
}
