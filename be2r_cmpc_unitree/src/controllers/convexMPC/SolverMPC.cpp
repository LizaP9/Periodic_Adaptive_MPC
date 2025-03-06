#include "SolverMPC.h"
#include "RobotState.h"
#include "common_types.h"
#include "convexMPC_interface.h"
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <JCQP/QpProblem.h>
#include <Utilities/Timer.h>
#include <qpOASES/include/qpOASES.hpp>
#include <stdio.h>
#include <sys/time.h>
#include <chrono>
#include <ros/ros.h>




#define BIG_NUMBER 5e10 // big enough to act like infinity, small enough to avoid numerical weirdness.

RobotState rs;
using Eigen::Dynamic;
using std::cout;
using std::endl;

Matrix<fpt, Dynamic, 13> A_qp;
Matrix<fpt, Dynamic, Dynamic> B_qp;
Matrix<fpt, 13, 12> Bdt;
Matrix<fpt, 13, 13> Adt;

// ------------------------ИЗМЕНЕНО------------------------------------
//Дополнительные глобальные матрицы:
Matrix<fpt, 13, 6> Qdt;               // дискретизированная матрица Q
Matrix<fpt, Dynamic, 6> Q_qp;         // (13*h) x 6 : вклад сил f в каждое из x_k

Matrix<fpt, 31, 31> ABc, expmm; //Matrix<fpt, 25, 25> ABc, expmm; - была размерность
// ------------------------ИЗМЕНЕНО------------------------------------

Matrix<fpt, Dynamic, Dynamic> S;
Matrix<fpt, Dynamic, 1> X_d;
Matrix<fpt, Dynamic, 1> U_b;
Matrix<fpt, Dynamic, Dynamic> fmat;

Matrix<fpt, Dynamic, Dynamic> qH;
Matrix<fpt, Dynamic, 1> qg;

Matrix<fpt, Dynamic, Dynamic> eye_12h;

qpOASES::real_t* H_qpoases;
qpOASES::real_t* g_qpoases;
qpOASES::real_t* A_qpoases;
qpOASES::real_t* lb_qpoases;
qpOASES::real_t* ub_qpoases;
qpOASES::real_t* q_soln;

qpOASES::real_t* H_red;
qpOASES::real_t* g_red;
qpOASES::real_t* A_red;
qpOASES::real_t* lb_red;
qpOASES::real_t* ub_red;
qpOASES::real_t* q_red;
u8 real_allocated = 0;

char var_elim[2000];
char con_elim[2000];

mfp* get_q_soln()
{
  return q_soln;
}

s8 near_zero(fpt a)
{
  return (a < 0.01 && a > -.01);
}

s8 near_one(fpt a)
{
  return near_zero(a - 1);
}
void matrix_to_real(qpOASES::real_t* dst, Matrix<fpt, Dynamic, Dynamic> src, s16 rows, s16 cols)
{
  s32 a = 0;
  for (s16 r = 0; r < rows; r++)
  {
    for (s16 c = 0; c < cols; c++)
    {
      dst[a] = src(r, c);
      a++;
    }
  }
}

// ------------------------ИЗМЕНЕНО------------------------------------
//добавлена передача матрицы Qc в функцию
void c2qp(Matrix<fpt, 13, 13> Ac, Matrix<fpt, 13, 12> Bc, Matrix<fpt, 13, 6> Qc, fpt dt, s16 horizon)
{
  ABc.setZero();
  ABc.block(0, 0, 13, 13) = Ac;
  ABc.block(0, 13, 13, 12) = Bc;
  ABc.block(0, 25, 13, 6) = Qc; // NEW: для Q

  ABc = dt * ABc;
  expmm = ABc.exp();
  Adt = expmm.block(0, 0, 13, 13);
  Bdt = expmm.block(0, 13, 13, 12);
  Qdt = expmm.block(0, 25, 13, 6);  // NEW дискретизированная Q

#ifdef K_PRINT_EVERYTHING
  cout << "Adt: \n" << Adt << "\nBdt:\n" << Bdt << "\nQdt:\n" << Qdt << endl;
#endif

  if (horizon > 19)
  {
    throw std::runtime_error("horizon is too long!");
  }

  Matrix<fpt, 13, 13> powerMats[20];
  powerMats[0].setIdentity();

  for (int i = 1; i < horizon + 1; i++)
  {
    powerMats[i] = Adt * powerMats[i - 1];
  }

  for (s16 r = 0; r < horizon; r++)
  {
    A_qp.block(13 * r, 0, 13, 13) = powerMats[r + 1]; // Adt.pow(r+1);

    for (s16 c = 0; c < horizon; c++)
    {
      if (r >= c)
      {
        s16 a_num = r - c;
        B_qp.block(13 * r, 12 * c, 13, 12) = powerMats[a_num] /*Adt.pow(a_num)*/ * Bdt;
        Q_qp.block(13*r, 0, 13, 6) += powerMats[a_num] * Qdt; // NEW заполняем матрицу Q_qp (Adt.pow(a_num)*Qdt)
      }
    }
  }



#ifdef K_PRINT_EVERYTHING
  cout << "AQP:\n" << A_qp << "\nBQP:\n" << B_qp << "\nQdt:\n" << Qdt << endl;
#endif
}
// ------------------------ИЗМЕНЕНО------------------------------------

void resize_qp_mats(s16 horizon)
{
  int mcount = 0;
  int h2 = horizon * horizon;

  A_qp.resize(13 * horizon, Eigen::NoChange);
  mcount += 13 * horizon * 1;

  B_qp.resize(13 * horizon, 12 * horizon);
  mcount += 13 * h2 * 12;

  S.resize(13 * horizon, 13 * horizon);
  mcount += 13 * 13 * h2;

  X_d.resize(13 * horizon, Eigen::NoChange);
  mcount += 13 * horizon;

  U_b.resize(20 * horizon, Eigen::NoChange);
  mcount += 20 * horizon;

  fmat.resize(20 * horizon, 12 * horizon);
  mcount += 20 * 12 * h2;

  qH.resize(12 * horizon, 12 * horizon);
  mcount += 12 * 12 * h2;

  qg.resize(12 * horizon, Eigen::NoChange);
  mcount += 12 * horizon;

  eye_12h.resize(12 * horizon, 12 * horizon);
  mcount += 12 * 12 * horizon;

// ------------------------ИЗМЕНЕНО------------------------------------
  Q_qp.resize(13*horizon, 6); // NEW: Q_qp  (13*horizon x 6)
  mcount += 13 * 6 * horizon;

  // printf("realloc'd %d floating point numbers.\n",mcount);
  mcount = 0;

  A_qp.setZero();
  B_qp.setZero();
  S.setZero();
  X_d.setZero();
  U_b.setZero();
  fmat.setZero();
  qH.setZero();
  eye_12h.setIdentity();
  Q_qp.setZero();   // NEW
// ------------------------ИЗМЕНЕНО------------------------------------

  // TODO: use realloc instead of free/malloc on size changes

  if (real_allocated)
  {

    free(H_qpoases);
    free(g_qpoases);
    free(A_qpoases);
    free(lb_qpoases);
    free(ub_qpoases);
    free(q_soln);
    free(H_red);
    free(g_red);
    free(A_red);
    free(lb_red);
    free(ub_red);
    free(q_red);
  }

  H_qpoases = (qpOASES::real_t*)malloc(12 * 12 * horizon * horizon * sizeof(qpOASES::real_t));
  mcount += 12 * 12 * h2;
  g_qpoases = (qpOASES::real_t*)malloc(12 * 1 * horizon * sizeof(qpOASES::real_t));
  mcount += 12 * horizon;
  A_qpoases = (qpOASES::real_t*)malloc(12 * 20 * horizon * horizon * sizeof(qpOASES::real_t));
  mcount += 12 * 20 * h2;
  lb_qpoases = (qpOASES::real_t*)malloc(20 * 1 * horizon * sizeof(qpOASES::real_t));
  mcount += 20 * horizon;
  ub_qpoases = (qpOASES::real_t*)malloc(20 * 1 * horizon * sizeof(qpOASES::real_t));
  mcount += 20 * horizon;
  q_soln = (qpOASES::real_t*)malloc(12 * horizon * sizeof(qpOASES::real_t));
  mcount += 12 * horizon;

  H_red = (qpOASES::real_t*)malloc(12 * 12 * horizon * horizon * sizeof(qpOASES::real_t));
  mcount += 12 * 12 * h2;
  g_red = (qpOASES::real_t*)malloc(12 * 1 * horizon * sizeof(qpOASES::real_t));
  mcount += 12 * horizon;
  A_red = (qpOASES::real_t*)malloc(12 * 20 * horizon * horizon * sizeof(qpOASES::real_t));
  mcount += 12 * 20 * h2;
  lb_red = (qpOASES::real_t*)malloc(20 * 1 * horizon * sizeof(qpOASES::real_t));
  mcount += 20 * horizon;
  ub_red = (qpOASES::real_t*)malloc(20 * 1 * horizon * sizeof(qpOASES::real_t));
  mcount += 20 * horizon;
  q_red = (qpOASES::real_t*)malloc(12 * horizon * sizeof(qpOASES::real_t));
  mcount += 12 * horizon;
  real_allocated = 1;

  // printf("malloc'd %d floating point numbers.\n",mcount);

#ifdef K_DEBUG
  printf("RESIZED MATRICES FOR HORIZON: %d\n", horizon);
#endif
}

inline Matrix<fpt, 3, 3> cross_mat(Matrix<fpt, 3, 3> I_inv, Matrix<fpt, 3, 1> r)
{
  Matrix<fpt, 3, 3> cm;
  cm << 0.f, -r(2), r(1), r(2), 0.f, -r(0), -r(1), r(0), 0.f;
  return I_inv * cm;
}

// continuous time state space matrices.
void ct_ss_mats(Matrix<fpt, 3, 3> I_world, fpt m, Matrix<fpt, 3, 4> r_feet, Matrix<fpt, 3, 3> R_yaw, Matrix<fpt, 13, 13>& A, Matrix<fpt, 13, 12>& B, float x_drag)
{
  A.setZero();
  A(3, 9) = 1.f;
  A(11, 9) = x_drag;
  A(4, 10) = 1.f;
  A(5, 11) = 1.f;

  A(11, 12) = 1.f;
  A.block(0, 6, 3, 3) = R_yaw.transpose();


  B.setZero();
  Matrix<fpt, 3, 3> I_inv = I_world.inverse();

  for (s16 b = 0; b < 4; b++)
  {
    B.block(6, b * 3, 3, 3) = cross_mat(I_inv, r_feet.col(b));
    B.block(9, b * 3, 3, 3) = Matrix<fpt, 3, 3>::Identity() / m;
  }

  //std::cout << "A_real:\n" << A << '\n' << std::endl;
  // std::cout << "B_real:\n" << B << '\n' << std::endl;

  //std::cout << "r_feet_real:\n" << r_feet << '\n' << std::endl;
  //std::cout << "x_drag:\n" << x_drag << "\n";

  // Eigen::MatrixXd tempSkewMatrix3;
  // Eigen::MatrixXd A_LQR;
  // Eigen::MatrixXd B_LQR;

  // Eigen::VectorXd x_COM_world;
  // Eigen::VectorXd xdot_COM_world;
  // Eigen::VectorXd omega_b_world;
  // Eigen::VectorXd omega_b_body; // new
  // Eigen::VectorXd quat_b_world;
  // Eigen::MatrixXd R_b_world;
  // Eigen::MatrixXd p_feet;

  // /* Desired Kinematics */
  // Eigen::VectorXd x_COM_world_desired;
  // Eigen::VectorXd xdot_COM_world_desired;
  // Eigen::VectorXd xddot_COM_world_desired;
  // Eigen::VectorXd omega_b_world_desired;
  // Eigen::VectorXd omega_b_body_desired; // new
  // Eigen::VectorXd omegadot_b_world_desired;
  // Eigen::MatrixXd R_b_world_desired;
  // Eigen::MatrixXd p_feet_desired; // new

  // Eigen::VectorXd tempVector3;

  // // Temporary variables for block assignment
  // Eigen::MatrixXd tempBlock;
  // tempBlock.setZero(3, 3);
  // Eigen::VectorXd rd;
  // rd.resize(3, 1);
  // Eigen::MatrixXd rd_hat;
  // rd_hat.resize(3, 3);

  // // Update the A matrix in sdot = A*s+B*df
  // tempSkewMatrix3.setIdentity();
  // A_LQR.block<3, 3>(0, 3) << tempSkewMatrix3;
  // A_LQR.block<3, 3>(6, 9) << tempSkewMatrix3;
  // crossMatrix(tempSkewMatrix3, -omega_b_body_desired);
  // A_LQR.block<3, 3>(6, 6) << tempSkewMatrix3;

  // uint8_t num_contact_points = 4;

  // omega_b_body = R_b_world.transpose() * omega_b_world;
  // omega_b_body_desired = R_b_world_desired.transpose() * omega_b_world_desired;

  // for (int i = 0; i < num_contact_points; i++)
  // {
  //   tempVector3 << f_ref_world(3 * i), f_ref_world(3 * i + 1), f_ref_world(3 * i + 2);
  //   crossMatrix(tempSkewMatrix3, tempVector3);
  //   tempBlock << tempBlock + Ig.inverse() * R_b_world_desired.transpose() * tempSkewMatrix3;
  // }
  // A_LQR.block<3, 3>(9, 0) << tempBlock;

  // tempBlock.setZero();
  // for (int i = 0; i < num_contact_points; i++)
  // {
  //   tempVector3 << f_ref_world(3 * i), f_ref_world(3 * i + 1), f_ref_world(3 * i + 2);
  //   rd << p_feet_desired.col(i);
  //   crossMatrix(rd_hat, rd);
  //   crossMatrix(tempSkewMatrix3, rd_hat * tempVector3);
  //   tempBlock << tempBlock + Ig.inverse() * R_b_world_desired.transpose() * tempSkewMatrix3;
  // }

  // A_LQR.block<3, 3>(9, 6) << tempBlock;
}

void quat_to_rpy(Quaternionf q, Matrix<fpt, 3, 1>& rpy)
{
  // from my MATLAB implementation

  // edge case!
  fpt as = t_min(-2. * (q.x() * q.z() - q.w() * q.y()), .99999);
  rpy(0) = atan2(2.f * (q.x() * q.y() + q.w() * q.z()), sq(q.w()) + sq(q.x()) - sq(q.y()) - sq(q.z()));
  rpy(1) = asin(as);
  rpy(2) = atan2(2.f * (q.y() * q.z() + q.w() * q.x()), sq(q.w()) - sq(q.x()) - sq(q.y()) + sq(q.z()));
}

void print_problem_setup(problem_setup* setup)
{
  printf("DT: %.3f\n", setup->dt);
  printf("Mu: %.3f\n", setup->mu);
  printf("F_Max: %.3f\n", setup->f_max);
  printf("Horizon: %d\n", setup->horizon);
}

void print_update_data(update_data_t* update, s16 horizon)
{
  print_named_array("p", update->p, 1, 3);
  print_named_array("v", update->v, 1, 3);
  print_named_array("q", update->q, 1, 4);
  print_named_array("w", update->r, 3, 4);
  pnv("Yaw", update->yaw);
  print_named_array("weights", update->weights, 1, 12);
  print_named_array("trajectory", update->traj, horizon, 12);
  pnv("Alpha", update->alpha);
  print_named_array("gait", update->gait, horizon, 4);
}

Matrix<fpt, 13, 1> x_0;
Matrix<fpt, 3, 3> I_world;
Matrix<fpt, 13, 13> A_ct;
Matrix<fpt, 13, 12> B_ct_r;


Eigen::Matrix<float, 6, 1> f_est = Eigen::Matrix<float, 6, 1>::Zero();
Eigen::Matrix<float, 6, 1> f_est_smoothed = Eigen::Matrix<float, 6, 1>::Zero();
Eigen::Matrix<float, 6, 1> f_est_static = Eigen::Matrix<float, 6, 1>::Zero();
float f_static_3 = 0.f;

Eigen::Matrix<float, 6, 1> alpha = (Eigen::Matrix<float, 6, 1>() << 0.99f, 0.99f, 0.99f, 0.9f, 0.99f, 0.99f).finished();
Eigen::Matrix<float, 6, 1> beta  = (Eigen::Matrix<float, 6, 1>() << 0.01f, 0.01f, 0.01f, 0.1f, 0.01f, 0.01f).finished();

std::vector<float> time_history;
std::vector<float> diff_history;

//std::vector<Eigen::Matrix<float, 6, 1>> f_ext_history;

// Функция для применения 1D-гaуссова фильтра к данным
std::vector<double> gaussian_filter(const std::vector<double>& data, float sigma) {
    int radius = std::ceil(3 * sigma); // радиус свёртки (обычно 3*sigma достаточно)
    std::vector<float> kernel(2 * radius + 1);
    float sum = 0.0;
    
    // Вычисляем ядро Гаусса
    for (int i = -radius; i <= radius; i++) {
        float value = std::exp(-0.5 * (i * i) / (sigma * sigma));
        kernel[i + radius] = value;
        sum += value;
    }
    // Нормализация ядра
    for (auto& k : kernel) {
        k /= sum;
    }
    
    // Применяем свёртку
    std::vector<double> filtered(data.size(), 0.0);
    int n = data.size();
    for (int i = 0; i < n; i++) {
        double accum = 0.0;
        for (int j = -radius; j <= radius; j++) {
            int index = i + j;
            // Обработка граничных условий: повтор крайних значений
            if (index < 0)
                index = 0;
            else if (index >= n)
                index = n - 1;
            accum += data[index] * kernel[j + radius];
        }
        filtered[i] = accum;
    }
    return filtered;
}


struct SinFitResult {
    double amp;    // Амплитуда
    double omega;  // Угловая частота
    double phase;  // Фаза
    double offset; // Смещение (статическое распределение)
    double freq;   // Частота
    double period; // Период
};

// Вычисление среднего значения вектора
double mean(const std::vector<double>& v) {
    double sum = std::accumulate(v.begin(), v.end(), 0.0);
    return sum / v.size();
}

// Вычисление стандартного отклонения вектора
double stddev(const std::vector<double>& v, double m) {
    double accum = 0.0;
    for (double x : v)
        accum += (x - m) * (x - m);
    return std::sqrt(accum / v.size());
}

// Функция для вычисления массива частот (аналог np.fft.fftfreq)
std::vector<double> fftfreq(int n, double dt) {
    std::vector<double> freqs(n);
    for (int k = 0; k < n; k++) {
        if (k <= n / 2)
            freqs[k] = k / (n * dt);
        else
            freqs[k] = (k - n) / (n * dt);
    }
    return freqs;
}

// Функция для подгона синусоиды
// y(t) = A * sin(omega * t + phase) + offset
// Реализована логика вычисления начальных приближений через FFT
SinFitResult fit_sin(const std::vector<double>& tt, const std::vector<double>& yy) {
    int n = tt.size();
    double dt = tt[1] - tt[0];  // предполагается равномерный шаг

    // Вычисление массива частот
    std::vector<double> freqs = fftfreq(n, dt);

    // Вычисление FFT сигнала yy
    // План для реального преобразования (выходной массив имеет размер n/2+1, но для простоты выделяем массив размера n)
    std::vector<std::complex<double>> fft_result(n);
    {
        fftw_plan plan = fftw_plan_dft_r2c_1d(n, const_cast<double*>(yy.data()),
                                              reinterpret_cast<fftw_complex*>(fft_result.data()),
                                              FFTW_ESTIMATE);
        fftw_execute(plan);
        fftw_destroy_plan(plan);
    }
    
    // Вычисление модуля БПФ (рассматриваем только первые n/2+1 точек)
    std::vector<double> Fyy(n / 2 + 1);
    for (int i = 0; i < n / 2 + 1; i++) {
        Fyy[i] = std::abs(fft_result[i]);
    }
    
    // Нахождение индекса максимума модуля БПФ, исключая нулевую частоту (смещение)
    int max_index = 1;
    double max_val = Fyy[1];
    for (int i = 2; i < Fyy.size(); i++) {
        if (Fyy[i] > max_val) {
            max_val = Fyy[i];
            max_index = i;
        }
    }
    double guess_freq = std::abs(freqs[max_index]);

    // Оценка амплитуды: sqrt(2) * стандартное отклонение
    double m = mean(yy);
    double s = stddev(yy, m);
    double guess_amp = s * std::sqrt(2.0);
    double guess_offset = m;
    double guess_omega = 2 * M_PI * guess_freq;
    double guess_phase = 0.0;

    // Здесь выполняется нелинейная оптимизация для подгонки параметров
    // y = A*sin(omega*t + phase) + offset
    // В данном примере вместо оптимизации возвращаются начальные приближения.
    // Для реального применения необходимо заменить следующий блок на вызов алгоритма оптимизации.
    double A = guess_amp;
    double w = guess_omega;
    double phase = guess_phase;
    double offset = guess_offset;

    double freq = w / (2 * M_PI);
    double period = (freq != 0) ? (1.0 / freq) : 0.0;

    SinFitResult result;
    result.amp = A;
    result.omega = w;
    result.phase = phase;
    result.offset = offset;
    result.freq = freq;
    result.period = period;
    return result;
}

// Функция estimate_disturbance, аналогичная Python-версии,
// возвращает статистическое распределение (offset), оценённую амплитуду,
// частоту и фазу.
std::tuple<double, double, double, double> estimate_disturbance(const std::vector<double>& tt, const std::vector<double>& diff) {
    SinFitResult res = fit_sin(tt, diff);
    double stat_dis = res.offset;
    double amp_est = res.amp;
    double freq_est = res.freq;
    double phase_est = res.phase;
    return std::make_tuple(stat_dis, amp_est, freq_est, phase_est);
}

float compensatory_force = 0.f;

double est_stat = 0.;
double est_amp = 0.;
double est_freq = 0.;
double est_phase = 0.;


extern float simulation_time;


void solve_mpc(update_data_t* update, problem_setup* setup)
{
  //static auto last_call_time = std::chrono::high_resolution_clock::now();
  static auto start_time = std::chrono::high_resolution_clock::now();

  rs.set(update->p, update->v, update->q, update->w, update->r, update->roll, update->pitch, update->yaw);

#ifdef K_PRINT_EVERYTHING

  printf("-----------------\n");
  printf("   PROBLEM DATA  \n");
  printf("-----------------\n");
  print_problem_setup(setup);

  printf("-----------------\n");
  printf("    ROBOT DATA   \n");
  printf("-----------------\n");
  rs.print();
  print_update_data(update, setup->horizon);
#endif

  // roll pitch yaw
  Matrix<fpt, 3, 1> rpy;
  quat_to_rpy(rs.q, rpy);

  // initial state (13 state representation)
  x_0 << rpy(2), rpy(1), rpy(0), rs.p, rs.w, rs.v, -9.8f;
  I_world = rs.R_yaw * rs.I_body * rs.R_yaw.transpose(); // original
  // I_world = rs.R_yaw.transpose() * rs.I_body * rs.R_yaw;
  // cout<<rs.R_yaw<<endl;
  ct_ss_mats(I_world, rs.m, rs.r_feet, rs.R_yaw, A_ct, B_ct_r, update->x_drag);
  //std::cout << "\nr_feet:\n" << rs.r_feet << endl;

#ifdef K_PRINT_EVERYTHING
  cout << "Initial state: \n" << x_0 << endl;
  cout << "World Inertia: \n" << I_world << endl;
  cout << "A CT: \n" << A_ct << endl;
  cout << "B CT (simplified): \n" << B_ct_r << endl;
#endif

// ------------------------ИЗМЕНЕНО------------------------------------
  Matrix<fpt, 13, 6> Q_ct;
  // Из уравнения динамики формируем матрицу Q_ct
  Q_ct.setZero();
  // первые 6 строк нули:
  //   (по умолчанию setZero)
  // следующие 6 строк = единичная 6x6:
  for(int iRow=6; iRow<12; iRow++)
    Q_ct(iRow, iRow-6) = 1.f;
  // 13-я строка = 0
// ------------------------ИЗМЕНЕНО------------------------------------


  // QP matrices
  c2qp(A_ct, B_ct_r, Q_ct, setup->dt, setup->horizon);


  // weights
  Matrix<fpt, 13, 1> full_weight;
  for (u8 i = 0; i < 12; i++)
  {
    full_weight(i) = update->weights[i];
  }
  full_weight(12) = 0.f;
  S.diagonal() = full_weight.replicate(setup->horizon, 1);

  // trajectory
  for (s16 i = 0; i < setup->horizon; i++)
  {
    for (s16 j = 0; j < 12; j++)
    {
      X_d(13 * i + j, 0) = update->traj[12 * i + j];
    }
  }
  // cout<<"XD:\n"<<X_d<<endl;

  // note - I'm not doing the shifting here.
  s16 k = 0;
  for (s16 i = 0; i < setup->horizon; i++)
  {
    for (s16 j = 0; j < 4; j++)
    {
      U_b(5 * k + 0) = BIG_NUMBER;
      U_b(5 * k + 1) = BIG_NUMBER;
      U_b(5 * k + 2) = BIG_NUMBER;
      U_b(5 * k + 3) = BIG_NUMBER;
      U_b(5 * k + 4) = update->gait[i * 4 + j] * setup->f_max;
      k++;
    }
  }

  fpt mu = 1.f / setup->mu;
  Matrix<fpt, 5, 3> f_block;

  f_block << mu, 0, 1.f, -mu, 0, 1.f, 0, mu, 1.f, 0, -mu, 1.f, 0, 0, 1.f;

  for (s16 i = 0; i < setup->horizon * 4; i++)
  {
    fmat.block(i * 5, i * 3, 5, 3) = f_block;
  }

  // ------------------------ИЗМЕНЕНО------------------------------------
  Eigen::VectorXf vec_X(13); // 3 + 3 + 3 + 3 + 1 = 13 элементов
  vec_X << rpy, rs.p, rs.w, rs.v, -9.8f;

  //std::cout << "rs.R_test :\n" << rs.R_yaw.transpose() << "\n";

  //f_ext_history.push_back(f_ext_test);
  //std::cout << "Размер f_ext_history: " << f_ext_history.size() << std::endl;


  // Скользящая средняя
  //static Eigen::Matrix<float, 6, 1> f_ext_prev = f_ext; // инициализируем при первом вызове

  //std::cout << "f_est_befor\n: " << f_est << std::endl;

  // АДАПТАЦИЯ
  // Коэффициенты сглаживания для адаптации
  //f_est = alpha * f_est + beta * f_ext;

  // --- АДАПТАЦИЯ: обновление оценки возмущения для компоненты f_ext[4] ---
  // Статическая переменная для отслеживания времени (накапливается с каждым вызовом)
  static float current_time = 0.f;
  current_time += setup->dt;

  // В качестве diff используем компоненту f_ext_test(4)
  diff_history.push_back(f_ext(3));


  //std::cout << "Test:\n" << simulation_time << "\n";

  //time_history.push_back(current_time);
  time_history.push_back(simulation_time);
  // std::cout << "simTime\n: " << simTime << std::endl;



  // Если накопилось достаточно точек (например, 300), выполняем оценку синуса
  const int window_size = 400;
  if (time_history.size() >= window_size)
  {
      if (time_history.size() <= 500){


        std::vector<double> t_window(time_history.end() - window_size, time_history.end());
        std::vector<double> diff_window(diff_history.end() - window_size, diff_history.end());
        

        std::vector<double> blurred = gaussian_filter(diff_window, 7.0);
        std::vector<double> very_blurred = gaussian_filter(diff_window, 27.0); // КОЭФФИЦИЕНТ СГЛАЖИВАНИЯ НЕОБХОДИМО НАСТРАИВАТЬ

        // Вычисляем разницу: blurred - very_blurred
        std::vector<double> diff_history_smoothed(diff_window.size());
        for (size_t i = 0; i < diff_window.size(); i++) {
            diff_history_smoothed[i] = blurred[i] - very_blurred[i];
        }

        // std::cout << "time_history:\n"<< std::endl;
        // for (const auto& value : t_window) {
        //     std::cout << value << " ";
        // }
        // std::cout << "\n"<< std::endl;

        
        // std::cout << "diff_history:\n"<< std::endl;

        // for (const auto& value : diff_window) {
        //     std::cout << value << " ";
        // }
        // std::cout << std::endl;
        // std::cout << "\n"<< std::endl;

        // // Выводим результат
        // std::cout << "diff_history_smoothed:" << std::endl;
        // for (double val : diff_history_smoothed) {
        //     std::cout << val << " ";
        // }
        // std::cout << std::endl;



        std::tuple<double, double, double, double> estimate = estimate_disturbance(t_window, diff_history_smoothed);

        
        est_stat = std::get<0>(estimate);
        est_amp = std::get<1>(estimate);
        est_freq = std::get<2>(estimate);
        est_phase = std::get<3>(estimate);


        // Аналогично Python-коду:
        // est_stat /= setup->dt;
        // est_amp  /= setup->dt;
        //std::cout << "est_freq\n: " << est_freq << std::endl;
        //est_freq *= 2 * M_PI;
        // Вычисляем компенсаторное усилие для f_ext[4]
        //compensatory_force = est_stat + est_amp * std::sin(current_time * est_freq + est_phase);

      }

      compensatory_force = est_amp + std::sin(2* M_PI * simulation_time * est_freq + est_phase);

      //compensatory_force = -0.78 + std::sin(2* M_PI * simulation_time * 0.3296 + est_phase);


      // Обновляем f_est для компоненты 4 с использованием сглаживающих коэффициентов
      f_est(3) = compensatory_force;

      //Выводим отладочную информацию
      std::cout << "Адаптация для компоненты f_ext[3]: stat=" << est_stat 
                << ", amp=" << est_amp 
                << ", freq=" << est_freq 
                << ", phase=" << est_phase 
                << ", compens_force=" << compensatory_force << std::endl;

  }

  f_est_smoothed = 0.95f* f_est_smoothed + 0.05f * f_est;


  //f_est = alpha.cwiseProduct(f_est) + beta.cwiseProduct(f_ext);

  //std::cout << f_est[3] << std::endl;
  //std::cout << f_ext[3] << std::endl;
  //std::cout << "f_ext\n: " << f_ext << std::endl;
  //std::cout << "f_est:\n" << f_est[3] << std::endl;
  //std::cout << "f_ext\n: " << f_ext[3] << std::endl;

  Matrix<fpt, 6, 1> f_ext_test;
  f_ext_test << 0.f, 0.f, 0.f, 0.f, 0.f, 0.f; //f = [tx ty tz fx fy fz]^T

    //  СТАЦИОНАРНАЯ АДАПТАЦИЯ
  f_est_static[3] = 0.97f * f_est_static[3] + 0.03f * f_ext[3];

  //f_ext_test << 0.f, 0.f, 0.f, 1.5 * std::sin(0.25 * simTime), 0.f, 0.f;

  //std::cout << "f_est\n: " << f_est << std::endl;

  // ------------------------ИЗМЕНЕНО------------------------------------

  qH = 2 * (B_qp.transpose() * S * B_qp + update->alpha * eye_12h);
// ------------------------ИЗМЕНЕНО------------------------------------
  if (time_history.size() > 500)
  {
    qg = 2 * B_qp.transpose() * S * (A_qp * x_0 + Q_qp * f_est - X_d); // F_est added
  }
  else{
    qg = 2 * B_qp.transpose() * S * (A_qp * x_0 + Q_qp * f_ext_test - X_d);
  }

// ------------------------ИЗМЕНЕНО------------------------------------

  QpProblem<double> jcqp(setup->horizon * 12, setup->horizon * 20);
  // use jcqp = 0
  if (update->use_jcqp == 1)
  {
    jcqp.A = fmat.cast<double>();
    jcqp.P = qH.cast<double>();
    jcqp.q = qg.cast<double>();
    jcqp.u = U_b.cast<double>();

    for (s16 i = 0; i < 20 * setup->horizon; i++)
    {
      jcqp.l[i] = 0.;
    }

    jcqp.settings.sigma = update->sigma;
    jcqp.settings.alpha = update->solver_alpha;
    jcqp.settings.terminate = update->terminate;
    jcqp.settings.rho = update->rho;
    jcqp.settings.maxIterations = update->max_iterations;
    jcqp.runFromDense(update->max_iterations, true, false);
  }
  else
  {
    matrix_to_real(H_qpoases, qH, setup->horizon * 12, setup->horizon * 12);
    matrix_to_real(g_qpoases, qg, setup->horizon * 12, 1);
    matrix_to_real(A_qpoases, fmat, setup->horizon * 20, setup->horizon * 12);
    matrix_to_real(ub_qpoases, U_b, setup->horizon * 20, 1);

    for (s16 i = 0; i < 20 * setup->horizon; i++)
    {
      lb_qpoases[i] = 0.0f;
    }

    s16 num_constraints = 20 * setup->horizon;
    s16 num_variables = 12 * setup->horizon;

    qpOASES::int_t nWSR = 100;

    int new_vars = num_variables;
    int new_cons = num_constraints;

    for (int i = 0; i < num_constraints; i++)
    {
      con_elim[i] = 0;
    }

    for (int i = 0; i < num_variables; i++)
    {
      var_elim[i] = 0;
    }

    for (int i = 0; i < num_constraints; i++)
    {
      if (!(near_zero(lb_qpoases[i]) && near_zero(ub_qpoases[i])))
      {
        continue;
      }

      double* c_row = &A_qpoases[i * num_variables];
      for (int j = 0; j < num_variables; j++)
      {
        if (near_one(c_row[j]))
        {
          new_vars -= 3;
          new_cons -= 5;
          int cs = (j * 5) / 3 - 3;
          var_elim[j - 2] = 1;
          var_elim[j - 1] = 1;
          var_elim[j] = 1;
          con_elim[cs] = 1;
          con_elim[cs + 1] = 1;
          con_elim[cs + 2] = 1;
          con_elim[cs + 3] = 1;
          con_elim[cs + 4] = 1;
        }
      }
    }
    // if(new_vars != num_variables)
    if (1 == 1)
    {
      int var_ind[new_vars];
      int con_ind[new_cons];
      int vc = 0;
      for (int i = 0; i < num_variables; i++)
      {
        if (!var_elim[i])
        {
          if (!(vc < new_vars))
          {
            printf("BAD ERROR 1\n");
          }
          var_ind[vc] = i;
          vc++;
        }
      }
      vc = 0;
      for (int i = 0; i < num_constraints; i++)
      {
        if (!con_elim[i])
        {
          if (!(vc < new_cons))
          {
            printf("BAD ERROR 1\n");
          }
          con_ind[vc] = i;
          vc++;
        }
      }
      for (int i = 0; i < new_vars; i++)
      {
        int olda = var_ind[i];
        g_red[i] = g_qpoases[olda];
        for (int j = 0; j < new_vars; j++)
        {
          int oldb = var_ind[j];
          H_red[i * new_vars + j] = H_qpoases[olda * num_variables + oldb];
        }
      }

      for (int con = 0; con < new_cons; con++)
      {
        for (int st = 0; st < new_vars; st++)
        {
          float cval = A_qpoases[(num_variables * con_ind[con]) + var_ind[st]];
          A_red[con * new_vars + st] = cval;
        }
      }
      for (int i = 0; i < new_cons; i++)
      {
        int old = con_ind[i];
        ub_red[i] = ub_qpoases[old];
        lb_red[i] = lb_qpoases[old];
      }

      if (update->use_jcqp == 0)
      {
        Timer solve_timer;
        qpOASES::QProblem problem_red(new_vars, new_cons);
        qpOASES::Options op;
        op.setToMPC();
        op.printLevel = qpOASES::PL_NONE;
        problem_red.setOptions(op);
        // int_t nWSR = 50000;

        int rval = problem_red.init(H_red, g_red, A_red, NULL, NULL, lb_red, ub_red, nWSR);
        (void)rval;
        int rval2 = problem_red.getPrimalSolution(q_red);
        if (rval2 != qpOASES::SUCCESSFUL_RETURN)
        {
          printf("failed to solve!\n");
        }

        vc = 0;
        for (int i = 0; i < num_variables; i++)
        {
          if (var_elim[i])
          {
            q_soln[i] = 0.0f;
          }
          else
          {
            q_soln[i] = q_red[vc];
            vc++;
          }
        }
      }
      else
      { // use jcqp == 2
        QpProblem<double> reducedProblem(new_vars, new_cons);

        reducedProblem.A = DenseMatrix<double>(new_cons, new_vars);
        int i = 0;
        for (int r = 0; r < new_cons; r++)
        {
          for (int c = 0; c < new_vars; c++)
          {
            reducedProblem.A(r, c) = A_red[i++];
          }
        }

        reducedProblem.P = DenseMatrix<double>(new_vars, new_vars);
        i = 0;
        for (int r = 0; r < new_vars; r++)
        {
          for (int c = 0; c < new_vars; c++)
          {
            reducedProblem.P(r, c) = H_red[i++];
          }
        }

        reducedProblem.q = Vector<double>(new_vars);
        for (int r = 0; r < new_vars; r++)
        {
          reducedProblem.q[r] = g_red[r];
        }

        reducedProblem.u = Vector<double>(new_cons);
        for (int r = 0; r < new_cons; r++)
        {
          reducedProblem.u[r] = ub_red[r];
        }

        reducedProblem.l = Vector<double>(new_cons);
        for (int r = 0; r < new_cons; r++)
        {
          reducedProblem.l[r] = lb_red[r];
        }

        //        jcqp.A = fmat.cast<double>();
        //        jcqp.P = qH.cast<double>();
        //        jcqp.q = qg.cast<double>();
        //        jcqp.u = U_b.cast<double>();
        //        for(s16 i = 0; i < 20*setup->horizon; i++)
        //          jcqp.l[i] = 0.;

        reducedProblem.settings.sigma = update->sigma;
        reducedProblem.settings.alpha = update->solver_alpha;
        reducedProblem.settings.terminate = update->terminate;
        reducedProblem.settings.rho = update->rho;
        reducedProblem.settings.maxIterations = update->max_iterations;
        reducedProblem.runFromDense(update->max_iterations, true, false);

        vc = 0;
        for (int kk = 0; kk < num_variables; kk++)
        {
          if (var_elim[kk])
          {
            q_soln[kk] = 0.0f;
          }
          else
          {
            q_soln[kk] = reducedProblem.getSolution()[vc];
            vc++;
          }
        }
      }
    }
  }

  if (update->use_jcqp == 1)
  {
    for (int i = 0; i < 12 * setup->horizon; i++)
    {
      q_soln[i] = jcqp.getSolution()[i];
    }
  }

  // for (int i = 0; i < 12; i++){
  //   std::cout << "q_soln_" << i << ':' << q_soln[i] << "\n";
  // }
  // Eigen::Vector12f f_sol;
  // for (int leg = 0; leg < 4; leg++)
  // {
  //   for (int axis = 0; axis < 3; axis++){
  //     f_sol[axis] = q_soln[leg * 3 + axis];
  //   }
  // }
  // std::cout << "f_sol" << f_sol << "\n";



  //std::cout << "q_soln_" << -rs.R_yaw.transpose() * q_soln << "\n";

  // auto end_time = std::chrono::high_resolution_clock::now();
  // std::chrono::duration<double> elapsed = end_time - start_time;
  // std::chrono::duration<double> time_since_last = start_time - last_call_time;
  // last_call_time = start_time;

  // std::cout << "solve_mpc execution time: " << elapsed.count() << " s\n";
  // std::cout << "Time since last call: " << time_since_last.count() << " s\n";
  // std::cout << "Approximate frequency: " << (1.0 / time_since_last.count()) << " Hz\n";
}

void crossMatrix(Eigen::MatrixXd& R, const Eigen::VectorXd& omega)
{
  R.setZero();
  R(0, 1) = -omega(2);
  R(0, 2) = omega(1);
  R(1, 0) = omega(2);
  R(1, 2) = -omega(0);
  R(2, 0) = -omega(1);
  R(2, 1) = omega(0);
}


// Функция оценки синусоидальной модели по последнему окну данных.
// tt и diff – векторы времени и соответствующих значений (для выбранной компоненты, например f_ext[4]).
// dt – шаг дискретизации.
// bool estimate_disturbance(const std::vector<float>& tt, 
//                           const std::vector<float>& diff, 
//                           float dt,
//                           float &stat_est, float &amp_est, 
//                           float &freq_est, float &phase_est)
// {
//     int N = tt.size();
//     if (N < 3) return false; // недостаточно данных

//     // Вычисляем среднее и стандартное отклонение по diff
//     float sum = 0.0f;
//     for (int i = 0; i < N; i++) {
//         sum += diff[i];
//     }
//     float mean_val = sum / N;

//     float sq_sum = 0.0f;
//     for (int i = 0; i < N; i++) {
//         float d = diff[i] - mean_val;
//         sq_sum += d * d;
//     }
//     float std_val = std::sqrt(sq_sum / N);

//     // --- Определение начального предположения для частоты через ДПФ (наивно) ---
//     // Рассмотрим только k=1,...,N/2 для поиска пика
//     float best_mag = 0.0f;
//     int best_k = 1;
//     for (int k = 1; k <= N/2; k++) {
//         float real = 0.0f, imag = 0.0f;
//         for (int n = 0; n < N; n++) {
//             float angle = 2 * M_PI * k * n / N;
//             real += diff[n] * std::cos(angle);
//             imag += diff[n] * std::sin(angle);
//         }
//         float mag = std::sqrt(real*real + imag*imag);
//         if (mag > best_mag) {
//             best_mag = mag;
//             best_k = k;
//         }
//     }
//     // Определяем частоту в Гц: общее время T = N*dt, разрешающая способность = 1/(N*dt)
//     float guess_freq = best_k / (N * dt);
//     float w_guess = 2 * M_PI * guess_freq; // угловая частота

//     // --- Линейная подгонка модели: y = B*sin(w*t) + D*cos(w*t) + c ---
//     // Построим матрицу проектирования A размером (N x 3) с колонками: sin(w*t), cos(w*t), 1
//     float Sxx[3][3] = { {0,0,0}, {0,0,0}, {0,0,0} };
//     float Sxy[3] = {0, 0, 0};

//     for (int i = 0; i < N; i++) {
//         float t_val = tt[i];
//         float s_val = std::sin(w_guess * t_val);
//         float c_val = std::cos(w_guess * t_val);
//         float y_val = diff[i];

//         Sxx[0][0] += s_val * s_val;
//         Sxx[0][1] += s_val * c_val;
//         Sxx[0][2] += s_val;
//         Sxx[1][0] += c_val * s_val;
//         Sxx[1][1] += c_val * c_val;
//         Sxx[1][2] += c_val;
//         Sxx[2][0] += s_val;
//         Sxx[2][1] += c_val;
//         Sxx[2][2] += 1.0f;

//         Sxy[0] += s_val * y_val;
//         Sxy[1] += c_val * y_val;
//         Sxy[2] += y_val;
//     }
    
//     // Решаем систему 3x3: (A^T A) * X = A^T y, где X = [B, D, c]^T
//     float X[3] = {0,0,0};
//     {
//         float A_mat[3][4];
//         for (int i = 0; i < 3; i++) {
//             A_mat[i][0] = Sxx[i][0];
//             A_mat[i][1] = Sxx[i][1];
//             A_mat[i][2] = Sxx[i][2];
//             A_mat[i][3] = Sxy[i];
//         }
//         // Прямой ход метода Гаусса
//         for (int i = 0; i < 3; i++) {
//             // Поиск опорного элемента
//             int pivot = i;
//             for (int j = i+1; j < 3; j++) {
//                 if (std::fabs(A_mat[j][i]) > std::fabs(A_mat[pivot][i]))
//                     pivot = j;
//             }
//             // Обмен строками
//             if (pivot != i) {
//                 for (int k = 0; k < 4; k++) {
//                     float tmp = A_mat[i][k];
//                     A_mat[i][k] = A_mat[pivot][k];
//                     A_mat[pivot][k] = tmp;
//                 }
//             }
//             if (std::fabs(A_mat[i][i]) < 1e-6f)
//                 return false; // вырожденная матрица
//             for (int j = i+1; j < 3; j++) {
//                 float factor = A_mat[j][i] / A_mat[i][i];
//                 for (int k = i; k < 4; k++) {
//                     A_mat[j][k] -= factor * A_mat[i][k];
//                 }
//             }
//         }
//         // Обратный ход
//         for (int i = 2; i >= 0; i--) {
//             float sum = 0.0f;
//             for (int j = i+1; j < 3; j++) {
//                 sum += A_mat[i][j] * X[j];
//             }
//             X[i] = (A_mat[i][3] - sum) / A_mat[i][i];
//         }
//     }
//     // Коэффициенты: X[0] = B, X[1] = D, X[2] = c
//     float B_coeff = X[0];
//     float D_coeff = X[1];
//     float c_coeff = X[2];

//     float A_est = std::sqrt(B_coeff*B_coeff + D_coeff*D_coeff);
//     float p_est = std::atan2(D_coeff, B_coeff);

//     // Записываем результаты:
//     stat_est  = c_coeff;
//     amp_est   = A_est;
//     freq_est  = w_guess / (2 * M_PI); // в Гц
//     phase_est = p_est;

//     return true;
// }
