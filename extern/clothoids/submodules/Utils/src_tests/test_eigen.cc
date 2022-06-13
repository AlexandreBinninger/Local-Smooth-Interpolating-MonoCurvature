/*--------------------------------------------------------------------------*\
 |                                                                          |
 |  Copyright (C) 2017                                                      |
 |                                                                          |
 |         , __                 , __                                        |
 |        /|/  \               /|/  \                                       |
 |         | __/ _   ,_         | __/ _   ,_                                | 
 |         |   \|/  /  |  |   | |   \|/  /  |  |   |                        |
 |         |(__/|__/   |_/ \_/|/|(__/|__/   |_/ \_/|/                       |
 |                           /|                   /|                        |
 |                           \|                   \|                        |
 |                                                                          |
 |      Enrico Bertolazzi                                                   |
 |      Dipartimento di Ingegneria Industriale                              |
 |      Universita` degli Studi di Trento                                   |
 |      email: enrico.bertolazzi@unitn.it                                   |
 |                                                                          |
\*--------------------------------------------------------------------------*/

#include "Utils.hh"
#include "Eigen/Core"
#include <random>

using namespace std;
typedef int    integer;
typedef double real_type;

static unsigned seed1 = 2;
static std::mt19937 generator(seed1);

static
real_type
rand( real_type xmin, real_type xmax ) {
  real_type random = real_type(generator())/generator.max();
  return xmin + (xmax-xmin)*random;
}

typedef Eigen::Matrix<real_type,Eigen::Dynamic,Eigen::Dynamic> dmat_t;
typedef Eigen::Matrix<real_type,Eigen::Dynamic,1>              dvec_t;

template <int N>
void
testVV() {

  int     N_TIMES = (1000000/N);
  double  to_ps   = 1000000.0/N_TIMES;

  typedef Eigen::Matrix<real_type,N,1> vecN_t;

  fmt::print("\nSize N = {}\n",N);

  Utils::Malloc<real_type> baseValue("real");
  Utils::Malloc<integer>   baseIndex("integer");

  baseValue.allocate(N*10);
  baseIndex.allocate(N*10);

  real_type * V1 = baseValue(N);
  real_type * V2 = baseValue(N);
  real_type * V3 = baseValue(N);

  vecN_t v1, v2, v3;
  dvec_t dv1, dv2, dv3;

  dv1.resize(N);
  dv2.resize(N);
  dv3.resize(N);

  for ( int i = 0; i < N; ++i ) {
    v1(i) = dv1(i) = V1[i] = rand(-1,1);
    v2(i) = dv2(i) = V2[i] = rand(-1,1);
    v3(i) = dv3(i) = V3[i] = rand(-1,1);
  }

  Utils::TicToc tm;

  // ===========================================================================

  tm.tic();
  for ( int i = 0; i < N_TIMES; ++i ) {
    real_type alpha = 1.0/i;
    dv3.noalias() = dv2 + alpha*dv1;
    dv1 = dv3;
  }
  tm.toc();
  fmt::print("(VV) AXPY = {:8.4} [ps] (eigen dynamic)\n", to_ps*tm.elapsed_ms() );

  // ===========================================================================

  {
    Eigen::Map<dvec_t> vv1(NULL,0), vv2(NULL,0), vv3(NULL,0);
    new (&vv1) Eigen::Map<dvec_t>(V1,N);
    new (&vv2) Eigen::Map<dvec_t>(V2,N);
    new (&vv3) Eigen::Map<dvec_t>(V3,N);
    tm.tic();
    for ( int i = 0; i < N_TIMES; ++i ) {
      real_type alpha = 1.0/i;
      vv3.noalias() = vv2 + alpha*vv1;
      vv1 = vv3;
    }
    tm.toc();
  }
  fmt::print("(VV) AXPY = {:8.4} [ps] (eigen map dynamic)\n", to_ps*tm.elapsed_ms() );

  // ===========================================================================

  tm.tic();
  for ( int i = 0; i < N_TIMES; ++i ) {
    real_type alpha = 1.0/i;
    v3.noalias() = v2 + alpha*v1;
    v2 = v3;
  }
  tm.toc();
  fmt::print("(VV) AXPY = {:8.4} [ps] (eigen fixed)\n", to_ps*tm.elapsed_ms() );

  // ===========================================================================

  {
    Eigen::Map<vecN_t> vv1(NULL), vv2(NULL), vv3(NULL);
    new (&vv1) Eigen::Map<vecN_t>(V1);
    new (&vv2) Eigen::Map<vecN_t>(V2);
    new (&vv3) Eigen::Map<vecN_t>(V3);
    tm.tic();
    for ( int i = 0; i < N_TIMES; ++i ) {
      real_type alpha = 1.0/i;
      vv3.noalias() = vv2 + alpha*vv1;
      vv1 = vv3;
    }
    tm.toc();
  }
  fmt::print("(VV) AXPY = {:8.4} [ps] (eigen fixed map)\n", to_ps*tm.elapsed_ms() );

  fmt::print("All done!\n");
}




template <int N>
void
testMM() {

  int     N_TIMES = (10000/N);
  double  to_ps   = 10000.0/N_TIMES;

  typedef Eigen::Matrix<real_type,N,N> matN_t;

  fmt::print("\nSize N = {}\n",N);

  Utils::Malloc<real_type> baseValue("real");
  Utils::Malloc<integer>   baseIndex("integer");

  baseValue.allocate(N*N*10);
  baseIndex.allocate(N*10);

  real_type * M1 = baseValue(N*N);
  real_type * M2 = baseValue(N*N);
  real_type * M3 = baseValue(N*N);

  matN_t m1, m2, m3;
  dmat_t dm1, dm2, dm3;

  dm1.resize(N,N);
  dm2.resize(N,N);
  dm3.resize(N,N);

  for ( int i = 0; i < N; ++i ) {
    for ( int j = 0; j < N; ++j ) {
      m1(i,j) = dm1(i,j) = M1[i+j*N] = rand(-1,1);
      m2(i,j) = dm2(i,j) = M2[i+j*N] = rand(-1,1);
      m3(i,j) = dm3(i,j) = M3[i+j*N] = rand(-1,1);
    }
  }

  Utils::TicToc tm;

  // ===========================================================================

  tm.tic();
  for ( int i = 0; i < N_TIMES; ++i ) {
    dm3.noalias() -= dm1*dm2;
    dm2 = dm3;
  }
  tm.toc();
  fmt::print("(MM) MULT = {:8.4} [ps] (eigen dynamic)\n", to_ps*tm.elapsed_ms() );

  // ===========================================================================

  {
    Eigen::Map<dmat_t> mm1(NULL,0,0), mm2(NULL,0,0), mm3(NULL,0,0);
    tm.tic();
    for ( int i = 0; i < N_TIMES; ++i ) {
      new (&mm1) Eigen::Map<dmat_t>(M1,N,N);
      new (&mm2) Eigen::Map<dmat_t>(M2,N,N);
      new (&mm3) Eigen::Map<dmat_t>(M3,N,N);
      mm3.noalias() -= mm1*mm2;
      mm2 = mm3;
    }
    tm.toc();
  }
  fmt::print("(MM) MULT = {:8.4} [ps] (eigen map dynamic)\n", to_ps*tm.elapsed_ms() );

  // ===========================================================================

  tm.tic();
  for ( int i = 0; i < N_TIMES; ++i ) {
    m3.noalias() -= m1*m2;
    m2 = m3;
  }
  tm.toc();
  fmt::print("(MM) MULT = {:8.4} [ps] (eigen fixed)\n", to_ps*tm.elapsed_ms() );

  // ===========================================================================

  {
    Eigen::Map<matN_t> mm1(NULL), mm2(NULL), mm3(NULL);
    tm.tic();
    for ( int i = 0; i < N_TIMES; ++i ) {
      new (&mm1) Eigen::Map<matN_t>(M1);
      new (&mm2) Eigen::Map<matN_t>(M2);
      new (&mm3) Eigen::Map<matN_t>(M3);
      mm3.noalias() -= mm1*mm2;
      mm2 = mm3;
    }
    tm.toc();
  }
  fmt::print("(MM) MULT = {:8.4} [ps] (eigen fixed map)\n", to_ps*tm.elapsed_ms() );

  // ===========================================================================

  fmt::print("All done!\n");
}


template <int N>
void
testMv() {

  int     N_TIMES = (1000000/N);
  double  to_ps   = 1000000.0/N_TIMES;

  typedef Eigen::Matrix<real_type,N,N> matN_t;
  typedef Eigen::Matrix<real_type,N,1> vecN_t;

  fmt::print("\nSize N = {}\n",N);

  Utils::Malloc<real_type> baseValue("real");
  Utils::Malloc<integer>   baseIndex("integer");

  baseValue.allocate(N*N*10);
  baseIndex.allocate(N*10);

  real_type * M = baseValue(N*N);
  real_type * V = baseValue(N);
  real_type * R = baseValue(N);

  matN_t m;
  dmat_t dm;

  vecN_t v,  r;
  dvec_t dv, dr;

  dm.resize(N,N);
  dv.resize(N);
  dr.resize(N);

  for ( int i = 0; i < N; ++i ) {
    dv(i) = v(i) = V[i] = rand(-1,1);
    dr(i) = r(i) = R[i] = rand(-1,1);
    for ( int j = 0; j < N; ++j ) {
      m(i,j) = dm(i,j) = M[i+j*N] = rand(-1,1);
    }
  }

  Utils::TicToc tm;

  // ===========================================================================

  tm.tic();
  for ( int i = 0; i < N_TIMES; ++i ) {
    dr.noalias() -= dm*dv;
    dv = dr;
  }
  tm.toc();
  fmt::print("(MV) MULT = {:8.4} [ps] (eigen dynamic)\n", to_ps*tm.elapsed_ms() );

  // ===========================================================================

  {
    Eigen::Map<dmat_t> mm(NULL,0,0);
    Eigen::Map<dvec_t> vv(NULL,0), rr(R,0);
    tm.tic();
    for ( int i = 0; i < N_TIMES; ++i ) {
      new (&mm) Eigen::Map<dmat_t>(M,N,N);
      new (&vv) Eigen::Map<dvec_t>(V,N);
      new (&rr) Eigen::Map<dvec_t>(R,N);
      rr.noalias() -= mm*vv;
      vv = rr;
    }
    tm.toc();
  }
  fmt::print("(MV) MULT = {:8.4} [ps] (eigen map dynamic)\n", to_ps*tm.elapsed_ms() );

  // ===========================================================================

  tm.tic();
  for ( int i = 0; i < N_TIMES; ++i ) {
    r.noalias() -= m*v;
    v = r;
  }
  tm.toc();
  fmt::print("(MV) MULT = {:8.4} [ps] (eigen fixed)\n", to_ps*tm.elapsed_ms() );

  // ===========================================================================

  {
    Eigen::Map<matN_t> mm(NULL);
    Eigen::Map<vecN_t> vv(NULL), rr(NULL);
    tm.tic();
    for ( int i = 0; i < N_TIMES; ++i ) {
      new (&mm) Eigen::Map<matN_t>(M);
      new (&vv) Eigen::Map<vecN_t>(V);
      new (&rr) Eigen::Map<vecN_t>(R);
      rr.noalias() -= mm*vv;
      vv = rr;
    }
    tm.toc();
  }
  fmt::print("(MV) MULT = {:8.4} [ps] (eigen fixed map)\n", to_ps*tm.elapsed_ms() );

  // ===========================================================================

  fmt::print("All done!\n");
}


template <int N>
void
testCopy() {

  int     N_TIMES = (1000000/N);
  double  to_ps   = 1000000.0/N_TIMES;

  typedef Eigen::Matrix<real_type,N,N> matN_t;

  fmt::print("\nSize N = {}\n",N);

  Utils::Malloc<real_type> baseValue("real");
  Utils::Malloc<integer>   baseIndex("integer");

  baseValue.allocate(N*N*10);
  baseIndex.allocate(N*10);

  real_type * M1 = baseValue(N*N);
  real_type * M2 = baseValue(N*N);
  real_type * M3 = baseValue(N*N);

  matN_t m1, m2, m3;
  dmat_t dm1, dm2, dm3;

  dm1.resize(N,N);
  dm2.resize(N,N);
  dm3.resize(N,N);

  for ( int i = 0; i < N; ++i ) {
    for ( int j = 0; j < N; ++j ) {
      m1(i,j) = dm1(i,j) = M1[i+j*N] = rand(-1,1);
      m2(i,j) = dm2(i,j) = M2[i+j*N] = rand(-1,1);
      m3(i,j) = dm3(i,j) = M3[i+j*N] = rand(-1,1);
    }
  }

  Utils::TicToc tm;

  // ===========================================================================

  tm.tic();
  for ( int i = 0; i < N_TIMES; ++i ) {
    dm2       = dm1;
    dm1(0,0) += dm2(0,0);
    dm2       = dm1;
    dm1(0,0) += dm2(0,0);
    dm2       = dm1;
    dm1(0,0) += dm2(0,0);
    dm2       = dm1;
    dm1(0,0) += dm2(0,0);
    dm2       = dm1;
    dm1(0,0) += dm2(0,0);
  }
  tm.toc();
  fmt::print("(MM) COPY = {:8.4} [ps] (eigen dynamic)\n", to_ps*tm.elapsed_ms() );

  // ===========================================================================
  {
    Eigen::Map<dmat_t> mm1(NULL,0,0);
    Eigen::Map<dmat_t> mm2(NULL,0,0);
    tm.tic();
    for ( int i = 0; i < N_TIMES; ++i ) {
      new (&mm1) Eigen::Map<dmat_t>(M1,N,N);
      new (&mm2) Eigen::Map<dmat_t>(M2,N,N);
      mm2       = mm1;
      mm1(0,0) += mm2(0,0);
      mm2       = mm1;
      mm1(0,0) += mm2(0,0);
      mm2       = mm1;
      mm1(0,0) += mm2(0,0);
      mm2       = mm1;
      mm1(0,0) += mm2(0,0);
      mm2       = mm1;
      mm1(0,0) += mm2(0,0);
    }
    tm.toc();
  }
  fmt::print("(MM) COPY = {:8.4} [ps] (eigen map dynamic)\n", to_ps*tm.elapsed_ms() );

  // ===========================================================================

  tm.tic();
  for ( int i = 0; i < N_TIMES; ++i ) {
    m2 = m1;
    m1(0,0) += m2(0,0);
    m2 = m1;
    m1(0,0) += m2(0,0);
    m2 = m1;
    m1(0,0) += m2(0,0);
    m2 = m1;
    m1(0,0) += m2(0,0);
    m2 = m1;
    m1(0,0) += m2(0,0);
  }
  tm.toc();
  fmt::print("(MM) COPY = {:8.4} [ps] (eigen fixed)\n", to_ps*tm.elapsed_ms() );

  // ===========================================================================

  tm.tic();
  for ( int i = 0; i < N_TIMES; ++i ) {
    Eigen::Map<matN_t> mm1(M1);
    Eigen::Map<matN_t> mm2(M2);
    mm2 = mm1;
    mm1(0,0) += mm2(0,0);
    mm2 = mm1;
    mm1(0,0) += mm2(0,0);
    mm2 = mm1;
    mm1(0,0) += mm2(0,0);
    mm2 = mm1;
    mm1(0,0) += mm2(0,0);
    mm2 = mm1;
    mm1(0,0) += mm2(0,0);
  }
  tm.toc();
  fmt::print("(MM) COPY = {:8.4} [ps] (eigen fixed map)\n", to_ps*tm.elapsed_ms() );

  fmt::print("All done!\n");
}


static
void
testVVall() {
  testVV<2>();
  //testCopy<3>();
  testVV<4>();
  //testCopy<5>();
  testVV<6>();
  //testCopy<7>();
  testVV<8>();
  //testCopy<9>();
  //testCopy<10>();
  //testCopy<11>();
  testVV<12>();
  //testCopy<13>();
  //testCopy<14>();
  //testCopy<15>();
  testVV<16>();
  //testCopy<17>();
  //testCopy<18>();
  //testCopy<19>();
  //testCopy<20>();
  testVV<100>();
  testVV<1000>();
}

static
void
testMvAll() {
  testMv<2>();
  //testMv<3>();
  testMv<4>();
  //testMv<5>();
  testMv<6>();
  //testMv<7>();
  testMv<8>();
  //testMv<9>();
  //testMv<10>();
  //testMv<11>();
  testMv<12>();
  //testMv<13>();
  //testMv<14>();
  //testMv<15>();
  testMv<16>();
  //testMv<17>();
  //testMv<18>();
  //testMv<19>();
  //testMv<20>();
  testMv<100>();
}

static
void
testMMall() {
  testMM<2>();
  //testMM<3>();
  testMM<4>();
  //testMM<5>();
  testMM<6>();
  //testMM<7>();
  testMM<8>();
  //testMM<9>();
  //testMM<10>();
  //testMM<11>();
  testMM<12>();
  //testMM<13>();
  //testMM<14>();
  //testMM<15>();
  testMM<16>();
  //testMM<17>();
  //testMM<18>();
  //testMM<19>();
  //testMM<20>();
  testMM<100>();
}

static
void
testCopyAll() {
  testCopy<2>();
  //testCopy<3>();
  testCopy<4>();
  //testCopy<5>();
  testCopy<6>();
  //testCopy<7>();
  testCopy<8>();
  //testCopy<9>();
  //testCopy<10>();
  //testCopy<11>();
  testCopy<12>();
  //testCopy<13>();
  //testCopy<14>();
  //testCopy<15>();
  testCopy<16>();
  //testCopy<17>();
  //testCopy<18>();
  //testCopy<19>();
  //testCopy<20>();
  testCopy<100>();
}

int
main() {
  Eigen::initParallel();
  Eigen::setNbThreads(4);

  testVVall();
  testMvAll();
  testMMall();
  testCopyAll();

  fmt::print("\n\nNUM THREAD {}\n",Eigen::nbThreads());
  fmt::print("\n\nAll done!\n");
  return 0;
}
