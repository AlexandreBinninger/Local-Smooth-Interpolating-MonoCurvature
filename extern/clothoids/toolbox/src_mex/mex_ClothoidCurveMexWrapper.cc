/****************************************************************************\
  Copyright (c) Enrico Bertolazzi 2016
  All Rights Reserved.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the file license.txt for more details.
\****************************************************************************/

#include "Clothoids.hh"
#include "mex_utils.hh"
#include "mex_info.hxx"

#include "mex_Workaround.hxx"

#define MEX_ERROR_MESSAGE \
"=====================================================================================\n" \
"ClothoidCurveMexWrapper:  Compute parameters of the G1 Hermite clothoid fitting\n" \
"\n" \
"USAGE:\n" \
"  - Constructors:\n" \
"    OBJ = ClothoidCurveMexWrapper( 'new' );\n" \
"    OBJ = ClothoidCurveMexWrapper( 'new', x0, y0, theta0, k0, dk, L );\n" \
"\n" \
"    On input:\n" \
"      x0, y0 = coordinate of initial point\n" \
"      theta0 = orientation (angle) of the clothoid at initial point" \
"      k0     = curvature of the clothoid at initial point\n" \
"      dk     = derivative of curvature respect to arclength\n" \
"      L      = length of the clothoid curve from initial to final point\n" \
"\n" \
"    On output:\n" \
"    OBJ = pointer to the internal object\n" \
"n" \
"  - Build:\n" \
"    ClothoidCurveMexWrapper( 'build', OBJ, x0, y0, theta0, k0, dk, L );\n" \
"    ClothoidCurveMexWrapper( 'build_G1', OBJ, x0, y0, theta0, x1, y1, theta1 );\n" \
"    [ L_D, k_D, dk_D ] = ClothoidCurveMexWrapper( 'build_G1_D', OBJ, x0, y0, theta0, x1, y1, theta1 );\n" \
"    res = ClothoidCurveMexWrapper( 'build_forward', OBJ,x0,y0,theta0,k0,x1,y1 );\n" \
"    ClothoidCurveMexWrapper( 'copy', OBJ, OBJ1 );\n" \
"\n" \
"  - Eval:\n" \
"    [x0,y0,theta0,k0,dk,smin,smax] = ClothoidCurveMexWrapper( 'getPars', OBJ );\n" \
"\n" \
"    [xp, yp, xm, ym] = ClothoidCurveMexWrapper( 'infinity', OBJ );\n" \
"\n" \
"  - Bounding Box:\n" \
"    TT = ClothoidCurveMexWrapper( 'bbox', OBJ, max_angle, max_size );%\n" \
"    TT = ClothoidCurveMexWrapper( 'bbox', OBJ, max_angle, max_size, offs );%\n" \
"\n" \
MEX_INFO_MESSAGE("ClothoidCurveMexWrapper") \
MEX_INFO_MESSAGE_END

namespace G2lib {

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  #define CMD_BASE "ClothoidCurveMexWrapper"
  #define G2LIB_CLASS ClothoidCurve
  #include "mex_common.hxx"
  #undef CMD_BASE
  #undef G2LIB_CLASS

  /*\
   |  ____    _  _____  _
   | |  _ \  / \|_   _|/ \
   | | | | |/ _ \ | | / _ \
   | | |_| / ___ \| |/ ___ \
   | |____/_/   \_\_/_/   \_\
   |
  \*/

  static
  ClothoidCurve *
  DATA_NEW( mxArray * & mx_id ) {
    ClothoidCurve * ptr = new ClothoidCurve();
    mx_id = convertPtr2Mat<ClothoidCurve>(ptr);
    return ptr;
  }

  static
  inline
  ClothoidCurve *
  DATA_GET( mxArray const * & mx_id ) {
    return convertMat2Ptr<ClothoidCurve>(mx_id);
  }

  /*\
   *                      _____                 _   _
   *  _ __ ___   _____  _|  ___|   _ _ __   ___| |_(_) ___  _ __
   * | '_ ` _ \ / _ \ \/ / |_ | | | | '_ \ / __| __| |/ _ \| '_ \
   * | | | | | |  __/>  <|  _|| |_| | | | | (__| |_| | (_) | | | |
   * |_| |_| |_|\___/_/\_\_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|
   *
  \*/

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  static
  void
  do_new(
    int nlhs, mxArray       *plhs[],
    int nrhs, mxArray const *prhs[]
  ) {

    MEX_ASSERT2(
      nlhs == 1,
      "ClothoidCurveMexWrapper, expected 1 output, nlhs = {}\n", nlhs
    );

    ClothoidCurve * ptr = DATA_NEW(arg_out_0);

    real_type x0(0), y0(0), theta0(0), k0(0), dk(0), L(0);

    if ( nrhs == 1 ) {
      // nothing to do
    } else if ( nrhs == 2 ) {

      #define CMD "ClothoidCurveMexWrapper('new',struct): "

      MEX_ASSERT( mxIsStruct(arg_in_1), CMD "Argument n.2 must be a struct" );

      mxArray * mx_x0     = mxGetField( arg_in_1, 0, "x0" );
      mxArray * mx_y0     = mxGetField( arg_in_1, 0, "y0" );
      mxArray * mx_theta0 = mxGetField( arg_in_1, 0, "theta0" );
      mxArray * mx_k0     = mxGetField( arg_in_1, 0, "k0" );
      mxArray * mx_dk     = mxGetField( arg_in_1, 0, "dk" );
      mxArray * mx_L      = mxGetField( arg_in_1, 0, "L" );

      MEX_ASSERT( mx_x0     != nullptr, CMD "Field `x0` is missing"     );
      MEX_ASSERT( mx_y0     != nullptr, CMD "Field `y0` is missing"     );
      MEX_ASSERT( mx_theta0 != nullptr, CMD "Field `theta0` is missing" );
      MEX_ASSERT( mx_k0     != nullptr, CMD "Field `k0` is missing"     );
      MEX_ASSERT( mx_dk     != nullptr, CMD "Field `dk` is missing"     );
      MEX_ASSERT( mx_L      != nullptr, CMD "Field `L` is missing"      );

      x0     = getScalarValue( mx_x0,     CMD "Field `x0` must be a real double scalar" );
      y0     = getScalarValue( mx_y0,     CMD "Field `y0` must be a real double scalar" );
      theta0 = getScalarValue( mx_theta0, CMD "Field `theta0` must be a real double scalar" );
      k0     = getScalarValue( mx_k0,     CMD "Field `k0` must be a real double scalar" );
      dk     = getScalarValue( mx_dk,     CMD "Field `dk` must be a real double scalar" );
      L      = getScalarValue( mx_L,      CMD "Field `L`  must be a real double scalar" );

      ptr->build( x0, y0, theta0, k0, dk, L );

      #undef CMD

    } else if ( nrhs == 7 ) {

      #define CMD "ClothoidCurveMexWrapper('new',x0,y0,theta0,k0,dk,L): "

      x0     = getScalarValue( arg_in_1, CMD "Error in reading x0"     );
      y0     = getScalarValue( arg_in_2, CMD "Error in reading y0"     );
      theta0 = getScalarValue( arg_in_3, CMD "Error in reading theta0" );
      k0     = getScalarValue( arg_in_4, CMD "Error in reading k0"     );
      dk     = getScalarValue( arg_in_5, CMD "Error in reading dk"     );
      L      = getScalarValue( arg_in_6, CMD "Error in reading L"      );

      ptr->build( x0, y0, theta0, k0, dk, L );

      #undef CMD

    } else {
      MEX_ASSERT2(
        false,
        "ClothoidCurveMexWrapper('new',...): expected 1, 2, or 7 inputs, nrhs = {}\n", nrhs
      );
    }
  }

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  static
  void
  do_build(
    int nlhs, mxArray       *plhs[],
    int nrhs, mxArray const *prhs[]
  ) {

    #define CMD "ClothoidCurveMexWrapper('build',OBJ,...): "
    MEX_ASSERT2( nlhs == 0, CMD "expected no output, nlhs = {}\n", nlhs );
    #undef CMD

    ClothoidCurve * ptr = DATA_GET(arg_in_1);

    real_type x0(0), y0(0), theta0(0), k0(0), dk(0), L(0);

    if ( nrhs == 2 ) {

      #define CMD "ClothoidCurveMexWrapper('build',OBJ,struct): "

      MEX_ASSERT( mxIsStruct(arg_in_1), CMD "Argument n.3 must be a struct" );

      mxArray * mx_x0     = mxGetField( arg_in_2, 0, "x0" );
      mxArray * mx_y0     = mxGetField( arg_in_2, 0, "y0" );
      mxArray * mx_theta0 = mxGetField( arg_in_2, 0, "theta0" );
      mxArray * mx_k0     = mxGetField( arg_in_2, 0, "k0" );
      mxArray * mx_dk     = mxGetField( arg_in_2, 0, "dk" );
      mxArray * mx_L      = mxGetField( arg_in_2, 0, "L" );

      MEX_ASSERT( mx_x0     != nullptr, CMD "Field `x0` is missing"     );
      MEX_ASSERT( mx_y0     != nullptr, CMD "Field `y0` is missing"     );
      MEX_ASSERT( mx_theta0 != nullptr, CMD "Field `theta0` is missing" );
      MEX_ASSERT( mx_k0     != nullptr, CMD "Field `k0` is missing"     );
      MEX_ASSERT( mx_dk     != nullptr, CMD "Field `dk` is missing"     );
      MEX_ASSERT( mx_L      != nullptr, CMD "Field `L` is missing"      );

      x0     = getScalarValue( mx_x0,     CMD "Field `x0` must be a real double scalar" );
      y0     = getScalarValue( mx_y0,     CMD "Field `y0` must be a real double scalar" );
      theta0 = getScalarValue( mx_theta0, CMD "Field `theta0` must be a real double scalar" );
      k0     = getScalarValue( mx_k0,     CMD "Field `k0` must be a real double scalar" );
      dk     = getScalarValue( mx_dk,     CMD "Field `dk` must be a real double scalar" );
      L      = getScalarValue( mx_L,      CMD "Field `L`  must be a real double scalar" );

      ptr->build( x0, y0, theta0, k0, dk, L );

      #undef CMD

    } else if ( nrhs == 8 ) {

      #define CMD "ClothoidCurveMexWrapper('build',OBJ,x0,y0,theta0,k0,dk,L): "

      x0     = getScalarValue( arg_in_2, CMD "Error in reading x0"     );
      y0     = getScalarValue( arg_in_3, CMD "Error in reading y0"     );
      theta0 = getScalarValue( arg_in_4, CMD "Error in reading theta0" );
      k0     = getScalarValue( arg_in_5, CMD "Error in reading k0"     );
      dk     = getScalarValue( arg_in_6, CMD "Error in reading dk"     );
      L      = getScalarValue( arg_in_7, CMD "Error in reading L"      );

      ptr->build( x0, y0, theta0, k0, dk, L );

      #undef CMD

    } else {
      MEX_ASSERT2(
        false,
        "ClothoidCurveMexWrapper('build',OBJ,...): expected 3, or 8 inputs, nrhs = {}\n", nrhs
      );
    }
  }

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  static
  void
  do_build_G1(
    int nlhs, mxArray       *plhs[],
    int nrhs, mxArray const *prhs[]
  ) {

    #define CMD "ClothoidCurveMexWrapper('build_G1',OBJ,x0,y0,theta0,x1,y1,theta1): "
    MEX_ASSERT2( nlhs <= 1, CMD "expected 1 or no output, nlhs = {}\n", nlhs );
    MEX_ASSERT2( nrhs == 8, CMD "expected 8 inputs, nrhs = {}\n", nrhs);

    ClothoidCurve * ptr = DATA_GET(arg_in_1);

    real_type x0(0), y0(0), theta0(0), x1(0), y1(0), theta1(0);

    x0     = getScalarValue( arg_in_2, CMD "Error in reading x0" );
    y0     = getScalarValue( arg_in_3, CMD "Error in reading y0" );
    theta0 = getScalarValue( arg_in_4, CMD "Error in reading theta0" );
    x1     = getScalarValue( arg_in_5, CMD "Error in reading x1" );
    y1     = getScalarValue( arg_in_6, CMD "Error in reading y1" );
    theta1 = getScalarValue( arg_in_7, CMD "Error in reading theta1" );

    int_type iter = ptr->build_G1( x0, y0, theta0, x1, y1, theta1 );
    if ( nlhs == 1 ) setScalarInt( arg_out_0, iter );

    #undef CMD
  }

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  static
  void
  do_build_G1_D(
    int nlhs, mxArray       *plhs[],
    int nrhs, mxArray const *prhs[]
  ) {

    #define CMD "ClothoidCurveMexWrapper('build_G1_D',OBJ,x0,y0,theta0,x1,y1,theta1): "
    MEX_ASSERT2(
      nlhs == 3 || nlhs == 4,
      CMD "expected 3 or 4 output, nlhs = {}\n", nlhs
    );
    MEX_ASSERT2(
      nrhs == 8,
      CMD "expected 8 inputs, nrhs = {}\n", nrhs
    );

    ClothoidCurve * ptr = DATA_GET(arg_in_1);

    real_type x0(0), y0(0), theta0(0), x1(0), y1(0), theta1(0);

    x0     = getScalarValue( arg_in_2, CMD "Error in reading x0" );
    y0     = getScalarValue( arg_in_3, CMD "Error in reading y0" );
    theta0 = getScalarValue( arg_in_4, CMD "Error in reading theta0" );
    x1     = getScalarValue( arg_in_5, CMD "Error in reading x1" );
    y1     = getScalarValue( arg_in_6, CMD "Error in reading y1" );
    theta1 = getScalarValue( arg_in_7, CMD "Error in reading theta1" );

    MEX_ASSERT2(
      nlhs == 3 || nlhs == 4,
      CMD "expected 3 or 4 outputs, nlhs = {}\n", nlhs
    );
    real_type * L_D  = createMatrixValue( arg_out_0, 2, 1 );
    real_type * k_D  = createMatrixValue( arg_out_1, 2, 1 );
    real_type * dk_D = createMatrixValue( arg_out_2, 2, 1 );
    int_type iter = ptr->build_G1_D( x0, y0, theta0, x1, y1, theta1, L_D, k_D, dk_D );
    if ( nlhs == 4 ) setScalarInt( arg_out_3, iter );

    #undef CMD
  }

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  static
  void
  do_build_forward(
    int nlhs, mxArray       *plhs[],
    int nrhs, mxArray const *prhs[]
  ) {

    #define CMD "ClothoidCurveMexWrapper('build_forward',OBJ,x0,y0,theta0,kappa0,x1,y1): "
    MEX_ASSERT2( nlhs == 1, CMD "expected 1 output, nlhs = {}\n", nlhs );
    MEX_ASSERT2( nrhs == 8, CMD "expected 8 inputs, nrhs = {}\n", nrhs );

    ClothoidCurve * ptr = DATA_GET(arg_in_1);

    real_type x0(0), y0(0), theta0(0), kappa0(0), x1(0), y1(0);

    x0     = getScalarValue( arg_in_2, CMD "Error in reading x0" );
    y0     = getScalarValue( arg_in_3, CMD "Error in reading y0" );
    theta0 = getScalarValue( arg_in_4, CMD "Error in reading theta0" );
    kappa0 = getScalarValue( arg_in_5, CMD "Error in reading kappa0" );
    x1     = getScalarValue( arg_in_6, CMD "Error in reading x1" );
    y1     = getScalarValue( arg_in_7, CMD "Error in reading y1" );

    bool ok = ptr->build_forward(x0, y0, theta0, kappa0, x1, y1);

    // returns the status of the interpolation
    setScalarBool( arg_out_0, ok );

    #undef CMD
  }

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  static
  void
  do_change_curvilinear_origin(
    int nlhs, mxArray       *plhs[],
    int nrhs, mxArray const *prhs[]
  ) {

    #define CMD "ClothoidCurveMexWrapper('changeCurvilinearOrigin',OBJ,s0,L): "
    MEX_ASSERT2( nrhs == 4, CMD "expected 4 inputs, nrhs = {}\n", nrhs );
    MEX_ASSERT2( nlhs == 0, CMD "expected NO outputs, nlhs = {}\n", nlhs );

    ClothoidCurve * ptr = DATA_GET(arg_in_1);

    real_type s0 = getScalarValue( arg_in_2, CMD "Error in reading s0" );
    real_type L  = getScalarValue( arg_in_3, CMD "Error in reading L"  );
    ptr->changeCurvilinearOrigin(s0,L);

    #undef CMD
  }

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  static
  void
  do_infinity(
    int nlhs, mxArray       *plhs[],
    int nrhs, mxArray const *prhs[]
  ) {

    #define CMD "ClothoidCurveMexWrapper('infinity',OBJ): "

    MEX_ASSERT2( nrhs == 2, CMD "expected 2 inputs, nrhs = {}\n", nrhs );
    MEX_ASSERT2( nlhs == 4, CMD "expected 4 output, nlhs = {}\n", nlhs );

    ClothoidCurve * ptr = DATA_GET(arg_in_1);

    real_type xp, yp, xm, ym;
    ptr->Pinfinity( xp, yp, true );
    ptr->Pinfinity( xm, ym, false );

    setScalarValue(arg_out_0, xp );
    setScalarValue(arg_out_1, yp );
    setScalarValue(arg_out_2, xm );
    setScalarValue(arg_out_3, ym );

    #undef CMD
  }

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  static
  void
  do_distance_by_sample(
    int nlhs, mxArray       *plhs[],
    int nrhs, mxArray const *prhs[]
  ) {

    #define CMD "ClothoidCurveMexWrapper('distanceBySample',OBJ,x,y,ds): "
    MEX_ASSERT2( nrhs == 5, CMD "expected 5 input, nrhs = {}\n", nrhs );

    ClothoidCurve * ptr = DATA_GET(arg_in_1);

    if ( nlhs > 0 ) {
      MEX_ASSERT2( nlhs <= 2, CMD "expected 1 or 2 output, nlhs = {}\n", nlhs );
      mwSize nrx, ncx, nry, ncy;
      real_type const * x;
      real_type const * y;
      x = getMatrixPointer(
        arg_in_2, nrx, ncx,
        CMD "`x` expected to be a real vector/matrix"
      );
      y = getMatrixPointer(
        arg_in_3, nry, ncy,
        CMD "`y` expected to be a real vector/matrix"
      );
      MEX_ASSERT2(
        nrx == nry && ncx == ncy,
        CMD "`x` and `y` expected to be of the same size, found\n"
        "size(x) = {} x {} size(y) = {} x {}\n",
        nrx, ncx, nry, ncy
      );
      real_type ds = getScalarValue( arg_in_4, CMD "`ds` expected to be a real scalar" );
      MEX_ASSERT2(
        ds > 0, CMD "`ds` = {} must be a positive number\n", ds
      );

      real_type * dst = createMatrixValue( arg_out_0, nrx, ncx );

      mwSize size = nrx*ncx;
      if ( nlhs > 1 ) {
        real_type * s = createMatrixValue( arg_out_1, nrx, ncx );
        for ( mwSize i = 0; i < size; ++i )
          *dst++ = ptr->distanceBySample( ds, *x++, *y++, *s++ );
      } else {
        for ( mwSize i = 0; i < size; ++i )
          *dst++ = ptr->distanceBySample( ds, *x++, *y++ );
      }
    }

    #undef CMD
  }

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  static
  void
  do_closest_by_sample(
    int nlhs, mxArray       *plhs[],
    int nrhs, mxArray const *prhs[]
  ) {

    #define CMD "ClothoidCurveMexWrapper('closestPointBySample',OBJ,x,y,ds): "
    MEX_ASSERT2( nrhs == 5, CMD "expected 5 input, nrhs = {}\n", nrhs);
    MEX_ASSERT2( nlhs == 4, CMD "expected 4 outputs, nlhs = {}\n", nlhs );

    ClothoidCurve * ptr = DATA_GET(arg_in_1);

    mwSize nrx, ncx, nry, ncy;
    real_type const * x;
    real_type const * y;
    x = getMatrixPointer(
      arg_in_2, nrx, ncx,
      CMD "`x` expected to be a real vector/matrix"
    );
    y = getMatrixPointer(
      arg_in_3, nry, ncy,
      CMD "`y` expected to be a real vector/matrix"
    );
    MEX_ASSERT2(
      nrx == nry && ncx == ncy,
      CMD "`x` and `y` expected to be of the same size, found\n"
      "size(x) = {} x {} size(y) = {} x {}\n",
      nrx, ncx, nry, ncy
    );

    real_type ds = getScalarValue( arg_in_4, CMD "`ds` expected to be a real scalar" );
    MEX_ASSERT2( ds > 0, CMD "`ds` = {} must be a positive number\n", ds );

    real_type * X   = createMatrixValue( arg_out_0, nrx, ncx );
    real_type * Y   = createMatrixValue( arg_out_1, nrx, ncx );
    real_type * S   = createMatrixValue( arg_out_2, nrx, ncx );
    real_type * dst = createMatrixValue( arg_out_3, nrx, ncx );

    mwSize size = nrx*ncx;
    for ( mwSize i = 0; i < size; ++i )
      *dst++ = ptr->closestPointBySample( ds, *x++, *y++, *X++, *Y++, *S++ );

    #undef CMD
  }

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  static
  void
  do_optimized_sample(
    int nlhs, mxArray       *plhs[],
    int nrhs, mxArray const *prhs[]
  ) {

    #define CMD "ClothoidCurveMexWrapper('optimized_sample',OBJ,npts,max_angle,offs[,'ISO'/'SAE']): "

    MEX_ASSERT2(
      nrhs == 5 || nrhs == 6,
      CMD "expected 5 inputs, nrhs = {}\n", nrhs
    );
    MEX_ASSERT2(
      nlhs == 1,
      CMD "expected 1 output, nlhs = {}\n", nlhs
    );

    ClothoidCurve * ptr = DATA_GET(arg_in_1);

    int_type  npts;
    real_type max_angle, offs;

    npts = int_type(getInt(
      arg_in_2, CMD "`max_angle` expected to be a real scalar"
    ));

    max_angle = getScalarValue(
      arg_in_3, CMD "`max_angle` expected to be a real scalar"
    );

    offs = getScalarValue(
      arg_in_4, CMD "`max_size` expected to be a real scalar"
    );

    bool ISO = true;
    if ( nrhs == 6 ) ISO = do_is_ISO( arg_in_5, CMD " last argument must be a string");

    std::vector<real_type> s;
    if ( ISO ) ptr->optimized_sample_ISO( offs, npts, max_angle, s );
    else       ptr->optimized_sample_SAE( offs, npts, max_angle, s );

    mwSize n = s.size();
    real_type * ss = createMatrixValue( arg_out_0, 1, n );
    std::copy( s.begin(), s.end(), ss );

    #undef CMD
  }

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  static
  void
  do_dkappa(
    int nlhs, mxArray       *plhs[],
    int nrhs, mxArray const *prhs[]
  ) {

    #define CMD "ClothoidCurveMexWrapper('dkappa',OBJ): "
    MEX_ASSERT2( nrhs == 2, CMD "expected 2 input, nrhs = {}\n", nrhs );
    MEX_ASSERT2( nlhs == 1, CMD "expected 1 output, nlhs = {}\n", nlhs );

    ClothoidCurve * ptr = DATA_GET(arg_in_1);
    setScalarValue( arg_out_0, ptr->dkappa() );

    #undef CMD
  }

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  static std::map<std::string,DO_CMD> cmd_to_fun = {
    {"new",do_new},
    {"build",do_build},
    {"build_G1",do_build_G1},
    {"build_G1_D",do_build_G1_D},
    {"build_forward",do_build_forward},
    {"changeCurvilinearOrigin",do_change_curvilinear_origin},
    {"dkappa",do_dkappa},
    {"infinity",do_infinity},
    {"distanceBySample",do_distance_by_sample},
    {"closestPointBySample",do_closest_by_sample},
    {"optimized_sample",do_optimized_sample},
    CMD_MAP_FUN
  };

  extern "C"
  void
  mexFunction(
    int nlhs, mxArray       *plhs[],
    int nrhs, mxArray const *prhs[]
  ) {

    // the first argument must be a string
    if ( nrhs == 0 ) {
      mexErrMsgTxt(MEX_ERROR_MESSAGE);
      return;
    }

    try {
      MEX_ASSERT( mxIsChar(arg_in_0), "First argument must be a string" );
      string cmd = mxArrayToString(arg_in_0);
      DO_CMD pfun = cmd_to_fun.at(cmd);
      pfun( nlhs, plhs, nrhs, prhs );
    } catch ( std::exception const & e ) {
      mexErrMsgTxt( fmt::format( "ClothoidCurve Error: {}", e.what() ).c_str() );
    } catch (...) {
      mexErrMsgTxt("ClothoidCurve failed\n");
    }
  }
}
