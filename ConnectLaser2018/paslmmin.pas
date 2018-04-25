unit paslmmin;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, math;

//
// Library:   lmfit (Levenberg-Marquardt least squares fitting)
//
// File:      lmmin.c
//
// Contents:  Levenberg-Marquardt minimization.
//
// Copyright: MINPACK authors, The University of Chikago (1980-1999)
//            Joachim Wuttke, Forschungszentrum Juelich GmbH (2004-2013)
//
// License:   see ../COPYING (FreeBSD)
// 
// Homepage:  apps.jcns.fz-juelich.de/lmfit

//*****************************************************************************/
//*  Numeric constants                                                        */
//*****************************************************************************/

//* machine-dependent constants from float.h */

const LM_MACHEP = 0.555e-16; //   DBL_EPSILON   /* resolution of arithmetic */
const LM_DWARF   = MinDouble;  //   DBL_MIN       /* smallest nonzero number */
const LM_SQRT_DWARF = sqrt(MinDouble); // sqrt(DBL_MIN) /* square should not underflow */
const LM_SQRT_GIANT = sqrt(MaxDouble); // sqrt(DBL_MAX) /* square should not overflow */
const LM_USERTOL  = 30.0 * LM_MACHEP; //  /* users are recommended to require this */

//* If the above values do not work, the following seem good for an x86:
// LM_MACHEP     .555e-16
// LM_DWARF      9.9e-324
// LM_SQRT_DWARF 1.e-160
// LM_SQRT_GIANT 1.e150
// LM_USER_TOL   1.e-14
//   The following values should work on any machine:
// LM_MACHEP     1.2e-16
// LM_DWARF      1.0e-38
// LM_SQRT_DWARF 3.834e-20
// LM_SQRT_GIANT 1.304e19
// LM_USER_TOL   1.e-14

//*****************************************************************************/
//*  Message texts (indexed by status.info)                                   */
//*****************************************************************************/

const lm_infmsg: array[0..11] of string = (
    'found zero (sum of squares below underflow limit)',
    'converged  (the relative error in the sum of squares is at most tol)',
    'converged  (the relative error of the parameter vector is at most tol)',
    'converged  (both errors are at most tol)',
    'trapped    (by degeneracy; increasing epsilon might help)',
    'exhausted  (number of function calls exceeding preset patience)',
    'failed     (ftol<tol: cannot reduce sum of squares any further)',
    'failed     (xtol<tol: cannot improve approximate solution any further)',
    'failed     (gtol<tol: cannot improve approximate solution any further)',
    'crashed    (not enough memory)',
    'exploded   (fatal coding error: improper input parameters)',
    'stopped    (break requested within function evaluation)'
);

const m_shortmsg: array[0..11] of string = (
    'found zero',
    'converged (f)',
    'converged (p)',
    'converged (2)',
    'degenerate',
    'call limit',
    'failed (f)',
    'failed (p)',
    'failed (o)',
    'no memory',
    'invalid input',
    'user break'
);


type
  // Collection of input parameters for fit control.
  lm_control_struct = record
      ftol: double;     // Relative error desired in the sum of squares.
                        // Termination occurs when both the actual and
                        // predicted relative reductions in the sum of squares
                        // are at most ftol.

      xtol: double;     // Relative error between last two approximations.
                        // Termination occurs when the relative error between
                        // two consecutive iterates is at most xtol.

      gtol: double;     // Orthogonality desired between fvec and its derivs.
                        // Termination occurs when the cosine of the angle
                        // between fvec and any column of the Jacobian is at
                        // most gtol in absolute value.

      epsilon: double;  // Step used to calculate the Jacobian, should be
                        // slightly larger than the relative error in the
                        // user-supplied functions.

      stepbound: double;// Used in determining the initial step bound. This
                        // bound is set to the product of stepbound and the
                        // Euclidean norm of diag*x if nonzero, or else to
                        // stepbound itself. In most cases stepbound should lie
                        // in the interval (0.1,100.0). Generally, the value
                        // 100.0 is recommended.

      patience: integer;// Used to set the maximum number of function evaluations
                        // to patience*(number_of_parameters+1).

      scale_diag: integer;   // If 1, the variables will be rescaled internally.
                             // Recommended value is 1.

      msgs: TStrings;        // Progress messages will be written to this object.
      verbosity: integer;    // OR'ed: 1: print some messages; 2: print Jacobian.
      n_maxpri: integer;     // -1, or max number of parameters to print.
      m_maxpri: integer;     // -1, or max number of residuals to print.
  end;
  plm_control_struct =  ^lm_control_struct;


  // Collection of output parameters for status info.
  lm_status_struct = record
      fnorm: double;       // norm of the residue vector fvec.
      nfev: integer;       // actual number of iterations.
      outcome: integer;    // Status indicator. Nonnegative values are used as index
                           // for the message text lm_infmsg, set in lmmin.c.

      userbreak: integer;  // Set when function evaluation requests termination.
  end;
  plm_status_struct = ^lm_status_struct;


const lm_control_double: lm_control_struct = (
    ftol: LM_USERTOL;
    xtol: LM_USERTOL;
    gtol: LM_USERTOL;
    epsilon: LM_USERTOL;
    stepbound: 100.0;
    patience: 100;
    scale_diag: 1;
    msgs: nil;
    verbosity: 0;
    n_maxpri: -1;
    m_maxpri: -1; );

type

//  TLMFunc = procedure(var par: array of double; m_dat: integer; data: pointer;
//                      var fvec: array of double; var userbreak: integer);
  TLMFunc = procedure(par: pdouble; m_dat: integer; data: pointer;
                      fvec: pdouble; userbreak: pinteger);



// Levenberg-Marquardt minimization.
procedure lmmin(n: integer; x: pdouble; m: integer; data: pointer;
                evaluate: TLMFunc;
                c: plm_control_struct; status: plm_status_struct);

//   This routine contains the core algorithm of our library.
//
//   It minimizes the sum of the squares of m nonlinear functions
//   in n variables by a modified Levenberg-Marquardt algorithm.
//   The function evaluation is done by the user-provided routine 'evaluate'.
//   The Jacobian is then calculated by a forward-difference approximation.
//
//   Parameters:
//
//      n is the number of variables (INPUT, positive integer).
//
//      par is the solution vector (INPUT/OUTPUT, array of length n).
//          On input it must be set to an estimated solution.
//          On output it yields the final estimate of the solution.
//
//      m_dat is the number of functions to be minimized (INPUT, positive integer).
//            It must fulfill m>=n.
//
//      data is a pointer that is ignored by lmmin; it is however forwarded
//        to the user-supplied functions evaluate and printout.
//        In a typical application, it contains experimental data to be fitted.
//
//      evaluate is a user-supplied function that calculates the m functions.
//        Parameters:
//          n, x, m, data as above.
//          fvec is an array of length m; on OUTPUT, it must contain the
//            m function values for the parameter vector x.
//          userbreak is an integer pointer. When //userbreak is set to a
//            nonzero value, lmmin will terminate.
//
//      control contains INPUT variables that control the fit algorithm,
//        as declared and explained in lmstruct.h
//
//      status contains OUTPUT variables that inform about the fit result,
//        as declared and explained in lmstruct.h

// Refined calculation of Eucledian norm.
//  double lm_enorm( int, const double * );
function lm_enorm(n: integer; x: pdouble): double;

// function declarations (implemented below).
//void lm_lmpar( int n, double *r, int ldr, int *ipvt, double *diag,
//               double *qtb, double delta, double *par, double *x,
//               double *sdiag, double *aux, double *xdi );

procedure lm_lmpar( n: integer; r: pdouble; ldr: integer; ipvt: pinteger; diag: pdouble;
                    qtb: pdouble; delta: double; par, x: pdouble;
                    sdiag, aux, xdi: pdouble);

//void lm_qrfac( int m, int n, double *a, int *ipvt,
//               double *rdiag, double *acnorm, double *wa );
procedure lm_qrfac( m, n: integer; a: pdouble; ipvt: pinteger;
                    rdiag: pdouble; acnorm: pdouble; wa: pdouble);

//void lm_qrsolv( int n, double *r, int ldr, int *ipvt, double *diag,
//                double *qtb, double *x, double *sdiag, double *wa );
procedure lm_qrsolv( n: integer; r: pdouble; ldr: integer; ipvt: pinteger; diag: pdouble;
                     qtb: pdouble; x: pdouble; sdiag: pdouble; wa: pdouble);



implementation


procedure lm_log(c: plm_control_struct; s: string);
begin
    if assigned(c^.msgs) then begin
        c^.msgs.Add(s);
    end else begin
        WriteLn(s);
    end;
end;


//*****************************************************************************/
//*  Monitoring auxiliaries.                                                  */
//*****************************************************************************/

//void lm_print_pars( int nout, const double *par, double fnorm, FILE* fout )
procedure lm_print_pars( nout: integer; par: pdouble; fnorm: double; c: plm_control_struct);//, FILE* fout )
var i: integer;
    s: string;
begin
    s := '';
    for i := 0 to nout - 1 do
         s += format(' %16.9g', [par[i]]);
    lm_log(c, s);
    lm_log(c, format(' => %18.11g', [fnorm]) );
end;



//*****************************************************************************/
//*  lmmin (main minimization routine)                                        */
//*****************************************************************************/

//void lmmin( int n, double *x, int m, const void *data,
//            void (*evaluate) (const double *par, int m_dat, const void *data,
//                              double *fvec, int *userbreak),
//            const lm_control_struct *C, lm_status_struct *S )

procedure lmmin(n: integer; x: pdouble; m: integer; data: pointer;
                evaluate: TLMFunc;
                c: plm_control_struct; status: plm_status_struct);
var
    fvec, diag, fjac, qtf, wa1, wa2, wa3, wf: pdouble;
    ipvt: pinteger;
    j, i: integer;
    actred, dirder, fnorm, fnorm1, gnorm, pnorm,
        prered, ratio, step, sum, temp, temp1, temp2, temp3: double;

    maxfev: integer;

    outer, inner: integer;  // loop counters, for monitoring
    inner_success: boolean; // flag for loop control
    lmpar: double;     // Levenberg-Marquardt parameter */
    delta: double;
    xnorm: double;
    eps: double; // for forward differences

    nout: integer;
    s: string;
const
    p0001: double = 1.0e-4;

label
    terminate;

begin
    maxfev := C^.patience * (n + 1);

    lmpar := 0;     // Levenberg-Marquardt parameter
    delta := 0;
    xnorm := 0;
    eps := sqrt(MAX(C^.epsilon, LM_MACHEP)); // for forward differences

    //nout = C->n_maxpri==-1 ? n : MIN( C->n_maxpri, n );
    if C^.n_maxpri = -1 then begin
        nout := n;
    end else
        nout := MIN( C^.n_maxpri, n );

    // Default status info; must be set ahead of first return statements
    status^.outcome := 0;      // status code
    status^.userbreak := 0;
    status^.nfev := 0;      // function evaluation counter

    // Check input parameters for errors.

    if ( n <= 0 ) then begin
        lm_log(c, Format('lmmin: invalid number of parameters %d', [n]));
        status^.outcome := 10; // invalid parameter
        exit;
    end;
    if (m < n) then begin
        lm_log(c, Format('lmmin: number of data points (%d) smaller than number of parameters (%d)', [m, n]));
        status^.outcome := 10;
        exit;
    end;
    if (C^.ftol < 0.0) or (C^.xtol < 0.0) or (C^.gtol < 0.0) then begin
        lm_log(c, Format('lmmin: negative tolerance (at least one of %g %g %g)',
                 [C^.ftol, C^.xtol, C^.gtol] ));
        status^.outcome := 10;
        exit;
    end;
    if (maxfev <= 0) then begin
        lm_log(c, Format('lmmin: nonpositive function evaluations limit %d',
                 [maxfev]));
        status^.outcome := 10;
        exit;
    end;
    if (C^.stepbound <= 0.0) then begin
        lm_log(c, Format('lmmin: nonpositive stepbound %g', [C^.stepbound]));
        status^.outcome := 10;
        exit;
    end;
    if (C^.scale_diag <> 0) and (C^.scale_diag <> 1) then begin
        lm_log(c, Format('lmmin: logical variable scale_diag=%d should be 0 or 1', [C^.scale_diag]));
        status^.outcome := 10;
        exit;
    end;


    // Allocate work space.
    fvec := GetMem(m * sizeof(double));

    diag := GetMem(n * sizeof(double));
    qtf  := GetMem(n * sizeof(double));
    fjac := GetMem(n * m * sizeof(double));
    wa1  := GetMem(n * sizeof(double));
    wa2  := GetMem(n * sizeof(double));
    wa3  := GetMem(n * sizeof(double));
    wf   := GetMem(m * sizeof(double));
    ipvt := GetMem(n * sizeof(integer));
    // alloc error -> S^.outcome := 9;

    if (C^.scale_diag = 0) then begin
        for j := 0 to n - 1 do
            diag[j] := 1.0;
    end;

    // Evaluate function at starting point and calculate norm.

    evaluate( x, m, data, fvec, @(status^.userbreak) );
    status^.nfev := 1;
    if ( status^.userbreak <> 0 ) then
        goto terminate;
    fnorm := lm_enorm(m, fvec);
    if( C^.verbosity <> 0 ) then begin
        lm_log(c, 'lmmin start');
        lm_print_pars( nout, x, fnorm, c);
    end;
    if( fnorm <= LM_DWARF ) then begin
        status^.outcome := 0; // sum of squares almost zero, nothing to do
        goto terminate;
    end;

    // The outer loop: compute gradient, then descend.

    //for( outer=0; ; ++outer ) {
    outer := 0;
    while true do begin

        // [outer]  Calculate the Jacobian.

        for j := 0 to n - 1 do begin
            temp := x[j];
            step := MAX(eps*eps, eps * abs(temp));
            x[j] += step; // replace temporarily
            evaluate( x, m, data, wf, @(status^.userbreak) );
            inc(status^.nfev);
            if ( status^.userbreak <> 0 ) then
                goto terminate;
            for i := 0 to m - 1 do
                fjac[j*m+i] := (wf[i] - fvec[i]) / step;
            x[j] := temp; // restore
        end;
        if ( C^.verbosity >= 10 ) then begin
            // print the entire matrix
            lm_log(c, '');
            lm_log(c, 'lmmin Jacobian\');
            for i := 0 to m - 1 do begin
                s := '  ';
                for j := 0 to n - 1 do
                    s += format('%.5e ', [fjac[j*m+i]]);
                lm_log(c, s);
            end;
        end;

        //  [outer]  Compute the QR factorization of the Jacobian.  ***/

//      fjac is an m by n array. The upper n by n submatrix of fjac
//        is made to contain an upper triangular matrix r with diagonal
//        elements of nonincreasing magnitude such that
//
//              p^T*(jac^T*jac)*p = r^T*r
//
//              (NOTE: ^T stands for matrix transposition),
//
//        where p is a permutation matrix and jac is the final calculated
//        Jacobian. Column j of p is column ipvt(j) of the identity matrix.
//        The lower trapezoidal part of fjac contains information generated
//        during the computation of r.
//
//      ipvt is an integer array of length n. It defines a permutation
//        matrix p such that jac*p = q*r, where jac is the final calculated
//        Jacobian, q is orthogonal (not stored), and r is upper triangular
//        with diagonal elements of nonincreasing magnitude. Column j of p
//        is column ipvt(j) of the identity matrix.
//

        lm_qrfac(m, n, fjac, ipvt, wa1, wa2, wa3);
        // return values are ipvt, wa1 = rdiag, wa2 = acnorm

        //  [outer]  Form q^T * fvec and store first n components in qtf.  ***/

        for i := 0 to m - 1 do
            wf[i] := fvec[i];

        for j := 0 to n - 1 do begin
            temp3 := fjac[j*m+j];
            if (temp3 <> 0.0) then begin
                sum := 0;
                for i := j to m - 1 do
                    sum += fjac[j*m+i] * wf[i];
                temp := -sum / temp3;
                for i := j to m - 1 do
                    wf[i] += fjac[j*m+i] * temp;
            end;
            fjac[j*m+j] := wa1[j];
            qtf[j] := wf[j];
        end;

        // [outer]  Compute norm of scaled gradient and detect degeneracy.

        gnorm := 0;
        for j := 0 to n -1 do begin
            if (wa2[ipvt[j]] = 0) then
                continue;
            sum := 0.0;
            for i := 0 to j do
                sum += fjac[j*m+i] * qtf[i];
            gnorm := MAX( gnorm, abs( sum / wa2[ipvt[j]] / fnorm ) );
        end;

        if (gnorm <= C^.gtol) then begin
            status^.outcome := 4;
            goto terminate;
        end;

        // [outer]  Initialize / update diag and delta.

        if (outer = 0) then begin
            // first iteration only
            if (C^.scale_diag <> 0) then begin
                // diag := norms of the columns of the initial Jacobian
                for j := 0 to n - 1 do begin
                    //diag[j] := wa2[j] ? wa2[j] : 1;
                    diag[j] := wa2[j];
                    if wa2[j] = 0 then diag[j] := 1;
                end;
                // xnorm := || D x ||
                for j := 0 to n -1 do
                    wa3[j] := diag[j] * x[j];
                xnorm := lm_enorm(n, wa3);
                if( C^.verbosity >= 2 ) then begin
                    lm_log(c, 'lmmin diag  ');
                    lm_print_pars(nout, x, xnorm, c);
                end;
                // only now print the header for the loop table
                if( C^.verbosity >= 3 ) then begin
                    lm_log(c, '  o  i     lmpar    prered          ratio    dirder      delta      pnorm                 fnorm');
                    s := '';
                    for i := 0 to nout - 1 do
                        s += format('               p%d', [i]);
                    lm_log(c, s);
                end;
            end else begin
                xnorm := lm_enorm(n, x);
            end;
            // initialize the step bound delta.
            if ( xnorm <> 0) then begin
                delta := C^.stepbound * xnorm;
            end else
                delta := C^.stepbound;
        end else begin
            if (C^.scale_diag <> 0) then begin
                for j := 0 to n - 1 do
                    diag[j] := MAX( diag[j], wa2[j] );
            end;
        end;

        //  The inner loop.
        inner := 0;
        //do {
        while true do begin

            //  [inner]  Determine the Levenberg-Marquardt parameter.

            lm_lmpar( n, fjac, m, ipvt, diag, qtf, delta, @lmpar,
                      wa1, wa2, wf, wa3 );
            // used return values are fjac (partly), lmpar, wa1 = x, wa3 = diag * x

            // predict scaled reduction
            pnorm := lm_enorm(n, wa3);
            temp2 := lmpar * SQR( pnorm / fnorm );
            for j := 0 to n -1 do begin
                wa3[j] := 0;
                for i := 0 to j do
                    wa3[i] -= fjac[j*m+i] * wa1[ipvt[j]];
            end;
            temp1 := SQR( lm_enorm(n, wa3) / fnorm );
            prered := temp1 + 2 * temp2;
            dirder := -temp1 + temp2; // scaled directional derivative

            // at first call, adjust the initial step bound.
            if (outer = 0) and (pnorm < delta) then
                delta := pnorm;

            //  [inner]  Evaluate the function at x + p.

            for j := 0 to n - 1 do
                wa2[j] := x[j] - wa1[j];

            evaluate( wa2, m, data, wf, @(status^.userbreak) );
            inc(status^.nfev);
            if ( status^.userbreak <> 0) then
                goto terminate;
            fnorm1 := lm_enorm(m, wf);

            // [inner]  Evaluate the scaled reduction.

            // actual scaled reduction
            actred := 1 - SQR(fnorm1/fnorm);

            // ratio of actual to predicted reduction
            //ratio := prered ? actred/prered : 0;
            if prered <> 0 then begin
                ratio := actred/prered;
            end else
                ratio := 0;

            if( C^.verbosity = 2 ) then begin
                lm_log(c, format('lmmin (%d:%d) ', [outer, inner]) );
                lm_print_pars( nout, wa2, fnorm1, c);
            end else if( C^.verbosity >= 3 ) then begin
                //lm_log(c, format('%3i %2i %9.2g %9.2g %14.6g %9.2g %10.3e %10.3e %21.15e',
                //                [outer, inner, lmpar, prered, ratio, dirder, delta, pnorm, fnorm1]) );
                s := '';
                for i := 0 to nout - 1 do
                    s += format(' %16.9g', [wa2[i]]);
                lm_log(c, s);
            end;

            // update the step bound
            if        ( ratio <= 0.25 ) then begin
                if      ( actred >= 0 ) then begin
                    temp := 0.5;
                end else if ( actred > -99 ) then begin// -99 = 1-1/0.1^2
                    temp := MAX( dirder / (2*dirder + actred), 0.1 );
                end else
                    temp := 0.1;
                delta := temp * MIN(delta, pnorm / 0.1);
                lmpar /= temp;
            end else if ( ratio >= 0.75 ) then begin
                delta := 2*pnorm;
                lmpar *= 0.5;
            end else if ( lmpar = 0 ) then begin
                delta := 2*pnorm;
            end;

            // [inner]  On success, update solution, and test for convergence.

            inner_success := ratio >= p0001;
            if ( inner_success ) then begin

                // update x, fvec, and their norms
                if (C^.scale_diag <> 0) then begin
                    for j := 0 to n - 1 do begin
                        x[j] := wa2[j];
                        wa2[j] := diag[j] * x[j];
                    end;
                end else begin
                    for j := 0 to n - 1 do
                        x[j] := wa2[j];
                end;
                for i := 0 to m - 1 do
                    fvec[i] := wf[i];
                xnorm := lm_enorm(n, wa2);
                fnorm := fnorm1;
            end;

            // convergence tests
            status^.outcome := 0;
            if( fnorm <= LM_DWARF ) then
                goto terminate;  // success: sum of squares almost zero
            // test two criteria (both may be fulfilled)
            if (abs(actred) <= C^.ftol) and (prered <= C^.ftol) and (ratio <= 2) then
                status^.outcome := 1;  // success: x almost stable
            if (delta <= C^.xtol * xnorm) then
                status^.outcome += 2; // success: sum of squares almost stable
            if (status^.outcome <> 0) then begin
                goto terminate;
            end;

            // [inner]  Tests for termination and stringent tolerances.

            if ( status^.nfev >= maxfev ) then begin
                status^.outcome := 5;
                goto terminate;
            end;
            if ( abs(actred) <= LM_MACHEP) and
                 (prered <= LM_MACHEP) and (ratio <= 2 ) then begin
                status^.outcome := 6;
                goto terminate;
            end;
            if ( delta <= LM_MACHEP*xnorm ) then begin
                status^.outcome := 7;
                goto terminate;
            end;
            if ( gnorm <= LM_MACHEP ) then begin
                status^.outcome := 8;
                goto terminate;
            end;

            // [inner]  End of the loop. Repeat if iteration unsuccessful.

            inc(inner);
            if inner_success then break
        //} while ( !inner_success );
        end;

        // [outer]  End of the loop.
        inc(outer);
    end;

terminate:
    status^.fnorm := lm_enorm(m, fvec);
    if ( C^.verbosity >= 2 ) then
        lm_log(c, Format('lmmin outcome (%d) xnorm %g ftol %g xtol %g\n',
                         [status^.outcome, xnorm, C^.ftol, C^.xtol]));
    if( C^.verbosity and 1 <> 0) then begin
        lm_log(c, 'lmmin final ');
        lm_print_pars( nout, x, status^.fnorm, c);
    end;
    if ( status^.userbreak <> 0) then // user-requested break
        status^.outcome := 11;

    // Deallocate the workspace.
    Freemem(fvec);
    Freemem(diag);
    Freemem(qtf);
    Freemem(fjac);
    Freemem(wa1);
    Freemem(wa2);
    Freemem(wa3);
    Freemem(wf);
    Freemem(ipvt);

end; // lmmin.





//*****************************************************************************/
//*  lm_lmpar (determine Levenberg-Marquardt parameter)                       */
//*****************************************************************************/
//void lm_lmpar(int n, double *r, int ldr, int *ipvt, double *diag,
//              double *qtb, double delta, double *par, double *x,
//              double *sdiag, double *aux, double *xdi)
procedure lm_lmpar( n: integer; r: pdouble; ldr: integer; ipvt: pinteger; diag: pdouble;
                    qtb: pdouble; delta: double; par, x: pdouble;
                    sdiag, aux, xdi: pdouble);

//     Given an m by n matrix a, an n by n nonsingular diagonal
//     matrix d, an m-vector b, and a positive number delta,
//     the problem is to determine a value for the parameter
//     par such that if x solves the system
//
//          a*x = b  and  sqrt(par)*d*x = 0
//
//     in the least squares sense, and dxnorm is the euclidean
//     norm of d*x, then either par=0 and (dxnorm-delta) < 0.1*delta,
//     or par>0 and abs(dxnorm-delta) < 0.1*delta.
//
//     Using lm_qrsolv, this subroutine completes the solution of the problem
//     if it is provided with the necessary information from the
//     qr factorization, with column pivoting, of a. That is, if
//     a*p = q*r, where p is a permutation matrix, q has orthogonal
//     columns, and r is an upper triangular matrix with diagonal
//     elements of nonincreasing magnitude, then lmpar expects
//     the full upper triangle of r, the permutation matrix p,
//     and the first n components of qT*b. On output
//     lmpar also provides an upper triangular matrix s such that
//
//          p^T*(a^T*a + par*d*d)*p = s^T*s.
//
//     s is employed within lmpar and may be of separate interest.
//
//     Only a few iterations are generally needed for convergence
//     of the algorithm. If, however, the limit of 10 iterations
//     is reached, then the output par will contain the best
//     value obtained so far.
//
//     parameters:
//
//      n is a positive integer input variable set to the order of r.
//
//      r is an n by n array. on input the full upper triangle
//        must contain the full upper triangle of the matrix r.
//        on OUTPUT the full upper triangle is unaltered, and the
//        strict lower triangle contains the strict upper triangle
//        (transposed) of the upper triangular matrix s.
//
//      ldr is a positive integer input variable not less than n
//        which specifies the leading dimension of the array r.
//
//      ipvt is an integer input array of length n which defines the
//        permutation matrix p such that a*p = q*r. column j of p
//        is column ipvt(j) of the identity matrix.
//
//      diag is an input array of length n which must contain the
//        diagonal elements of the matrix d.
//
//      qtb is an input array of length n which must contain the first
//        n elements of the vector (q transpose)*b.
//
//      delta is a positive input variable which specifies an upper
//        bound on the euclidean norm of d*x.
//
//      par is a nonnegative variable. on input par contains an
//        initial estimate of the levenberg-marquardt parameter.
//        on OUTPUT par contains the final estimate.
//
//      x is an OUTPUT array of length n which contains the least
//        squares solution of the system a*x = b, sqrt(par)*d*x = 0,
//        for the output par.
//
//      sdiag is an array of length n needed as workspace; on OUTPUT
//        it contains the diagonal elements of the upper triangular matrix s.
//
//      aux is a multi-purpose work array of length n.
//
//      xdi is a work array of length n. On OUTPUT: diag[j] * x[j].


var i, iter, j, nsing: integer;
    dxnorm, fp, fp_old, gnorm, parc, parl, paru: double;
    sum, temp: double;
const
    p1: double = 0.1;
begin

    // lmpar: compute and store in x the gauss-newton direction. if the
    // jacobian is rank-deficient, obtain a least squares solution.

    nsing := n;
    for j := 0 to n - 1 do begin
        aux[j] := qtb[j];
        if (r[j * ldr + j] = 0) and (nsing = n) then
            nsing := j;
        if (nsing < n) then
            aux[j] := 0;
    end;
    for j := nsing - 1 downto 0 do begin
        aux[j] := aux[j] / r[j + ldr * j];
        temp := aux[j];
        for i := 0 to j - 1 do
            aux[i] -= r[j * ldr + i] * temp;
    end;

    for j := 0 to n -1 do
        x[ipvt[j]] := aux[j];

    // lmpar: initialize the iteration counter, evaluate the function at the
    // origin, and test for acceptance of the gauss-newton direction.

    for j := 0 to n -1 do
        xdi[j] := diag[j] * x[j];
    dxnorm := lm_enorm(n, xdi);
    fp := dxnorm - delta;
    if (fp <= p1 * delta) then begin
//#ifdef LMFIT_DEBUG_MESSAGES
//        printf("debug lmpar nsing %d n %d, terminate (fp<p1*delta)\n",
//               nsing, n);
//#endif
        par^ := 0;
        exit;
    end;

    // lmpar: if the jacobian is not rank deficient, the newton
    // step provides a lower bound, parl, for the 0.0 of
    // the function. otherwise set this bound to 0.0.

    parl := 0;
    if (nsing >= n) then begin
        for j := 0 to n - 1 do
            aux[j] := diag[ipvt[j]] * xdi[ipvt[j]] / dxnorm;

        for j := 0 to n - 1 do begin
            sum := 0.0;
            for i := 0 to j - 1 do
                sum += r[j * ldr + i] * aux[i];
            aux[j] := (aux[j] - sum) / r[j + ldr * j];
        end;
        temp := lm_enorm(n, aux);
        parl := fp / delta / temp / temp;
    end;

    // lmpar: calculate an upper bound, paru, for the 0. of the function.

    for j := 0 to n -1 do begin
        sum := 0;
        for i := 0 to j do
            sum += r[j * ldr + i] * qtb[i];
        aux[j] := sum / diag[ipvt[j]];
    end;
    gnorm := lm_enorm(n, aux);
    paru := gnorm / delta;
    if (paru = 0.0) then
        paru := LM_DWARF / MIN(delta, p1);

    // lmpar: if the input par lies outside of the interval (parl,paru),
    // set par to the closer endpoint.

    par^ := MAX(par^, parl);
    par^ := MIN(par^, paru);
    if (par^ = 0.0) then
        par^ := gnorm / dxnorm;

    // lmpar: iterate.

    //for (iter=0; ; iter++) {
    iter := 0;
    while true do begin

        // evaluate the function at the current value of par.

        if (par^ = 0.0) then
            par^ := MAX(LM_DWARF, 0.001 * paru);
        temp := sqrt(par^);
        for j := 0 to n -1 do
            aux[j] := temp * diag[j];

        lm_qrsolv( n, r, ldr, ipvt, aux, qtb, x, sdiag, xdi );
        // return values are r, x, sdiag

        for j := 0 to n - 1 do
            xdi[j] := diag[j] * x[j]; // used as output
        dxnorm := lm_enorm(n, xdi);
        fp_old := fp;
        fp := dxnorm - delta;

        // if the function is small enough, accept the current value
        // of par. Also test for the exceptional cases where parl
        // is zero or the number of iterations has reached 10.

        if (abs(fp) <= p1 * delta)
            or ((parl = 0.0) and (fp <= fp_old) and (fp_old < 0.0))
            or (iter = 10) then begin
//#ifdef LMFIT_DEBUG_MESSAGES
//            printf("debug lmpar nsing %d iter %d "
//                   "par %.4e [%.4e %.4e] delta %.4e fp %.4e\n",
//                   nsing, iter, *par, parl, paru, delta, fp);
//#endif
            break; // the only exit from the iteration.
        end;

        // compute the Newton correction.

        for j := 0 to n - 1 do
            aux[j] := diag[ipvt[j]] * xdi[ipvt[j]] / dxnorm;

        for j := 0 to n - 1 do begin
            aux[j] := aux[j] / sdiag[j];
            for i := j + 1 to n - 1 do
                aux[i] -= r[j * ldr + i] * aux[j];
        end;
        temp := lm_enorm(n, aux);
        parc := fp / delta / temp / temp;

        // depending on the sign of the function, update parl or paru.

        if (fp > 0) then begin
            parl := MAX(parl, par^);
        end else if (fp < 0) then
            paru := MIN(paru, par^);
        // the case fp==0 is precluded by the break condition

        // compute an improved estimate for par.

        par^ := MAX(parl, par^ + parc);

        inc(iter);
    end;

end; //*** lm_lmpar. ***/



//*****************************************************************************/
//*  lm_qrfac (QR factorization, from lapack)                                 */
//*****************************************************************************/

//void lm_qrfac(int m, int n, double *a, int *ipvt,
//              double *rdiag, double *acnorm, double *wa)
//procedure lm_qrfac( m, n: integer; var a: array of double; var ipvt: array of integer;
//                    var rdiag: array of double; var acnorm: array of double; var wa: array of double);
procedure lm_qrfac( m, n: integer; a: pdouble; ipvt: pinteger;
                    rdiag: pdouble; acnorm: pdouble; wa: pdouble);

//     This subroutine uses Householder transformations with column
//     pivoting (optional) to compute a qr factorization of the
//     m by n matrix a. That is, qrfac determines an orthogonal
//     matrix q, a permutation matrix p, and an upper trapezoidal
//     matrix r with diagonal elements of nonincreasing magnitude,
//     such that a*p = q*r. The Householder transformation for
//     column k, k = 1,2,...,min(m,n), is of the form
//
//          i - (1/u(k))*u*uT
//
//     where u has zeroes in the first k-1 positions. The form of
//     this transformation and the method of pivoting first
//     appeared in the corresponding linpack subroutine.
//
//     Parameters:
//
//      m is a positive integer input variable set to the number
//        of rows of a.
//
//      n is a positive integer input variable set to the number
//        of columns of a.
//
//      a is an m by n array. On input a contains the matrix for
//        which the qr factorization is to be computed. On OUTPUT
//        the strict upper trapezoidal part of a contains the strict
//        upper trapezoidal part of r, and the lower trapezoidal
//        part of a contains a factored form of q (the non-trivial
//        elements of the u vectors described above).
//
//      ipvt is an integer OUTPUT array of length lipvt. This array
//        defines the permutation matrix p such that a*p = q*r.
//        Column j of p is column ipvt(j) of the identity matrix.
//
//      rdiag is an OUTPUT array of length n which contains the
//        diagonal elements of r.
//
//      acnorm is an OUTPUT array of length n which contains the
//        norms of the corresponding columns of the input matrix a.
//        If this information is not needed, then acnorm can coincide
//        with rdiag.
//
//      wa is a work array of length n.

var i, j, k, kmax, minmn: integer;
    ajnorm, sum, temp: double;

label
    pivot_ok;

begin

    // qrfac: compute initial column norms and initialize several arrays.

    for j := 0 to n - 1 do begin
        acnorm[j] := lm_enorm(m, @a[j*m]);
        rdiag[j] := acnorm[j];
        wa[j] := rdiag[j];
        ipvt[j] := j;
    end;
//#ifdef LMFIT_DEBUG_MESSAGES
//    printf("debug qrfac\n");
//#endif

    // qrfac: reduce a to r with Householder transformations.

    minmn := MIN(m, n);
    for j := 0 to minmn - 1 do begin

        // bring the column of largest norm into the pivot position.

        kmax := j;
        for k := j + 1 to n - 1 do
            if (rdiag[k] > rdiag[kmax]) then
                kmax := k;
        if (kmax = j) then
            goto pivot_ok;

        for i := 0 to m - 1 do begin
            temp := a[j*m+i];
            a[j*m+i] := a[kmax*m+i];
            a[kmax*m+i] := temp;
        end;

        rdiag[kmax] := rdiag[j];
        wa[kmax] := wa[j];
        k := ipvt[j];
        ipvt[j] := ipvt[kmax];
        ipvt[kmax] := k;

      pivot_ok:
        // compute the Householder transformation to reduce the
        // j-th column of a to a multiple of the j-th unit vector.

        ajnorm := lm_enorm(m-j, @a[j*m+j]);
        if (ajnorm = 0.0) then begin
            rdiag[j] := 0;
            continue;
        end;

        if (a[j*m+j] < 0.0) then
            ajnorm := -ajnorm;
        for i := j to m - 1 do
            a[j*m+i] /= ajnorm;
        a[j*m+j] += 1;

        // apply the transformation to the remaining columns
        //  and update the norms.

        for k := j + 1 to n -1 do begin
            sum := 0;

            for i := j to m - 1 do
                sum += a[j*m+i] * a[k*m+i];

            temp := sum / a[j + m * j];

            for i := j to m -1 do
                a[k*m+i] -= temp * a[j*m+i];

            if (rdiag[k] <> 0.0) then begin
                temp := a[m * k + j] / rdiag[k];
                temp := MAX(0.0, 1 - temp * temp);
                rdiag[k] *= sqrt(temp);
                temp := rdiag[k] / wa[k];
                if ( 0.05 * SQR(temp) <= LM_MACHEP ) then begin
                    rdiag[k] := lm_enorm(m-j-1, @a[m*k+j+1]);
                    wa[k] := rdiag[k];
                end;
            end;
        end;


        rdiag[j] := -ajnorm;
    end;
end; //*** lm_qrfac. ***/



//*****************************************************************************/
//*  lm_qrsolv (linear least-squares)                                         */
//*****************************************************************************/

//void lm_qrsolv(int n, double *r, int ldr, int *ipvt, double *diag,
//               double *qtb, double *x, double *sdiag, double *wa)
procedure lm_qrsolv( n: integer; r: pdouble; ldr: integer; ipvt: pinteger; diag: pdouble;
                     qtb: pdouble; x: pdouble; sdiag: pdouble; wa: pdouble);
//
//     Given an m by n matrix a, an n by n diagonal matrix d,
//     and an m-vector b, the problem is to determine an x which
//     solves the system

//          a*x = b  and  d*x = 0

//     in the least squares sense.

//     This subroutine completes the solution of the problem
//     if it is provided with the necessary information from the
//     qr factorization, with column pivoting, of a. That is, if
//     a*p = q*r, where p is a permutation matrix, q has orthogonal
//     columns, and r is an upper triangular matrix with diagonal
//     elements of nonincreasing magnitude, then qrsolv expects
//     the full upper triangle of r, the permutation matrix p,
//     and the first n components of (q transpose)*b. The system
//     a*x = b, d*x = 0, is then equivalent to

//          r*z = q^T*b,  p^T*d*p*z = 0,

//     where x = p*z. If this system does not have full rank,
//     then a least squares solution is obtained. On output qrsolv
//     also provides an upper triangular matrix s such that

//          p^T *(a^T *a + d*d)*p = s^T *s.

//     s is computed within qrsolv and may be of separate interest.

//     Parameters

//      n is a positive integer input variable set to the order of r.

//      r is an n by n array. On input the full upper triangle
//        must contain the full upper triangle of the matrix r.
//        On OUTPUT the full upper triangle is unaltered, and the
//        strict lower triangle contains the strict upper triangle
//        (transposed) of the upper triangular matrix s.
//
//      ldr is a positive integer input variable not less than n
//        which specifies the leading dimension of the array r.
//
//      ipvt is an integer input array of length n which defines the
//        permutation matrix p such that a*p = q*r. Column j of p
//        is column ipvt(j) of the identity matrix.
//
//      diag is an input array of length n which must contain the
//        diagonal elements of the matrix d.
//
//      qtb is an input array of length n which must contain the first
//        n elements of the vector (q transpose)*b.
//
//      x is an OUTPUT array of length n which contains the least
//        squares solution of the system a*x = b, d*x = 0.
//
//      sdiag is an OUTPUT array of length n which contains the
//        diagonal elements of the upper triangular matrix s.
//
//      wa is a work array of length n.

var i, kk, j, k, nsing: integer;
    qtbpj, sum, temp: double;
    _sin, _cos, _tan, _cot: double; // local variables, not functions

label
  L90;

begin
//*** qrsolv: copy r and q^T*b to preserve input and initialize s.
//     in particular, save the diagonal elements of r in x. ***/

    for j := 0 to n - 1 do begin
        for i := j to n - 1 do begin
            r[j * ldr + i] := r[i * ldr + j];
        end;
        x[j] := r[j * ldr + j];
        wa[j] := qtb[j];
    end;

    // qrsolv: eliminate the diagonal matrix d using a Givens rotation.

    for j := 0 to n -1 do begin

        // qrsolv: prepare the row of d to be eliminated, locating the
        // diagonal element using p from the qr factorization.

        if (diag[ipvt[j]] = 0.0) then
            goto L90;
        for k := j to n - 1 do begin
            sdiag[k] := 0.0;
        end;
        sdiag[j] := diag[ipvt[j]];

        // qrsolv: the transformations to eliminate the row of d modify only
        // a single element of qT*b beyond the first n, which is initially 0. ***/

        qtbpj := 0.0;
        for k := j to n - 1 do begin

            // determine a Givens rotation which eliminates the
            // appropriate element in the current row of d.

            if (sdiag[k] = 0.0) then
                continue;
            kk := k + ldr * k;
            if (abs(r[kk]) < abs(sdiag[k])) then begin
                _cot := r[kk] / sdiag[k];
                _sin := 1 / sqrt(1 + SQR(_cot));
                _cos := _sin * _cot;
            end else begin
                _tan := sdiag[k] / r[kk];
                _cos := 1 / sqrt(1 + SQR(_tan));
                _sin := _cos * _tan;
            end;

            // compute the modified diagonal element of r and
            // the modified element of ((q^T)*b,0).

            r[kk] := _cos * r[kk] + _sin * sdiag[k];
            temp := _cos * wa[k] + _sin * qtbpj;
            qtbpj := -_sin * wa[k] + _cos * qtbpj;
            wa[k] := temp;

            // accumulate the tranformation in the row of s.

            for i := k + 1 to n - 1 do begin
                temp := _cos * r[k * ldr + i] + _sin * sdiag[i];
                sdiag[i] := -_sin * r[k * ldr + i] + _cos * sdiag[i];
                r[k * ldr + i] := temp;
            end;
        end;

        L90:
        // store the diagonal element of s and restore
        // the corresponding diagonal element of r.

        sdiag[j] := r[j * ldr + j];
        r[j * ldr + j] := x[j];
    end;

    // qrsolv: solve the triangular system for z. if the system is
    // singular, then obtain a least squares solution.

    nsing := n;
    for j := 0 to n - 1 do begin
        if (sdiag[j] = 0.0) and (nsing = n) then
            nsing := j;
        if (nsing < n) then
            wa[j] := 0;
    end;

    for j := nsing - 1 downto 0 do begin
        sum := 0;
        for i := j + 1 to nsing - 1 do
            sum += r[j * ldr + i] * wa[i];
        wa[j] := (wa[j] - sum) / sdiag[j];
    end;

    // qrsolv: permute the components of z back to components of x.

    for j := 0 to n - 1 do
        x[ipvt[j]] := wa[j];

end; //*** lm_qrsolv. ***/




//*****************************************************************************/
//*  lm_enorm (Euclidean norm)                                                */
//*****************************************************************************/

//function lm_enorm(n: integer; constref x: array of double): double;
function lm_enorm(n: integer; x: pdouble): double;
//     Given an n-vector x, this function calculates the
//     euclidean norm of x.

//     The euclidean norm is computed by accumulating the sum of
//     squares in three different sums. The sums of squares for the
//     small and large components are scaled so that no overflows
//     occur. Non-destructive underflows are permitted. Underflows
//     and overflows do not occur in the computation of the unscaled
//     sum of squares for the intermediate components.
//     The definitions of small, intermediate and large components
//     depend on two constants, LM_SQRT_DWARF and LM_SQRT_GIANT. The main
//     restrictions on these constants are that LM_SQRT_DWARF**2 not
//     underflow and LM_SQRT_GIANT**2 not overflow.

//     Parameters

//      n is a positive integer input variable.

//      x is an input array of length n.
var i: integer;
    agiant, s1, s2, s3, xabs, x1max, x3max, temp: double;
begin
    s1 := 0;
    s2 := 0;
    s3 := 0;
    x1max := 0;
    x3max := 0;
    agiant := LM_SQRT_GIANT / n;

    // sum squares.
    for i := 0 to n - 1 do begin
        xabs := abs(x[i]);
        if (xabs > LM_SQRT_DWARF) then begin
            if ( xabs < agiant ) then begin
                s2 += xabs * xabs;
            end else if ( xabs > x1max ) then begin
                temp := x1max / xabs;
                s1 := 1 + s1 * SQR(temp);
                x1max := xabs;
            end else begin
                temp := xabs / x1max;
                s1 += SQR(temp);
            end;
        end else if ( xabs > x3max ) then begin
            temp := x3max / xabs;
            s3 := 1 + s3 * SQR(temp);
            x3max := xabs;
        end else if (xabs <> 0.0) then begin
            temp := xabs / x3max;
            s3 += SQR(temp);
        end;
    end;

    // calculation of norm.
    if (s1 <> 0) then begin
        exit(x1max * sqrt(s1 + (s2 / x1max) / x1max));
    end else if (s2 <> 0) then begin
        if (s2 >= x3max) then begin
            exit(sqrt(s2 * (1 + (x3max / s2) * (x3max * s3))));
        end else
            exit(sqrt(x3max * ((s2 / x3max) + (x3max * s3))));
    end else
        exit(x3max * sqrt(s3));

end;

end.




