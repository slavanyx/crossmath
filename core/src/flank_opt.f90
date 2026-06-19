!> Per-ruling and GLOBAL cutter-axis optimization for flank milling.
!>
!> refine_seeded : 4-DOF Nelder-Mead minimisation of an objective that combines
!>   the worst-case envelope deviation of a ruling with an optional smoothness
!>   penalty pulling the axis toward a neighbour target.
!> refine_minmax : pure min-max (penalty off), seeded from the two-point axis.
!> optimize_global : Gauss-Seidel block coordinate descent over all rulings
!>   minimising  J = sum_i max_v|g_i|  +  mu * sum_i ||axis_i - neighbour_avg||^2.
!>   This couples neighbours, so accuracy and orientation smoothness are
!>   optimised jointly (unlike min-max followed by a separate smoothing pass).
module flank_opt_mod
  use vec3_mod
  use flank_mod, only: two_point, deviation_cone
  implicit none
  private
  public :: refine_minmax, optimize_global, optimize_double_flank

  ! --- objective context (module state for the Nelder-Mead callback) ---
  real(dp) :: ctx_a(3), ctx_b(3)
  real(dp) :: ctx_a2(3), ctx_b2(3)      ! second wall (double-flank)
  real(dp) :: ctx_alpha0(3), ctx_q00(3)
  real(dp) :: ctx_t1(3), ctx_t2(3)
  real(dp) :: ctx_R
  integer  :: ctx_nv
  logical  :: ctx_double = .false.      ! evaluate both walls
  real(dp) :: ctx_mu = 0.0_dp           ! smoothness penalty weight
  real(dp) :: ctx_alpha_nb(3) = 0.0_dp  ! neighbour axis target
  real(dp) :: ctx_q0_nb(3) = 0.0_dp     ! neighbour point target
  real(dp) :: ctx_gamma = 0.0_dp        ! tool taper half-angle (0 = cylinder)
  real(dp), parameter :: ctx_wq = 0.01_dp  ! relative weight of point penalty

contains

  subroutine refine_minmax(a_pt, ap, b_pt, bp, R, nv, q0, alpha, emax)
    real(dp), intent(in)  :: a_pt(3), ap(3), b_pt(3), bp(3), R
    integer,  intent(in)  :: nv
    real(dp), intent(out) :: q0(3), alpha(3), emax
    real(dp) :: alpha0(3), q00(3)
    ctx_gamma = 0.0_dp
    call two_point(a_pt, ap, b_pt, bp, R, q00, alpha0)
    call refine_seeded(a_pt, b_pt, R, nv, alpha0, q00, 0.0_dp, &
                       alpha0, q00, q0, alpha, emax)
  end subroutine refine_minmax

  subroutine optimize_global(a, b, ap, bp, nu, R, nv, mu, gamma, nsweeps, &
                             q0, alpha, dev)
    integer,  intent(in)  :: nu, nv, nsweeps
    real(dp), intent(in)  :: a(3,nu), b(3,nu), ap(3,nu), bp(3,nu), R, mu, gamma
    real(dp), intent(out) :: q0(3,nu), alpha(3,nu), dev(nu)
    integer  :: i, sw, lo, hi
    real(dp) :: anb(3), qnb(3), e
    ctx_gamma = gamma

    ! initialise from two-point
    do i = 1, nu
      call two_point(a(:,i), ap(:,i), b(:,i), bp(:,i), R, q0(:,i), alpha(:,i))
    end do

    do sw = 1, nsweeps
      do i = 1, nu
        lo = max(1, i-1); hi = min(nu, i+1)
        if (i == 1) then
          anb = alpha(:,hi); qnb = q0(:,hi)
        else if (i == nu) then
          anb = alpha(:,lo); qnb = q0(:,lo)
        else
          anb = unit3(alpha(:,lo) + alpha(:,hi))
          qnb = 0.5_dp * (q0(:,lo) + q0(:,hi))
        end if
        call refine_seeded(a(:,i), b(:,i), R, nv, alpha(:,i), q0(:,i), mu, &
                           anb, qnb, q0(:,i), alpha(:,i), e)
      end do
    end do

    ! report pure peak deviation per ruling
    do i = 1, nu
      dev(i) = peak_dev(a(:,i), b(:,i), q0(:,i), alpha(:,i), R, nv)
    end do
  end subroutine optimize_global

  !> Double-flank channel milling: one cylinder tangent to BOTH walls (this
  !> blade's wall aL,bL and the facing wall aR,bR), per ruling-pair, minimising
  !> the worse of the two wall deviations. Returns axes and per-wall deviations.
  subroutine optimize_double_flank(aL, bL, aR, bR, nu, R, nv, mu, gamma, &
                                   nsweeps, q0, alpha, devL, devR)
    integer,  intent(in)  :: nu, nv, nsweeps
    real(dp), intent(in)  :: aL(3,nu), bL(3,nu), aR(3,nu), bR(3,nu)
    real(dp), intent(in)  :: R, mu, gamma
    real(dp), intent(out) :: q0(3,nu), alpha(3,nu), devL(nu), devR(nu)
    integer  :: i, sw, lo, hi
    real(dp) :: anb(3), qnb(3), seed_a(3), seed_q(3)

    ctx_gamma = gamma
    ! seed: axis centred in the channel, along the mean ruling direction
    do i = 1, nu
      seed_q = 0.25_dp*(aL(:,i)+bL(:,i)+aR(:,i)+bR(:,i))
      seed_a = unit3((bL(:,i)-aL(:,i)) + (bR(:,i)-aR(:,i)))
      q0(:,i) = seed_q; alpha(:,i) = seed_a
    end do

    do sw = 1, nsweeps
      do i = 1, nu
        lo = max(1, i-1); hi = min(nu, i+1)
        if (i == 1) then
          anb = alpha(:,hi); qnb = q0(:,hi)
        else if (i == nu) then
          anb = alpha(:,lo); qnb = q0(:,lo)
        else
          anb = unit3(alpha(:,lo) + alpha(:,hi))
          qnb = 0.5_dp * (q0(:,lo) + q0(:,hi))
        end if
        call refine_double(aL(:,i), bL(:,i), aR(:,i), bR(:,i), R, nv, &
                           alpha(:,i), q0(:,i), mu, anb, qnb, q0(:,i), alpha(:,i))
      end do
    end do

    do i = 1, nu
      devL(i) = peak_dev(aL(:,i), bL(:,i), q0(:,i), alpha(:,i), R, nv)
      devR(i) = peak_dev(aR(:,i), bR(:,i), q0(:,i), alpha(:,i), R, nv)
    end do
  end subroutine optimize_double_flank

  subroutine refine_double(aL, bL, aR, bR, R, nv, alpha_seed, q0_seed, mu, &
                           alpha_nb, q0_nb, q0, alpha)
    real(dp), intent(in)  :: aL(3), bL(3), aR(3), bR(3), R
    real(dp), intent(in)  :: alpha_seed(3), q0_seed(3), mu, alpha_nb(3), q0_nb(3)
    integer,  intent(in)  :: nv
    real(dp), intent(out) :: q0(3), alpha(3)
    real(dp) :: ref(3), x(4), fbest

    ref = [0.0_dp, 0.0_dp, 1.0_dp]
    if (abs(dot3(alpha_seed, ref)) > 0.9_dp) ref = [1.0_dp, 0.0_dp, 0.0_dp]
    ctx_t1 = unit3(cross(alpha_seed, ref))
    ctx_t2 = cross(alpha_seed, ctx_t1)
    ctx_a = aL; ctx_b = bL; ctx_a2 = aR; ctx_b2 = bR
    ctx_alpha0 = alpha_seed; ctx_q00 = q0_seed
    ctx_R = R; ctx_nv = nv
    ctx_mu = mu; ctx_alpha_nb = alpha_nb; ctx_q0_nb = q0_nb
    ctx_double = .true.
    x = 0.0_dp
    call nelder_mead(x, 4, 500, fbest)
    ctx_double = .false.
    call decode(x, q0, alpha)
  end subroutine refine_double

  ! ---- internals ----------------------------------------------------------

  subroutine refine_seeded(a_pt, b_pt, R, nv, alpha_seed, q0_seed, mu, &
                           alpha_nb, q0_nb, q0, alpha, emax_pure)
    real(dp), intent(in)  :: a_pt(3), b_pt(3), R, alpha_seed(3), q0_seed(3)
    real(dp), intent(in)  :: mu, alpha_nb(3), q0_nb(3)
    integer,  intent(in)  :: nv
    real(dp), intent(out) :: q0(3), alpha(3), emax_pure
    real(dp) :: ref(3), x(4), fbest

    ref = [0.0_dp, 0.0_dp, 1.0_dp]
    if (abs(dot3(alpha_seed, ref)) > 0.9_dp) ref = [1.0_dp, 0.0_dp, 0.0_dp]
    ctx_t1 = unit3(cross(alpha_seed, ref))
    ctx_t2 = cross(alpha_seed, ctx_t1)

    ctx_a = a_pt; ctx_b = b_pt
    ctx_alpha0 = alpha_seed; ctx_q00 = q0_seed
    ctx_R = R; ctx_nv = nv
    ctx_mu = mu; ctx_alpha_nb = alpha_nb; ctx_q0_nb = q0_nb

    x = 0.0_dp
    call nelder_mead(x, 4, 400, fbest)
    call decode(x, q0, alpha)
    emax_pure = peak_dev(a_pt, b_pt, q0, alpha, R, nv)
  end subroutine refine_seeded

  function peak_dev(a_pt, b_pt, q0, alpha, R, nv) result(emax)
    real(dp), intent(in) :: a_pt(3), b_pt(3), q0(3), alpha(3), R
    integer,  intent(in) :: nv
    real(dp) :: emax, v, pt(3), g(1)
    integer :: j
    emax = 0.0_dp
    do j = 1, nv
      v = real(j - 1, dp) / real(nv - 1, dp)
      pt = (1.0_dp - v) * a_pt + v * b_pt
      call deviation_cone(q0, alpha, R, ctx_gamma, pt, 1, g)
      emax = max(emax, abs(g(1)))
    end do
  end function peak_dev

  subroutine decode(x, q0, alpha)
    real(dp), intent(in)  :: x(4)
    real(dp), intent(out) :: q0(3), alpha(3)
    alpha = unit3(ctx_alpha0 + x(1)*ctx_t1 + x(2)*ctx_t2)
    q0    = ctx_q00 + x(3)*ctx_t1 + x(4)*ctx_t2
  end subroutine decode

  function objval(x) result(f)
    real(dp), intent(in) :: x(4)
    real(dp) :: f, q0(3), alpha(3), pt(3), g(1), v
    integer :: j
    call decode(x, q0, alpha)
    f = 0.0_dp
    do j = 1, ctx_nv
      v = real(j - 1, dp) / real(ctx_nv - 1, dp)
      pt = (1.0_dp - v) * ctx_a + v * ctx_b
      call deviation_cone(q0, alpha, ctx_R, ctx_gamma, pt, 1, g)
      f = max(f, abs(g(1)))
    end do
    if (ctx_double) then          ! second wall (double-flank channel)
      do j = 1, ctx_nv
        v = real(j - 1, dp) / real(ctx_nv - 1, dp)
        pt = (1.0_dp - v) * ctx_a2 + v * ctx_b2
        call deviation_cone(q0, alpha, ctx_R, ctx_gamma, pt, 1, g)
        f = max(f, abs(g(1)))
      end do
    end if
    if (ctx_mu > 0.0_dp) then
      f = f + ctx_mu * ( dot3(alpha - ctx_alpha_nb, alpha - ctx_alpha_nb) &
                       + ctx_wq * dot3(q0 - ctx_q0_nb, q0 - ctx_q0_nb) )
    end if
  end function objval

  subroutine nelder_mead(x, n, maxiter, fbest)
    integer, intent(in)    :: n, maxiter
    real(dp), intent(inout) :: x(n)
    real(dp), intent(out)  :: fbest
    real(dp), parameter :: a_r = 1.0_dp, g_e = 2.0_dp, r_c = 0.5_dp, s_h = 0.5_dp
    real(dp), parameter :: step = 0.05_dp, tol = 1.0e-12_dp
    real(dp) :: s(n, n+1), fs(n+1), xc(n), xr(n), xe(n), xcc(n), fr, fe, fc
    integer  :: j, it, lo, hi, hi2

    do j = 1, n+1
      s(:, j) = x
    end do
    do j = 1, n                 ! vertex j+1 perturbs coordinate j
      s(j, j+1) = s(j, j+1) + step
    end do
    do j = 1, n+1
      fs(j) = objval(s(:, j))
    end do

    do it = 1, maxiter
      lo = 1; hi = 1
      do j = 2, n+1
        if (fs(j) < fs(lo)) lo = j
        if (fs(j) > fs(hi)) hi = j
      end do
      hi2 = lo
      do j = 1, n+1
        if (j /= hi .and. fs(j) > fs(hi2)) hi2 = j
      end do
      if (abs(fs(hi) - fs(lo)) < tol) exit

      xc = 0.0_dp
      do j = 1, n+1
        if (j /= hi) xc = xc + s(:, j)
      end do
      xc = xc / real(n, dp)

      xr = xc + a_r * (xc - s(:, hi)); fr = objval(xr)
      if (fr < fs(lo)) then
        xe = xc + g_e * (xr - xc); fe = objval(xe)
        if (fe < fr) then; s(:, hi) = xe; fs(hi) = fe
        else;              s(:, hi) = xr; fs(hi) = fr; end if
      else if (fr < fs(hi2)) then
        s(:, hi) = xr; fs(hi) = fr
      else
        xcc = xc + r_c * (s(:, hi) - xc); fc = objval(xcc)
        if (fc < fs(hi)) then
          s(:, hi) = xcc; fs(hi) = fc
        else
          do j = 1, n+1
            if (j /= lo) then
              s(:, j) = s(:, lo) + s_h * (s(:, j) - s(:, lo))
              fs(j) = objval(s(:, j))
            end if
          end do
        end if
      end if
    end do

    lo = 1
    do j = 2, n+1
      if (fs(j) < fs(lo)) lo = j
    end do
    x = s(:, lo); fbest = fs(lo)
  end subroutine nelder_mead

end module flank_opt_mod
