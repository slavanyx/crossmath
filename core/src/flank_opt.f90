!> Phase 2: per-ruling min-max (Chebyshev) refinement of the cutter axis.
!>
!> Starting from the two-point axis, we minimise the worst-case absolute
!> envelope deviation along a ruling over 4 DOF:
!>   - axis direction tilt (2 DOF) in the plane spanned by {t1,t2} ⟂ alpha0
!>   - axis reference-point shift (2 DOF) in that same plane
!> The objective max_v |g(v)| is non-smooth, so a derivative-free
!> Nelder-Mead simplex is used.
module flank_opt_mod
  use vec3_mod
  use flank_mod, only: two_point, deviation
  implicit none
  private
  public :: refine_minmax

  ! --- objective context (module state for the Nelder-Mead callback) ---
  real(dp) :: ctx_a(3), ctx_b(3)          ! ruling endpoints
  real(dp) :: ctx_alpha0(3), ctx_q00(3)   ! two-point seed
  real(dp) :: ctx_t1(3), ctx_t2(3)        ! frame perpendicular to alpha0
  real(dp) :: ctx_R
  integer  :: ctx_nv

contains

  subroutine refine_minmax(a_pt, ap, b_pt, bp, R, nv, q0, alpha, emax)
    real(dp), intent(in)  :: a_pt(3), ap(3), b_pt(3), bp(3), R
    integer,  intent(in)  :: nv
    real(dp), intent(out) :: q0(3), alpha(3), emax
    real(dp) :: alpha0(3), q00(3), x(4), fbest, ref(3)

    ! seed with the two-point solution
    call two_point(a_pt, ap, b_pt, bp, R, q00, alpha0)

    ! build an orthonormal frame {t1,t2} spanning the plane ⟂ alpha0
    ref = [0.0_dp, 0.0_dp, 1.0_dp]
    if (abs(dot3(alpha0, ref)) > 0.9_dp) ref = [1.0_dp, 0.0_dp, 0.0_dp]
    ctx_t1 = unit3(cross(alpha0, ref))
    ctx_t2 = cross(alpha0, ctx_t1)

    ctx_a = a_pt; ctx_b = b_pt
    ctx_alpha0 = alpha0; ctx_q00 = q00
    ctx_R = R; ctx_nv = nv

    x = 0.0_dp                              ! start at the two-point point
    call nelder_mead(x, 4, 400, fbest)

    call decode(x, q0, alpha)
    emax = fbest
  end subroutine refine_minmax

  !> Map the 4 DOF to a concrete (q0, alpha).
  subroutine decode(x, q0, alpha)
    real(dp), intent(in)  :: x(4)
    real(dp), intent(out) :: q0(3), alpha(3)
    alpha = unit3(ctx_alpha0 + x(1)*ctx_t1 + x(2)*ctx_t2)
    q0    = ctx_q00 + x(3)*ctx_t1 + x(4)*ctx_t2
  end subroutine decode

  !> Objective: worst-case |g| along the ruling for parameters x.
  function objval(x) result(f)
    real(dp), intent(in) :: x(4)
    real(dp) :: f, q0(3), alpha(3), pt(3), g(1), v
    integer :: j
    call decode(x, q0, alpha)
    f = 0.0_dp
    do j = 1, ctx_nv
      v = real(j - 1, dp) / real(ctx_nv - 1, dp)
      pt = (1.0_dp - v) * ctx_a + v * ctx_b
      call deviation(q0, alpha, ctx_R, pt, 1, g)
      f = max(f, abs(g(1)))
    end do
  end function objval

  !> Compact Nelder-Mead simplex minimiser (n dims, in-place best in x).
  subroutine nelder_mead(x, n, maxiter, fbest)
    integer, intent(in)    :: n, maxiter
    real(dp), intent(inout) :: x(n)
    real(dp), intent(out)  :: fbest
    real(dp), parameter :: a_r = 1.0_dp, g_e = 2.0_dp, r_c = 0.5_dp, s_h = 0.5_dp
    real(dp), parameter :: step = 0.05_dp, tol = 1.0e-12_dp
    real(dp) :: s(n, n+1), fs(n+1), xc(n), xr(n), xe(n), xcc(n), fr, fe, fc
    integer  :: i, j, it, lo, hi, hi2

    ! initial simplex
    do j = 1, n+1
      s(:, j) = x
      if (j > 1) s(j-1, j) = s(j-1, j) + step
      fs(j) = objval(s(:, j))
    end do

    do it = 1, maxiter
      ! order: find best(lo), worst(hi), second-worst(hi2)
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

      ! centroid of all but worst
      xc = 0.0_dp
      do j = 1, n+1
        if (j /= hi) xc = xc + s(:, j)
      end do
      xc = xc / real(n, dp)

      ! reflection
      xr = xc + a_r * (xc - s(:, hi))
      fr = objval(xr)
      if (fr < fs(lo)) then
        xe = xc + g_e * (xr - xc)            ! expansion
        fe = objval(xe)
        if (fe < fr) then
          s(:, hi) = xe; fs(hi) = fe
        else
          s(:, hi) = xr; fs(hi) = fr
        end if
      else if (fr < fs(hi2)) then
        s(:, hi) = xr; fs(hi) = fr
      else
        xcc = xc + r_c * (s(:, hi) - xc)     ! contraction
        fc = objval(xcc)
        if (fc < fs(hi)) then
          s(:, hi) = xcc; fs(hi) = fc
        else
          do j = 1, n+1                      ! shrink toward best
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
    x = s(:, lo)
    fbest = fs(lo)
  end subroutine nelder_mead

end module flank_opt_mod
