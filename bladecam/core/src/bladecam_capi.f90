!> C ABI for the BladeCAM core (consumed from Python via ctypes, or any C host).
!>
!> Array layout: a NumPy array of shape (n, 3) that is C-contiguous has the
!> same memory order as a Fortran array declared (3, n) -- i.e. xyz of point j
!> are contiguous. The wrappers below therefore declare (3, n) and expect the
!> caller to pass (n, 3) C-contiguous float64 buffers.
module bladecam_capi
  use iso_c_binding, only: c_int, c_double
  use vec3_mod,   only: dp
  use ruled_mod,  only: distribution
  use flank_mod,  only: two_point, deviation
  use flank_opt_mod, only: refine_minmax
  implicit none

contains

  subroutine bc_distribution(a, b, nu, delta, vstar, strict) &
       bind(C, name="bc_distribution")
    integer(c_int), value :: nu
    real(c_double), intent(in)  :: a(3, nu), b(3, nu)
    real(c_double), intent(out) :: delta(nu), vstar(nu), strict(3, nu)
    call distribution(a, b, nu, delta, vstar, strict)
  end subroutine bc_distribution

  subroutine bc_two_point(a_pt, ap, b_pt, bp, R, q0, alpha) &
       bind(C, name="bc_two_point")
    real(c_double), intent(in)  :: a_pt(3), ap(3), b_pt(3), bp(3)
    real(c_double), value       :: R
    real(c_double), intent(out) :: q0(3), alpha(3)
    call two_point(a_pt, ap, b_pt, bp, R, q0, alpha)
  end subroutine bc_two_point

  subroutine bc_deviation(q0, alpha, R, pts, npts, g) &
       bind(C, name="bc_deviation")
    integer(c_int), value :: npts
    real(c_double), intent(in)  :: q0(3), alpha(3), pts(3, npts)
    real(c_double), value       :: R
    real(c_double), intent(out) :: g(npts)
    call deviation(q0, alpha, R, pts, npts, g)
  end subroutine bc_deviation

  subroutine bc_refine_minmax(a_pt, ap, b_pt, bp, R, nv, q0, alpha, emax) &
       bind(C, name="bc_refine_minmax")
    real(c_double), intent(in)  :: a_pt(3), ap(3), b_pt(3), bp(3)
    real(c_double), value       :: R
    integer(c_int), value       :: nv
    real(c_double), intent(out) :: q0(3), alpha(3)
    real(c_double), intent(out) :: emax
    call refine_minmax(a_pt, ap, b_pt, bp, R, nv, q0, alpha, emax)
  end subroutine bc_refine_minmax

end module bladecam_capi
