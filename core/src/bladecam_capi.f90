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
  use kinematics_mod, only: inverse_kin_ac
  use topp_mod, only: topp_ra
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

  !> Batch 5-axis inverse kinematics: contact points Q(3,npts) and tool axes
  !> O(3,npts) -> machine axes m(5,npts) = [X,Y,Z,A,C] (A,C in radians).
  subroutine bc_ik_path(Q, O, npts, piv, m) bind(C, name="bc_ik_path")
    integer(c_int), value :: npts
    real(c_double), intent(in)  :: Q(3, npts), O(3, npts), piv(3)
    real(c_double), intent(out) :: m(5, npts)
    integer :: k
    do k = 1, npts
      call inverse_kin_ac(Q(:, k), O(:, k), piv, m(:, k))
    end do
  end subroutine bc_ik_path

  !> Time-optimal parameterization of a joint path q(ndof,n).
  subroutine bc_topp(q, ndof, n, vmax, amax, a0, aN, aprof, ttotal) &
       bind(C, name="bc_topp")
    integer(c_int), value :: ndof, n
    real(c_double), intent(in)  :: q(ndof, n), vmax(ndof), amax(ndof)
    real(c_double), value       :: a0, aN
    real(c_double), intent(out) :: aprof(n)
    real(c_double), intent(out) :: ttotal
    call topp_ra(q, ndof, n, vmax, amax, a0, aN, aprof, ttotal)
  end subroutine bc_topp

end module bladecam_capi
