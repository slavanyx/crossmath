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
  use flank_mod,  only: two_point, deviation, deviation_cone, swept_deviation
  use flank_opt_mod, only: refine_minmax, optimize_global, optimize_double_flank
  use kinematics_mod, only: inverse_kin_ac
  use topp_mod, only: topp_ra
  use chatter_mod, only: stability_lobes, stability_lobes_frf
  use collision_mod, only: tool_clearance
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

  subroutine bc_deviation_cone(q0, alpha, R, gamma, pts, npts, g) &
       bind(C, name="bc_deviation_cone")
    integer(c_int), value :: npts
    real(c_double), intent(in)  :: q0(3), alpha(3), pts(3, npts)
    real(c_double), value       :: R, gamma
    real(c_double), intent(out) :: g(npts)
    call deviation_cone(q0, alpha, R, gamma, pts, npts, g)
  end subroutine bc_deviation_cone

  subroutine bc_refine_minmax(a_pt, ap, b_pt, bp, R, nv, q0, alpha, emax) &
       bind(C, name="bc_refine_minmax")
    real(c_double), intent(in)  :: a_pt(3), ap(3), b_pt(3), bp(3)
    real(c_double), value       :: R
    integer(c_int), value       :: nv
    real(c_double), intent(out) :: q0(3), alpha(3)
    real(c_double), intent(out) :: emax
    call refine_minmax(a_pt, ap, b_pt, bp, R, nv, q0, alpha, emax)
  end subroutine bc_refine_minmax

  !> Global envelope optimization over the whole blade: joint accuracy +
  !> smoothness via coupled block coordinate descent. Rails a,b,ap,bp are
  !> (3,nu); outputs q0,alpha (3,nu) and per-ruling peak deviation dev(nu).
  subroutine bc_optimize_global(a, b, ap, bp, nu, R, nv, mu, gamma, nsweeps, &
                                swept_w, window, q0, alpha, dev) &
       bind(C, name="bc_optimize_global")
    integer(c_int), value :: nu, nv, nsweeps, window
    real(c_double), intent(in)  :: a(3,nu), b(3,nu), ap(3,nu), bp(3,nu)
    real(c_double), value       :: R, mu, gamma, swept_w
    real(c_double), intent(out) :: q0(3,nu), alpha(3,nu), dev(nu)
    call optimize_global(a, b, ap, bp, nu, R, nv, mu, gamma, nsweeps, &
                         swept_w, window, q0, alpha, dev)
  end subroutine bc_optimize_global

  !> Double-flank channel optimization: one cylinder tangent to both walls
  !> (aL,bL) and (aR,bR), each (3,nu). Outputs axes + per-wall deviations.
  subroutine bc_optimize_double_flank(aL, bL, aR, bR, nu, R, nv, mu, gamma, &
       nsweeps, q0, alpha, devL, devR) bind(C, name="bc_optimize_double_flank")
    integer(c_int), value :: nu, nv, nsweeps
    real(c_double), intent(in)  :: aL(3,nu), bL(3,nu), aR(3,nu), bR(3,nu)
    real(c_double), value       :: R, mu, gamma
    real(c_double), intent(out) :: q0(3,nu), alpha(3,nu), devL(nu), devR(nu)
    call optimize_double_flank(aL, bL, aR, bR, nu, R, nv, mu, gamma, nsweeps, &
                               q0, alpha, devL, devR)
  end subroutine bc_optimize_double_flank

  !> Chatter stability-lobe diagram: returns rpm and limiting depth a_lim
  !> arrays of length nlobes*nptsper.
  subroutine bc_stability_lobes(wn_hz, zeta, k_stiff, Kt, n_teeth, &
                                nlobes, nptsper, rpm, alim) &
       bind(C, name="bc_stability_lobes")
    real(c_double), value :: wn_hz, zeta, k_stiff, Kt
    integer(c_int), value :: n_teeth, nlobes, nptsper
    real(c_double), intent(out) :: rpm(nlobes*nptsper), alim(nlobes*nptsper)
    call stability_lobes(wn_hz, zeta, k_stiff, Kt, n_teeth, &
                         nlobes, nptsper, rpm, alim)
  end subroutine bc_stability_lobes

  !> Stability lobes from a measured FRF. rpm/alim length nlobes*nf.
  subroutine bc_stability_lobes_frf(freq, reg, img, nf, Kt, n_teeth, nlobes, &
                                    rpm, alim) bind(C, name="bc_stability_lobes_frf")
    integer(c_int), value :: nf, n_teeth, nlobes
    real(c_double), intent(in)  :: freq(nf), reg(nf), img(nf)
    real(c_double), value       :: Kt
    real(c_double), intent(out) :: rpm(nlobes*nf), alim(nlobes*nf)
    call stability_lobes_frf(freq, reg, img, nf, Kt, n_teeth, nlobes, rpm, alim)
  end subroutine bc_stability_lobes_frf

  !> Batch 5-axis inverse kinematics: contact points Q(3,npts) and tool axes
  !> O(3,npts) -> machine axes m(5,npts) = [X,Y,Z,A,C] (A,C in radians).
  subroutine bc_ik_path(kind, Q, O, npts, piv, m) bind(C, name="bc_ik_path")
    integer(c_int), value :: kind, npts
    real(c_double), intent(in)  :: Q(3, npts), O(3, npts), piv(3)
    real(c_double), intent(out) :: m(5, npts)
    integer :: k
    do k = 1, npts
      call inverse_kin_ac(kind, Q(:, k), O(:, k), piv, m(:, k))
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

  !> Swept-envelope deviation of design points vs the whole toolpath.
  subroutine bc_swept_deviation(q0, alpha, Lflute, R, nu, pts, npts, g) &
       bind(C, name="bc_swept_deviation")
    integer(c_int), value :: nu, npts
    real(c_double), intent(in)  :: q0(3,nu), alpha(3,nu), Lflute(nu), pts(3,npts)
    real(c_double), value       :: R
    real(c_double), intent(out) :: g(npts)
    call swept_deviation(q0, alpha, Lflute, R, nu, pts, npts, g)
  end subroutine bc_swept_deviation

  !> Tool+holder clearance to an obstacle cloud, per station. clr(nu) < 0 means
  !> collision/gouge.
  subroutine bc_tool_clearance(q0, alpha, R, Lf, Rh, gap, Lh, nu, pts, npts, clr) &
       bind(C, name="bc_tool_clearance")
    integer(c_int), value :: nu, npts
    real(c_double), intent(in)  :: q0(3,nu), alpha(3,nu), pts(3,npts)
    real(c_double), value       :: R, Lf, Rh, gap, Lh
    real(c_double), intent(out) :: clr(nu)
    call tool_clearance(q0, alpha, R, Lf, Rh, gap, Lh, nu, pts, npts, clr)
  end subroutine bc_tool_clearance

end module bladecam_capi
