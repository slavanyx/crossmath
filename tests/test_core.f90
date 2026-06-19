!> Unit tests for the BladeCAM numeric core. Nonzero exit code on failure.
program test_core
  use vec3_mod
  use ruled_mod
  use flank_mod
  use flank_opt_mod
  use kinematics_mod
  use topp_mod
  use chatter_mod
  implicit none

  integer :: nfail
  nfail = 0

  call test_deviation_basic(nfail)
  call test_two_point_developable(nfail)
  call test_distribution_cylinder(nfail)
  call test_distribution_twisted(nfail)
  call test_refine_improves(nfail)
  call test_ik_roundtrip(nfail)
  call test_topp_straight(nfail)
  call test_global_smoother(nfail)
  call test_cone_reduces_cylinder(nfail)
  call test_chatter_lobes(nfail)

  if (nfail == 0) then
    print *, "ALL TESTS PASSED"
  else
    print *, "FAILED TESTS:", nfail
    error stop 1
  end if

contains

  subroutine check(cond, name, nfail)
    logical, intent(in) :: cond
    character(*), intent(in) :: name
    integer, intent(inout) :: nfail
    if (cond) then
      print *, "  ok   ", name
    else
      print *, "  FAIL ", name
      nfail = nfail + 1
    end if
  end subroutine check

  !> deviation = dist(point,axis) - R, axis = z through origin, R = 1
  subroutine test_deviation_basic(nfail)
    integer, intent(inout) :: nfail
    real(dp) :: q0(3), alpha(3), g(3), pts(3, 3)
    q0 = [0.0_dp, 0.0_dp, 0.0_dp]
    alpha = [0.0_dp, 0.0_dp, 1.0_dp]
    pts(:, 1) = [1.0_dp, 0.0_dp, 5.0_dp]   ! on cylinder -> g = 0
    pts(:, 2) = [2.0_dp, 0.0_dp, 0.0_dp]   ! outside     -> g = 1
    pts(:, 3) = [0.0_dp, 0.5_dp, 3.0_dp]   ! inside      -> g = -0.5
    call deviation(q0, alpha, 1.0_dp, pts, 3, g)
    call check(abs(g(1)) < 1.0e-12_dp, "deviation on-surface = 0", nfail)
    call check(abs(g(2) - 1.0_dp) < 1.0e-12_dp, "deviation outside = +1", nfail)
    call check(abs(g(3) + 0.5_dp) < 1.0e-12_dp, "deviation inside = -0.5", nfail)
  end subroutine test_deviation_basic

  !> Flat (developable) ruled strip: tool axis should be ~parallel to the
  !> ruling and the whole ruling should be machined within tolerance.
  subroutine test_two_point_developable(nfail)
    integer, intent(inout) :: nfail
    real(dp) :: a_pt(3), ap(3), b_pt(3), bp(3), q0(3), alpha(3), emax, R
    R = 2.0_dp
    a_pt = [0.0_dp, 0.0_dp, 0.0_dp]
    b_pt = [0.0_dp, 0.0_dp, 10.0_dp]   ! ruling along +z
    ap   = [1.0_dp, 0.0_dp, 0.0_dp]    ! rails advance along +x (planar strip)
    bp   = [1.0_dp, 0.0_dp, 0.0_dp]
    call two_point(a_pt, ap, b_pt, bp, R, q0, alpha)
    emax = max_dev_ruling(a_pt, b_pt, q0, alpha, R, 21)
    call check(abs(abs(alpha(3)) - 1.0_dp) < 1.0e-9_dp, &
               "developable: axis parallel to ruling", nfail)
    call check(emax < 1.0e-9_dp, "developable: zero flank error", nfail)
  end subroutine test_two_point_developable

  !> A right circular cylinder is developable: e is constant so e'=0 and
  !> distribution() must flag delta as huge (non-twisting).
  subroutine test_distribution_cylinder(nfail)
    integer, intent(inout) :: nfail
    integer, parameter :: nu = 21
    real(dp) :: a(3, nu), b(3, nu), delta(nu), vstar(nu), strict(3, nu)
    integer :: i
    real(dp) :: t
    do i = 1, nu
      t = real(i - 1, dp) / real(nu - 1, dp) * 6.2831853_dp
      a(:, i) = [cos(t), sin(t), 0.0_dp]   ! base circle
      b(:, i) = [cos(t), sin(t), 1.0_dp]   ! same circle, lifted -> const director
    end do
    call distribution(a, b, nu, delta, vstar, strict)
    call check(delta(nu/2) > 1.0e6_dp, "cylinder: delta flagged huge", nfail)
  end subroutine test_distribution_cylinder

  !> A twisted (non-developable) ruled surface must yield finite, nonzero
  !> distribution parameter -> the core actually detects warp.
  subroutine test_distribution_twisted(nfail)
    integer, intent(inout) :: nfail
    integer, parameter :: nu = 41
    real(dp) :: a(3, nu), b(3, nu), delta(nu), vstar(nu), strict(3, nu)
    integer :: i
    real(dp) :: t, dval
    do i = 1, nu
      t = real(i - 1, dp) / real(nu - 1, dp)
      a(:, i) = [t, 0.0_dp, 0.0_dp]                 ! straight hub rail on x-axis
      b(:, i) = [t, cos(t), sin(t)]                 ! shroud rail rotates -> warp
    end do
    call distribution(a, b, nu, delta, vstar, strict)
    dval = delta(nu/2)
    call check(abs(dval) < 1.0e6_dp .and. abs(dval) > 1.0e-6_dp, &
               "twisted: finite nonzero delta", nfail)
  end subroutine test_distribution_twisted

  !> Min-max refinement must not be worse than the two-point seed, and on a
  !> warped ruling it should strictly reduce the peak deviation.
  subroutine test_refine_improves(nfail)
    integer, intent(inout) :: nfail
    real(dp) :: a_pt(3), ap(3), b_pt(3), bp(3), R
    real(dp) :: q0(3), alpha(3), e_two, e_ref
    R = 5.0_dp
    ! a twisted ruling: hub tangent and shroud tangent point differently
    a_pt = [0.0_dp, 0.0_dp, 0.0_dp]
    b_pt = [2.0_dp, 0.0_dp, 12.0_dp]
    ap   = [1.0_dp, 0.0_dp, 0.0_dp]
    bp   = [0.7_dp, 0.7_dp, 0.0_dp]     ! rotated tangent -> non-developable
    call two_point(a_pt, ap, b_pt, bp, R, q0, alpha)
    e_two = max_dev_ruling(a_pt, b_pt, q0, alpha, R, 41)
    call refine_minmax(a_pt, ap, b_pt, bp, R, 41, q0, alpha, e_ref)
    call check(e_ref <= e_two + 1.0e-9_dp, "refine: not worse than two-point", nfail)
    call check(e_ref < 0.99_dp * e_two, "refine: strictly improves warped ruling", nfail)
  end subroutine test_refine_improves

  !> Inverse kinematics followed by forward kinematics must recover the
  !> original contact point and tool axis.
  subroutine test_ik_roundtrip(nfail)
    integer, intent(inout) :: nfail
    real(dp) :: Q(3), O(3), piv(3), m(5), Q2(3), O2(3)
    piv = [0.0_dp, 0.0_dp, -50.0_dp]
    Q = [12.0_dp, -7.0_dp, 30.0_dp]
    O = unit3([0.3_dp, 0.4_dp, 0.866_dp])
    call inverse_kin_ac(Q, O, piv, m)
    call forward_kin_ac(m, piv, Q2, O2)
    call check(norm3(Q2 - Q) < 1.0e-9_dp, "IK round-trip: point", nfail)
    call check(norm3(O2 - O) < 1.0e-9_dp, "IK round-trip: axis", nfail)
  end subroutine test_ik_roundtrip

  !> TOPP on a single straight axis of length Lp with symmetric accel should
  !> match the trapezoidal/triangular-profile time within discretisation.
  subroutine test_topp_straight(nfail)
    integer, intent(inout) :: nfail
    integer, parameter :: n = 201
    real(dp) :: q(1, n), vmax(1), amax(1), aprof(n), ttot, Lp, tref
    integer :: k
    Lp = 100.0_dp
    do k = 1, n
      q(1, k) = Lp * real(k - 1, dp) / real(n - 1, dp)
    end do
    vmax(1) = 50.0_dp     ! mm/s
    amax(1) = 500.0_dp    ! mm/s^2
    call topp_ra(q, 1, n, vmax, amax, 0.0_dp, 0.0_dp, aprof, ttot)
    ! analytic trapezoid: t = L/vmax + vmax/amax  (reaches cruise: L > vmax^2/amax)
    tref = Lp/vmax(1) + vmax(1)/amax(1)
    call check(abs(ttot - tref) / tref < 0.02_dp, "TOPP straight-axis time", nfail)
    call check(maxval(aprof) <= vmax(1)**2 + 1.0e-6_dp, "TOPP respects vmax", nfail)
  end subroutine test_topp_straight

  !> Global optimization with a smoothness penalty (mu>0) must produce a
  !> smoother axis field than the pure min-max field (mu=0), measured by the
  !> summed squared second-difference of the cutter axis.
  subroutine test_global_smoother(nfail)
    integer, intent(inout) :: nfail
    integer, parameter :: nu = 30, nv = 31
    real(dp) :: a(3,nu), b(3,nu), ap(3,nu), bp(3,nu)
    real(dp) :: q0(3,nu), al0(3,nu), alm(3,nu), dev0(nu), devm(nu)
    real(dp) :: R, t, rough0, roughm
    integer :: i
    R = 5.0_dp
    do i = 1, nu
      t = real(i-1, dp) / real(nu-1, dp)
      a(:,i) = [10.0_dp*t, 0.0_dp, 0.0_dp]
      b(:,i) = [10.0_dp*t, 4.0_dp*cos(1.5_dp*t), 12.0_dp + 4.0_dp*sin(1.5_dp*t)]
    end do
    do i = 1, nu
      if (i == 1) then
        ap(:,i) = a(:,2)-a(:,1); bp(:,i) = b(:,2)-b(:,1)
      else if (i == nu) then
        ap(:,i) = a(:,nu)-a(:,nu-1); bp(:,i) = b(:,nu)-b(:,nu-1)
      else
        ap(:,i) = 0.5_dp*(a(:,i+1)-a(:,i-1)); bp(:,i) = 0.5_dp*(b(:,i+1)-b(:,i-1))
      end if
    end do
    call optimize_global(a, b, ap, bp, nu, R, nv, 0.0_dp,  0.0_dp, 3, q0, al0, dev0)
    call optimize_global(a, b, ap, bp, nu, R, nv, 50.0_dp, 0.0_dp, 6, q0, alm, devm)
    rough0 = axis_roughness(al0, nu)
    roughm = axis_roughness(alm, nu)
    call check(roughm < rough0, "global: penalty yields smoother axis field", nfail)
    call check(maxval(devm) < 5.0_dp, "global: deviation stays bounded", nfail)
  end subroutine test_global_smoother

  !> Conical deviation with gamma=0 must equal the cylindrical deviation;
  !> and a cone matched to a tapered point set reduces the deviation.
  subroutine test_cone_reduces_cylinder(nfail)
    integer, intent(inout) :: nfail
    real(dp) :: q0(3), alpha(3), pts(3,3), gc(3), gz(3), R, gam
    integer :: j
    R = 2.0_dp
    q0 = [0.0_dp, 0.0_dp, 0.0_dp]; alpha = [0.0_dp, 0.0_dp, 1.0_dp]
    pts(:,1) = [2.0_dp, 0.0_dp, 0.0_dp]
    pts(:,2) = [3.0_dp, 0.0_dp, 5.0_dp]
    pts(:,3) = [4.0_dp, 0.0_dp, 10.0_dp]
    call deviation(q0, alpha, R, pts, 3, gz)
    call deviation_cone(q0, alpha, R, 0.0_dp, pts, 3, gc)
    call check(maxval(abs(gc - gz)) < 1.0e-12_dp, "cone gamma=0 == cylinder", nfail)
    ! these points lie on a cone of slope (radius grows 2->4 over lambda 0->10)
    gam = atan(0.2_dp)
    call deviation_cone(q0, alpha, R, gam, pts, 3, gc)
    call check(maxval(abs(gc)) < 1.0e-9_dp, "matched cone: zero deviation", nfail)
  end subroutine test_cone_reduces_cylinder

  !> Stability lobes: depths positive/finite, and more damping raises the
  !> minimum stable depth.
  subroutine test_chatter_lobes(nfail)
    integer, intent(inout) :: nfail
    integer, parameter :: nl = 4, np = 50, ntot = nl*np
    real(dp) :: rpm(ntot), a1(ntot), a2(ntot)
    call stability_lobes(800.0_dp, 0.03_dp, 2.0e4_dp, 800.0_dp, 4, nl, np, rpm, a1)
    call stability_lobes(800.0_dp, 0.06_dp, 2.0e4_dp, 800.0_dp, 4, nl, np, rpm, a2)
    call check(all(a1 > 0.0_dp) .and. all(rpm > 0.0_dp), "lobes positive/finite", nfail)
    call check(minval(a2) > minval(a1), "more damping -> higher stable depth", nfail)
    ! REGRESSION: epsilon must be reduced mod 2*pi so the high-speed (k=0) lobe
    ! is present. Without the fix, max rpm stays ~ 60*wc/(N*2*pi) ~ 1.6e4; with
    ! it the k=0 lobe asymptotes to much higher spindle speeds.
    call check(maxval(rpm) > 3.0e4_dp, "high-speed (k=0) lobe present", nfail)
  end subroutine test_chatter_lobes

  function axis_roughness(al, nu) result(r)
    integer, intent(in) :: nu
    real(dp), intent(in) :: al(3,nu)
    real(dp) :: r, d2(3)
    integer :: i
    r = 0.0_dp
    do i = 2, nu-1
      d2 = al(:,i+1) - 2.0_dp*al(:,i) + al(:,i-1)
      r = r + dot3(d2, d2)
    end do
  end function axis_roughness

end program test_core
