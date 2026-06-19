!> Unit tests for the BladeCAM numeric core. Nonzero exit code on failure.
program test_core
  use vec3_mod
  use ruled_mod
  use flank_mod
  use flank_opt_mod
  use kinematics_mod
  use topp_mod
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

end program test_core
