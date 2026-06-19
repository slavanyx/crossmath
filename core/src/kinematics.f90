!> 5-axis inverse kinematics for a table-table A-C machine.
!>
!> Convention (configurable in a real post): the workpiece sits on a C table
!> (rotation about Z) carried by an A cradle (rotation about X); the spindle
!> is fixed along +Z. Forward map (machine -> part):
!>     part_axis  = Rz(C) Rx(A) (0,0,1)
!>     part_point = Rz(C) Rx(A) (Pm - piv) + piv
!> The inverse below is the exact algebraic inverse, so a forward/inverse
!> round trip is identity (verified in the test suite).
module kinematics_mod
  use vec3_mod
  implicit none
  private
  public :: inverse_kin_ac, forward_kin_ac

contains

  pure function rotx(t) result(R)
    real(dp), intent(in) :: t
    real(dp) :: R(3,3), c, s
    c = cos(t); s = sin(t)
    R = reshape([1.0_dp,0.0_dp,0.0_dp, 0.0_dp,c,s, 0.0_dp,-s,c], [3,3])
  end function rotx

  pure function rotz(t) result(R)
    real(dp), intent(in) :: t
    real(dp) :: R(3,3), c, s
    c = cos(t); s = sin(t)
    R = reshape([c,s,0.0_dp, -s,c,0.0_dp, 0.0_dp,0.0_dp,1.0_dp], [3,3])
  end function rotz

  !> (contact point Q, tool axis O, pivot) -> machine axes [X,Y,Z,A,C] (A,C rad)
  subroutine inverse_kin_ac(Q, O, piv, m)
    real(dp), intent(in)  :: Q(3), O(3), piv(3)
    real(dp), intent(out) :: m(5)
    real(dp) :: oo(3), A, C, Pm(3)
    oo = unit3(O)
    A = acos(max(-1.0_dp, min(1.0_dp, oo(3))))
    C = atan2(oo(1), -oo(2))
    Pm = matmul(rotx(-A), matmul(rotz(-C), Q - piv)) + piv
    m(1:3) = Pm
    m(4) = A
    m(5) = C
  end subroutine inverse_kin_ac

  !> machine axes -> (contact point Q, tool axis O) in part frame
  subroutine forward_kin_ac(m, piv, Q, O)
    real(dp), intent(in)  :: m(5), piv(3)
    real(dp), intent(out) :: Q(3), O(3)
    real(dp) :: A, C, RR(3,3)
    A = m(4); C = m(5)
    RR = matmul(rotz(C), rotx(A))
    Q = matmul(RR, m(1:3) - piv) + piv
    O = matmul(RR, [0.0_dp, 0.0_dp, 1.0_dp])
  end subroutine forward_kin_ac

end module kinematics_mod
