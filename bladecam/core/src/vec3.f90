!> 3-vector primitives shared across the BladeCAM numeric core.
module vec3_mod
  use iso_c_binding, only: c_double
  implicit none
  private
  public :: dp, cross, dot3, norm3, unit3, det3

  integer, parameter :: dp = c_double

contains

  pure function cross(a, b) result(c)
    real(dp), intent(in) :: a(3), b(3)
    real(dp) :: c(3)
    c(1) = a(2)*b(3) - a(3)*b(2)
    c(2) = a(3)*b(1) - a(1)*b(3)
    c(3) = a(1)*b(2) - a(2)*b(1)
  end function cross

  pure function dot3(a, b) result(d)
    real(dp), intent(in) :: a(3), b(3)
    real(dp) :: d
    d = a(1)*b(1) + a(2)*b(2) + a(3)*b(3)
  end function dot3

  pure function norm3(a) result(n)
    real(dp), intent(in) :: a(3)
    real(dp) :: n
    n = sqrt(dot3(a, a))
  end function norm3

  pure function unit3(a) result(u)
    real(dp), intent(in) :: a(3)
    real(dp) :: u(3), n
    n = norm3(a)
    if (n > 0.0_dp) then
      u = a / n
    else
      u = a
    end if
  end function unit3

  !> Scalar triple product det[a b c] = a . (b x c)
  pure function det3(a, b, c) result(d)
    real(dp), intent(in) :: a(3), b(3), c(3)
    real(dp) :: d
    d = dot3(a, cross(b, c))
  end function det3

end module vec3_mod
