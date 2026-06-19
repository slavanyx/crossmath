!> Ruled-surface differential invariants for an impeller blade flank.
!>
!> The flank is S(u,v) = (1-v)*a(u) + v*b(u), with a(u) the hub rail and
!> b(u) the shroud rail, sampled at nu stations of a UNIFORM parameter u.
!> In director form S = c(u) + v*e(u) (|e|=1) we compute:
!>
!>   distribution parameter  delta(u) = det(c',e,e') / (e'.e')
!>   striction parameter     vstar(u) = -(c'.e')/(e'.e')
!>   striction curve         sigma(u) = c(u) + vstar(u)*e(u)
!>
!> delta -> 0 marks developable/locally-cylindrical rulings (a cylinder
!> matches exactly); small |delta| marks strongly warped rulings where the
!> tangent plane twists fast and cylindrical flank error is largest.
module ruled_mod
  use vec3_mod
  implicit none
  private
  public :: distribution

contains

  subroutine distribution(a, b, nu, delta, vstar, strict)
    integer, intent(in) :: nu
    real(dp), intent(in) :: a(3, nu), b(3, nu)
    real(dp), intent(out) :: delta(nu), vstar(nu), strict(3, nu)

    integer :: i, im, ip
    real(dp) :: c(3), cp(3), e(3), ep(3), ee, du

    do i = 1, nu
      im = max(1, i - 1)
      ip = min(nu, i + 1)
      du = real(ip - im, dp)              ! index span for central difference

      c  = a(:, i)
      e  = unit3(b(:, i) - a(:, i))
      cp = (a(:, ip) - a(:, im)) / du
      ! derivative of the UNIT director (drives the twist)
      ep = (unit3(b(:, ip) - a(:, ip)) - unit3(b(:, im) - a(:, im))) / du

      ee = dot3(ep, ep)
      if (ee > 1.0e-14_dp) then
        delta(i) = det3(cp, e, ep) / ee   ! cp . (e x ep) / |ep|^2
        vstar(i) = -dot3(cp, ep) / ee
      else
        delta(i) = huge(1.0_dp)           ! locally developable / cylindrical
        vstar(i) = 0.0_dp
      end if
      strict(:, i) = c + vstar(i) * e
    end do
  end subroutine distribution

end module ruled_mod
