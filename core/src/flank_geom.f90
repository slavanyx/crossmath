!> Flank-milling tool positioning and envelope-deviation evaluation.
module flank_mod
  use vec3_mod
  implicit none
  private
  public :: two_point, deviation, max_dev_ruling

contains

  !> Two-point cutter positioning (Bedi/Mann/Menzel) for one ruling.
  !> Tangency is enforced at both ruling ends by offsetting each endpoint
  !> by R along the local surface normal; the cutter axis joins the two
  !> offset points. ap = a'(u), bp = b'(u) are the rail tangents.
  subroutine two_point(a_pt, ap, b_pt, bp, R, q0, alpha)
    real(dp), intent(in)  :: a_pt(3), ap(3), b_pt(3), bp(3), R
    real(dp), intent(out) :: q0(3), alpha(3)
    real(dp) :: ruling(3), n0(3), n1(3), A0(3), A1(3)

    ruling = b_pt - a_pt
    ! surface normal n = unit( P_u x P_v ); P_v = ruling, P_u = a' at v=0, b' at v=1
    n0 = unit3(cross(ap, ruling))
    n1 = unit3(cross(bp, ruling))
    A0 = a_pt + R * n0
    A1 = b_pt + R * n1
    alpha = unit3(A1 - A0)
    q0 = A0
  end subroutine two_point

  !> Signed deviation g = dist(point, axis) - R for npts design points.
  !> g > 0 : material left (undercut);  g < 0 : gouge.
  subroutine deviation(q0, alpha, R, pts, npts, g)
    integer, intent(in)  :: npts
    real(dp), intent(in) :: q0(3), alpha(3), R, pts(3, npts)
    real(dp), intent(out) :: g(npts)
    integer :: i
    real(dp) :: ahat(3), w(3), perp(3), proj

    ahat = unit3(alpha)
    do i = 1, npts
      w = pts(:, i) - q0
      proj = dot3(w, ahat)
      perp = w - proj * ahat
      g(i) = norm3(perp) - R
    end do
  end subroutine deviation

  !> Convenience: max |g| along a single ruling sampled at nv stations,
  !> for a given cutter axis. Returns the peak absolute deviation.
  function max_dev_ruling(a_pt, b_pt, q0, alpha, R, nv) result(emax)
    integer, intent(in)  :: nv
    real(dp), intent(in) :: a_pt(3), b_pt(3), q0(3), alpha(3), R
    real(dp) :: emax
    integer :: j
    real(dp) :: v, pt(3), g(1)

    emax = 0.0_dp
    do j = 1, nv
      v = real(j - 1, dp) / real(nv - 1, dp)
      pt = (1.0_dp - v) * a_pt + v * b_pt
      call deviation(q0, alpha, R, pt, 1, g)
      emax = max(emax, abs(g(1)))
    end do
  end function max_dev_ruling

end module flank_mod
