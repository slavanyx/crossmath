!> Flank-milling tool positioning and envelope-deviation evaluation.
module flank_mod
  use vec3_mod
  implicit none
  private
  public :: two_point, deviation, deviation_cone, max_dev_ruling, swept_deviation

contains

  !> Two-point cutter positioning (Bedi/Mann/Menzel) for one ruling.
  !> Tangency is enforced at both ruling ends by offsetting each endpoint
  !> by R along the local surface normal; the cutter axis joins the two
  !> offset points. ap = a'(u), bp = b'(u) are the rail tangents.
  subroutine two_point(a_pt, ap, b_pt, bp, R, q0, alpha)
    real(dp), intent(in)  :: a_pt(3), ap(3), b_pt(3), bp(3), R
    real(dp), intent(out) :: q0(3), alpha(3)
    real(dp) :: ruling(3), rhat(3), n0(3), n1(3), A0(3), A1(3), ref(3)
    real(dp), parameter :: tiny = 1.0e-12_dp

    ruling = b_pt - a_pt
    rhat = unit3(ruling)
    ! surface normal n = unit( P_u x P_v ); P_v = ruling, P_u = a' at v=0, b' at v=1
    n0 = cross(ap, ruling)
    n1 = cross(bp, ruling)
    ! Degenerate guard: if a rail tangent is (near) parallel to the ruling its
    ! cross product vanishes and the normal is ill-defined -- without this the
    ! offset would collapse (A=point) so the cylinder passes THROUGH the surface
    ! (a gouge of -R). Fall back to the other end's normal, or, if both are
    ! degenerate, to any direction perpendicular to the ruling.
    if (norm3(n0) < tiny .and. norm3(n1) < tiny) then
      ref = [0.0_dp, 0.0_dp, 1.0_dp]
      if (abs(dot3(rhat, ref)) > 0.9_dp) ref = [1.0_dp, 0.0_dp, 0.0_dp]
      n0 = cross(rhat, ref); n1 = n0
    else if (norm3(n0) < tiny) then
      n0 = n1
    else if (norm3(n1) < tiny) then
      n1 = n0
    end if
    n0 = unit3(n0); n1 = unit3(n1)
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

  !> Signed deviation for a CONICAL tool: local radius rho(lambda)=R+lambda*tan(g)
  !> where lambda is the axial coordinate from q0 along alpha. The radial gap is
  !> projected onto the cone surface normal (factor cos g). gamma=0 -> cylinder.
  subroutine deviation_cone(q0, alpha, R, gamma, pts, npts, g)
    integer, intent(in)  :: npts
    real(dp), intent(in) :: q0(3), alpha(3), R, gamma, pts(3, npts)
    real(dp), intent(out) :: g(npts)
    integer :: i
    real(dp) :: ahat(3), w(3), perp(3), lam, rho, tg, cg
    ahat = unit3(alpha)
    tg = tan(gamma); cg = cos(gamma)
    do i = 1, npts
      w = pts(:, i) - q0
      lam = dot3(w, ahat)
      perp = w - lam * ahat
      rho = R + lam * tg
      g(i) = (norm3(perp) - rho) * cg
    end do
  end subroutine deviation_cone

  !> Swept-envelope deviation: for each design point, the signed distance to the
  !> CLOSEST of all cutter positions (finite flute segments), minus R. Captures
  !> cross-station interference (the tool at one station gouging the surface that
  !> "belongs" to another) which the per-station deviation cannot see. g < 0 is a
  !> real overcut/gouge of the machined part.
  subroutine swept_deviation(q0, alpha, Lflute, R, nu, pts, npts, g)
    integer,  intent(in)  :: nu, npts
    real(dp), intent(in)  :: q0(3,nu), alpha(3,nu), Lflute(nu), R, pts(3,npts)
    real(dp), intent(out) :: g(npts)
    integer  :: p, i
    real(dp) :: ahat(3,nu), w(3), lam, d
    do i = 1, nu
      ahat(:,i) = unit3(alpha(:,i))
    end do
    do p = 1, npts
      g(p) = huge(1.0_dp)
      do i = 1, nu
        w = pts(:,p) - q0(:,i)
        lam = dot3(w, ahat(:,i))
        lam = max(0.0_dp, min(Lflute(i), lam))     ! finite engaged flute
        d = norm3(w - lam*ahat(:,i)) - R
        if (d < g(p)) g(p) = d
      end do
    end do
  end subroutine swept_deviation

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
