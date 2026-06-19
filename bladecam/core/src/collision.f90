!> Tool/holder collision and gouge checking via signed distance fields.
!>
!> The tool is modelled as two coaxial capped cylinders along the axis from the
!> reference point q0 in direction alpha:
!>   flute :  radius R,  axial [0, Lf]
!>   holder:  radius Rh, axial [Lf+gap, Lf+gap+Lh]
!> For each obstacle point we evaluate the signed distance to this solid
!> (negative = penetration). The per-station clearance is the minimum over all
!> obstacle points; a negative clearance means a collision/gouge.
module collision_mod
  use vec3_mod
  implicit none
  private
  public :: tool_clearance, capped_cyl_sdf

contains

  !> Signed distance from a point (axial a measured from the cylinder base,
  !> radial r >= 0) to a capped cylinder of height h and radius rad.
  !> Negative inside the solid. (Standard capped-cylinder SDF.)
  pure function capped_cyl_sdf(a, r, h, rad) result(sdf)
    real(dp), intent(in) :: a, r, h, rad
    real(dp) :: sdf, dx, dy, ox, oy, outside, inside
    dx = r - rad
    dy = abs(a - 0.5_dp*h) - 0.5_dp*h
    ox = max(dx, 0.0_dp); oy = max(dy, 0.0_dp)
    outside = sqrt(ox*ox + oy*oy)
    inside = min(max(dx, dy), 0.0_dp)
    sdf = outside + inside
  end function capped_cyl_sdf

  !> Per-station minimum clearance of the tool+holder to an obstacle cloud.
  !> q0,alpha are (3,nu); pts is (3,npts). Returns clr(nu) (min signed distance
  !> over all points for each station; negative = collision).
  subroutine tool_clearance(q0, alpha, R, Lf, Rh, gap, Lh, nu, pts, npts, clr)
    integer,  intent(in)  :: nu, npts
    real(dp), intent(in)  :: q0(3,nu), alpha(3,nu), R, Lf, Rh, gap, Lh
    real(dp), intent(in)  :: pts(3,npts)
    real(dp), intent(out) :: clr(nu)
    integer  :: i, j
    real(dp) :: ahat(3), w(3), lam, perp, sdf, sflute, shold

    do i = 1, nu
      ahat = unit3(alpha(:,i))
      clr(i) = huge(1.0_dp)
      do j = 1, npts
        w = pts(:,j) - q0(:,i)
        lam = dot3(w, ahat)
        perp = norm3(w - lam*ahat)
        sflute = capped_cyl_sdf(lam,             perp, Lf, R)
        shold  = capped_cyl_sdf(lam - (Lf+gap),  perp, Lh, Rh)
        sdf = min(sflute, shold)
        if (sdf < clr(i)) clr(i) = sdf
      end do
    end do
  end subroutine tool_clearance

end module collision_mod
