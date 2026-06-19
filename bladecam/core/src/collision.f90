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
  public :: tool_clearance, capped_cyl_sdf, swept_clearance, holder_clearance
  public :: assembly_clearance

contains

  !> Signed distance from a point to the tool solid (flute + holder), given the
  !> axis reference q0 and unit direction ahat. Negative = penetration.
  pure function tool_point_sdf(q0, ahat, R, Lf, Rh, gap, Lh, p) result(sdf)
    real(dp), intent(in) :: q0(3), ahat(3), R, Lf, Rh, gap, Lh, p(3)
    real(dp) :: sdf, w(3), lam, perp
    w = p - q0
    lam = dot3(w, ahat)
    perp = norm3(w - lam*ahat)
    sdf = min(capped_cyl_sdf(lam,            perp, Lf, R), &
              capped_cyl_sdf(lam - (Lf+gap), perp, Lh, Rh))
  end function tool_point_sdf

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
    real(dp) :: ahat(3), sdf

    do i = 1, nu
      ahat = unit3(alpha(:,i))
      clr(i) = huge(1.0_dp)
      do j = 1, npts
        sdf = tool_point_sdf(q0(:,i), ahat, R, Lf, Rh, gap, Lh, pts(:,j))
        if (sdf < clr(i)) clr(i) = sdf
      end do
    end do
  end subroutine tool_clearance

  !> Per-station clearance of the HOLDER ALONE (the shank/holder capped cylinder
  !> at axial [Lf+gap, Lf+gap+Lh], radius Rh) to an obstacle cloud. This is the
  !> check that matters against the blade BEING machined: the flute is tangent to
  !> that blade by design (so a full-tool check there is a false positive), but
  !> the holder must still clear it -- which it may not at a steep lead/lean tilt
  !> or when the flute is shorter than the ruling. clr(nu); negative = collision.
  subroutine holder_clearance(q0, alpha, Rh, base, Lh, nu, pts, npts, clr)
    integer,  intent(in)  :: nu, npts
    real(dp), intent(in)  :: q0(3,nu), alpha(3,nu), Rh, base, Lh, pts(3,npts)
    real(dp), intent(out) :: clr(nu)
    integer  :: i, j
    real(dp) :: ahat(3), w(3), lam, perp, sdf
    do i = 1, nu
      ahat = unit3(alpha(:,i))
      clr(i) = huge(1.0_dp)
      do j = 1, npts
        w = pts(:,j) - q0(:,i)
        lam = dot3(w, ahat)
        perp = norm3(w - lam*ahat)
        sdf = capped_cyl_sdf(lam - base, perp, Lh, Rh)
        if (sdf < clr(i)) clr(i) = sdf
      end do
    end do
  end subroutine holder_clearance

  !> SDF from a point to a stacked tool ASSEMBLY: nseg coaxial capped cylinders,
  !> segment s spanning axial [segLo(s), segHi(s)] with radius segR(s). Models the
  !> whole rotating stack (flute + holder + spindle nose) as one solid.
  pure function assembly_sdf(q0, ahat, nseg, segR, segLo, segHi, p) result(sdf)
    integer,  intent(in) :: nseg
    real(dp), intent(in) :: q0(3), ahat(3), segR(nseg), segLo(nseg), segHi(nseg), p(3)
    real(dp) :: sdf, w(3), lam, perp, d
    integer  :: s
    w = p - q0
    lam = dot3(w, ahat)
    perp = norm3(w - lam*ahat)
    sdf = huge(1.0_dp)
    do s = 1, nseg
      d = capped_cyl_sdf(lam - segLo(s), perp, segHi(s) - segLo(s), segR(s))
      if (d < sdf) sdf = d
    end do
  end function assembly_sdf

  !> Signed distance from the assembly to a fixture HALF-SPACE (forbidden where
  !> n.(x-p0) < 0; n unit). Exact for capped cylinders: the nearest point is a
  !> cap rim in the -n direction. Negative = the stack dips into the fixture.
  pure function assembly_plane_clr(q0, ahat, nseg, segR, segLo, segHi, p0, n) result(clr)
    integer,  intent(in) :: nseg
    real(dp), intent(in) :: q0(3), ahat(3), segR(nseg), segLo(nseg), segHi(nseg)
    real(dp), intent(in) :: p0(3), n(3)
    real(dp) :: clr, np, clo, chi, nperp
    integer  :: s
    np = norm3(n - dot3(n, ahat)*ahat)        ! |component of n perpendicular to axis|
    nperp = np
    clr = huge(1.0_dp)
    do s = 1, nseg
      clo = dot3(q0 + segLo(s)*ahat - p0, n) - segR(s)*nperp
      chi = dot3(q0 + segHi(s)*ahat - p0, n) - segR(s)*nperp
      clr = min(clr, clo, chi)
    end do
  end function assembly_plane_clr

  !> Continuous swept-volume clearance of the full tool ASSEMBLY (flute+holder+
  !> spindle, nseg capped-cylinder segments) over each motion segment [i,i+1],
  !> against an obstacle cloud, PLUS a fixture half-space (n.(x-p0)>=0 allowed;
  !> set use_plane=0 to skip). The minimum over the swept motion is found by a
  !> coarse scan + golden-section refine, as in swept_clearance. clr(i) covers
  !> segment [i,i+1]; clr(nu) the final static pose. <0 = collision.
  subroutine assembly_clearance(q0, alpha, nseg, segR, segLo, segHi, nu, &
                                pts, npts, p0, n, use_plane, nscan, clr)
    integer,  intent(in)  :: nseg, nu, npts, use_plane, nscan
    real(dp), intent(in)  :: q0(3,nu), alpha(3,nu), segR(nseg), segLo(nseg), segHi(nseg)
    real(dp), intent(in)  :: pts(3,npts), p0(3), n(3)
    real(dp), intent(out) :: clr(nu)
    integer  :: i, j, s, ns
    real(dp) :: ah0(3), ah1(3), tbest, fbest, t, f, tlo, thi, ahn(3)

    ns = max(1, nscan)
    do i = 1, nu - 1
      ah0 = unit3(alpha(:,i)); ah1 = unit3(alpha(:,i+1))
      clr(i) = huge(1.0_dp)
      do j = 1, npts
        tbest = 0.0_dp; fbest = huge(1.0_dp)
        do s = 0, ns
          t = real(s, dp) / real(ns, dp)
          f = aseg_sdf(q0(:,i), q0(:,i+1), ah0, ah1, t, nseg, segR, segLo, segHi, pts(:,j))
          if (f < fbest) then; fbest = f; tbest = t; end if
        end do
        tlo = max(0.0_dp, tbest - 1.0_dp/real(ns, dp))
        thi = min(1.0_dp, tbest + 1.0_dp/real(ns, dp))
        call agolden(q0(:,i), q0(:,i+1), ah0, ah1, nseg, segR, segLo, segHi, &
                     pts(:,j), tlo, thi, f)
        fbest = min(fbest, f)
        if (fbest < clr(i)) clr(i) = fbest
      end do
      if (use_plane /= 0) then
        clr(i) = min(clr(i), &
          assembly_plane_clr(q0(:,i), ah0, nseg, segR, segLo, segHi, p0, n))
      end if
    end do
    ! final station: static
    ahn = unit3(alpha(:,nu))
    clr(nu) = huge(1.0_dp)
    do j = 1, npts
      f = assembly_sdf(q0(:,nu), ahn, nseg, segR, segLo, segHi, pts(:,j))
      if (f < clr(nu)) clr(nu) = f
    end do
    if (use_plane /= 0) clr(nu) = min(clr(nu), &
      assembly_plane_clr(q0(:,nu), ahn, nseg, segR, segLo, segHi, p0, n))
  end subroutine assembly_clearance

  pure function aseg_sdf(qa, qb, aha, ahb, t, nseg, segR, segLo, segHi, p) result(sdf)
    integer,  intent(in) :: nseg
    real(dp), intent(in) :: qa(3), qb(3), aha(3), ahb(3), t
    real(dp), intent(in) :: segR(nseg), segLo(nseg), segHi(nseg), p(3)
    real(dp) :: sdf, q0(3), ah(3)
    q0 = (1.0_dp - t)*qa + t*qb
    ah = unit3((1.0_dp - t)*aha + t*ahb)
    sdf = assembly_sdf(q0, ah, nseg, segR, segLo, segHi, p)
  end function aseg_sdf

  subroutine agolden(qa, qb, aha, ahb, nseg, segR, segLo, segHi, p, tlo, thi, fmin)
    integer,  intent(in)  :: nseg
    real(dp), intent(in)  :: qa(3), qb(3), aha(3), ahb(3)
    real(dp), intent(in)  :: segR(nseg), segLo(nseg), segHi(nseg), p(3), tlo, thi
    real(dp), intent(out) :: fmin
    real(dp), parameter :: gr = 0.6180339887498949_dp
    real(dp) :: a, b, c, d, fc, fd
    integer  :: it
    a = tlo; b = thi
    c = b - gr*(b - a); d = a + gr*(b - a)
    fc = aseg_sdf(qa, qb, aha, ahb, c, nseg, segR, segLo, segHi, p)
    fd = aseg_sdf(qa, qb, aha, ahb, d, nseg, segR, segLo, segHi, p)
    do it = 1, 30
      if (fc < fd) then
        b = d; d = c; fd = fc; c = b - gr*(b - a)
        fc = aseg_sdf(qa, qb, aha, ahb, c, nseg, segR, segLo, segHi, p)
      else
        a = c; c = d; fc = fd; d = a + gr*(b - a)
        fd = aseg_sdf(qa, qb, aha, ahb, d, nseg, segR, segLo, segHi, p)
      end if
    end do
    fmin = min(fc, fd)
  end subroutine agolden

  !> Continuous swept-volume clearance: as the tool sweeps from station i to i+1
  !> (q0 lerped, axis normalised-lerped) the minimum clearance to the obstacle
  !> cloud is found over the WHOLE motion, not just the endpoints. For each
  !> obstacle point and segment the tool SDF is minimised over t in [0,1] by a
  !> coarse scan plus golden-section refinement (the SDF is smooth and locally
  !> unimodal over one short segment). clr(i) covers segment [i,i+1]; clr(nu) is
  !> the static clearance at the final station.
  subroutine swept_clearance(q0, alpha, R, Lf, Rh, gap, Lh, nu, pts, npts, &
                             nscan, clr)
    integer,  intent(in)  :: nu, npts, nscan
    real(dp), intent(in)  :: q0(3,nu), alpha(3,nu), R, Lf, Rh, gap, Lh
    real(dp), intent(in)  :: pts(3,npts)
    real(dp), intent(out) :: clr(nu)
    integer  :: i, j, s, ns
    real(dp) :: ah0(3), ah1(3), tbest, fbest, t, f, tlo, thi

    ns = max(1, nscan)        ! guard against a zero scan count (div-by-zero)
    do i = 1, nu - 1
      ah0 = unit3(alpha(:,i)); ah1 = unit3(alpha(:,i+1))
      clr(i) = huge(1.0_dp)
      do j = 1, npts
        ! coarse scan for the worst (min-SDF) time over the segment
        tbest = 0.0_dp; fbest = huge(1.0_dp)
        do s = 0, ns
          t = real(s, dp) / real(ns, dp)
          f = seg_sdf(q0(:,i), q0(:,i+1), ah0, ah1, t, R, Lf, Rh, gap, Lh, pts(:,j))
          if (f < fbest) then; fbest = f; tbest = t; end if
        end do
        ! golden-section refine in the bracketing interval around tbest
        tlo = max(0.0_dp, tbest - 1.0_dp/real(ns, dp))
        thi = min(1.0_dp, tbest + 1.0_dp/real(ns, dp))
        call golden_min(q0(:,i), q0(:,i+1), ah0, ah1, R, Lf, Rh, gap, Lh, &
                        pts(:,j), tlo, thi, f)
        fbest = min(fbest, f)
        if (fbest < clr(i)) clr(i) = fbest
      end do
    end do
    ! last station: static clearance
    ah0 = unit3(alpha(:,nu))
    clr(nu) = huge(1.0_dp)
    do j = 1, npts
      f = tool_point_sdf(q0(:,nu), ah0, R, Lf, Rh, gap, Lh, pts(:,j))
      if (f < clr(nu)) clr(nu) = f
    end do
  end subroutine swept_clearance

  !> Tool SDF to point p at interpolation parameter t along a segment.
  pure function seg_sdf(qa, qb, aha, ahb, t, R, Lf, Rh, gap, Lh, p) result(sdf)
    real(dp), intent(in) :: qa(3), qb(3), aha(3), ahb(3), t, R, Lf, Rh, gap, Lh, p(3)
    real(dp) :: sdf, q0(3), ah(3)
    q0 = (1.0_dp - t)*qa + t*qb
    ah = unit3((1.0_dp - t)*aha + t*ahb)
    sdf = tool_point_sdf(q0, ah, R, Lf, Rh, gap, Lh, p)
  end function seg_sdf

  !> Golden-section minimisation of seg_sdf over t in [tlo, thi]; returns fmin.
  subroutine golden_min(qa, qb, aha, ahb, R, Lf, Rh, gap, Lh, p, tlo, thi, fmin)
    real(dp), intent(in)  :: qa(3), qb(3), aha(3), ahb(3), R, Lf, Rh, gap, Lh, p(3)
    real(dp), intent(in)  :: tlo, thi
    real(dp), intent(out) :: fmin
    real(dp), parameter :: gr = 0.6180339887498949_dp
    real(dp) :: a, b, c, d, fc, fd
    integer  :: it
    a = tlo; b = thi
    c = b - gr*(b - a); d = a + gr*(b - a)
    fc = seg_sdf(qa, qb, aha, ahb, c, R, Lf, Rh, gap, Lh, p)
    fd = seg_sdf(qa, qb, aha, ahb, d, R, Lf, Rh, gap, Lh, p)
    do it = 1, 30
      if (fc < fd) then
        b = d; d = c; fd = fc
        c = b - gr*(b - a)
        fc = seg_sdf(qa, qb, aha, ahb, c, R, Lf, Rh, gap, Lh, p)
      else
        a = c; c = d; fc = fd
        d = a + gr*(b - a)
        fd = seg_sdf(qa, qb, aha, ahb, d, R, Lf, Rh, gap, Lh, p)
      end if
    end do
    fmin = min(fc, fd)
  end subroutine golden_min

end module collision_mod
