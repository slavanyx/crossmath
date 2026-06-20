!> Dexel material-removal simulation (verified machined geometry).
!>
!> A dexel is a ray carrying the 1-D extent of remaining material along it. The
!> swept volume of the moving capped-cylinder tool is subtracted from a field of
!> rays; what remains reconstructs the actually-machined surface. Casting rays
!> along three orthogonal directions (tri-dexel) or along design-surface normals
!> gives a true volumetric removal check -- it catches leftover stock and
!> between-pass scallops that a per-design-point envelope projection can miss.
module dexel_mod
  use vec3_mod
  implicit none
  private
  public :: dexel_carve, dexel_removed_intervals

contains

  !> Ray (origin O, unit direction D) intersected with one capped cylinder
  !> (axis q0+ s*ahat, s in [0,Lf], radius R). Returns the parameter interval
  !> [tin,tout] where the ray is inside the solid; hit=.false. if it misses.
  pure subroutine ray_cyl(O, D, q0, ahat, R, Lf, tin, tout, hit)
    real(dp), intent(in)  :: O(3), D(3), q0(3), ahat(3), R, Lf
    real(dp), intent(out) :: tin, tout
    logical,  intent(out) :: hit
    real(dp) :: m(3), mp(3), dp_(3), aa, bb, cc, disc, sq, t1, t2
    real(dp) :: da, ma, ta, tb, tlo, thi
    hit = .false.; tin = 0.0_dp; tout = 0.0_dp
    m  = O - q0
    da = dot3(D, ahat); ma = dot3(m, ahat)
    dp_ = D - da*ahat                 ! component of D perpendicular to the axis
    mp  = m - ma*ahat
    aa = dot3(dp_, dp_); bb = 2.0_dp*dot3(mp, dp_); cc = dot3(mp, mp) - R*R
    ! radial (infinite-cylinder) interval [t1,t2]
    if (aa > 1.0e-14_dp) then
      disc = bb*bb - 4.0_dp*aa*cc
      if (disc < 0.0_dp) return       ! ray never within R of the axis
      sq = sqrt(disc)
      t1 = (-bb - sq) / (2.0_dp*aa); t2 = (-bb + sq) / (2.0_dp*aa)
    else
      if (cc > 0.0_dp) return         ! ray parallel to axis and outside radius
      t1 = -huge(1.0_dp); t2 = huge(1.0_dp)
    end if
    ! axial-cap interval: s(t) = ma + t*da in [0, Lf]
    if (abs(da) > 1.0e-14_dp) then
      ta = (0.0_dp - ma) / da; tb = (Lf - ma) / da
      tlo = min(ta, tb); thi = max(ta, tb)
    else
      if (ma < 0.0_dp .or. ma > Lf) return   ! outside the caps, never enters
      tlo = -huge(1.0_dp); thi = huge(1.0_dp)
    end if
    tin = max(t1, tlo); tout = min(t2, thi)
    if (tout > tin) hit = .true.
  end subroutine ray_cyl

  !> Carve the swept tool (nu capped-cylinder poses) out of a field of nray
  !> dexels. Ray r runs from its origin orig(:,r) along unit dir(:,r), with
  !> initial solid material over t in [0, seg0(r)]. Returns removed(r) = total
  !> length of [0,seg0] removed by the swept tool (union over all poses), and
  !> first_cut(r) = smallest t that is removed (the machined-surface crossing
  !> from the ray origin side), or seg0(r) if nothing is removed.
  subroutine dexel_carve(q0, alpha, R, Lf, nu, orig, dir, seg0, nray, &
                         removed, first_cut)
    integer,  intent(in)  :: nu, nray
    real(dp), intent(in)  :: q0(3,nu), alpha(3,nu), R, Lf(nu)
    real(dp), intent(in)  :: orig(3,nray), dir(3,nray), seg0(nray)
    real(dp), intent(out) :: removed(nray), first_cut(nray)
    integer  :: ir, i, k, m, nseg
    real(dp) :: ahat(3,nu), a(2,nu), tin, tout, lo, hi, acc
    logical  :: hit

    do i = 1, nu
      ahat(:,i) = unit3(alpha(:,i))
    end do

    do ir = 1, nray
      nseg = 0
      ! collect all removed sub-intervals on this ray, clamped to [0,seg0]
      do i = 1, nu
        call ray_cyl(orig(:,ir), dir(:,ir), q0(:,i), ahat(:,i), R, Lf(i), &
                     tin, tout, hit)
        if (.not. hit) cycle
        lo = max(0.0_dp, tin); hi = min(seg0(ir), tout)
        if (hi > lo) then
          nseg = nseg + 1
          a(1,nseg) = lo; a(2,nseg) = hi
        end if
      end do
      if (nseg == 0) then
        removed(ir) = 0.0_dp; first_cut(ir) = seg0(ir); cycle
      end if
      ! sort sub-intervals by start (insertion sort; nseg <= nu, small)
      do k = 2, nseg
        lo = a(1,k); hi = a(2,k); m = k - 1
        do while (m >= 1)
          if (a(1,m) <= lo) exit
          a(1,m+1) = a(1,m); a(2,m+1) = a(2,m); m = m - 1
        end do
        a(1,m+1) = lo; a(2,m+1) = hi
      end do
      ! merge overlaps and accumulate union length
      acc = 0.0_dp
      lo = a(1,1); hi = a(2,1)
      do k = 2, nseg
        if (a(1,k) <= hi) then
          if (a(2,k) > hi) hi = a(2,k)
        else
          acc = acc + (hi - lo); lo = a(1,k); hi = a(2,k)
        end if
      end do
      acc = acc + (hi - lo)
      removed(ir) = acc
      first_cut(ir) = a(1,1)       ! smallest removed t (intervals are sorted)
    end do
  end subroutine dexel_carve

  !> Merged set of removed sub-intervals along each ray for the swept tool, so a
  !> PERSISTENT interval-dexel stock can be carried across operations (roughing
  !> then finishing -> rest material). For ray r the tool's footprint is the
  !> union of [tin,tout] over all poses, clamped to t>=0, merged into disjoint
  !> ascending intervals written to rlo(:,r)/rhi(:,r) with count rn(r) (capped at
  !> maxseg). Unlike the height-field carve this captures removal from EITHER end
  !> or the middle of a ray, so a tilted finish tool and a top-down rougher are
  !> both represented correctly.
  subroutine dexel_removed_intervals(q0, alpha, R, Lf, nu, orig, dir, nray, &
                                     maxseg, rlo, rhi, rn)
    integer,  intent(in)  :: nu, nray, maxseg
    real(dp), intent(in)  :: q0(3,nu), alpha(3,nu), R, Lf(nu)
    real(dp), intent(in)  :: orig(3,nray), dir(3,nray)
    real(dp), intent(out) :: rlo(maxseg,nray), rhi(maxseg,nray)
    integer,  intent(out) :: rn(nray)
    integer  :: ir, i, k, m, nseg, nm
    real(dp) :: ahat(3,nu), a(2,nu), tin, tout, lo, hi, clo, chi
    logical  :: hit

    do i = 1, nu
      ahat(:,i) = unit3(alpha(:,i))
    end do

    do ir = 1, nray
      rn(ir) = 0
      nseg = 0
      do i = 1, nu
        call ray_cyl(orig(:,ir), dir(:,ir), q0(:,i), ahat(:,i), R, Lf(i), &
                     tin, tout, hit)
        if (.not. hit) cycle
        lo = max(0.0_dp, tin); hi = tout
        if (hi > lo) then
          nseg = nseg + 1
          a(1,nseg) = lo; a(2,nseg) = hi
        end if
      end do
      if (nseg == 0) cycle
      ! sort sub-intervals by start (insertion sort; nseg <= nu)
      do k = 2, nseg
        lo = a(1,k); hi = a(2,k); m = k - 1
        do while (m >= 1)
          if (a(1,m) <= lo) exit
          a(1,m+1) = a(1,m); a(2,m+1) = a(2,m); m = m - 1
        end do
        a(1,m+1) = lo; a(2,m+1) = hi
      end do
      ! merge overlaps into disjoint ascending intervals; store (cap at maxseg)
      nm = 0
      clo = a(1,1); chi = a(2,1)
      do k = 2, nseg
        if (a(1,k) <= chi) then
          if (a(2,k) > chi) chi = a(2,k)
        else
          if (nm < maxseg) then
            nm = nm + 1; rlo(nm,ir) = clo; rhi(nm,ir) = chi
          end if
          clo = a(1,k); chi = a(2,k)
        end if
      end do
      if (nm < maxseg) then
        nm = nm + 1; rlo(nm,ir) = clo; rhi(nm,ir) = chi
      end if
      rn(ir) = nm
    end do
  end subroutine dexel_removed_intervals

end module dexel_mod
