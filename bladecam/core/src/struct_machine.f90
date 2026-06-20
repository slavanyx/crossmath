!> Structural machine-model collision: the tool-side kinematic branch (the
!> rotating tool assembly) against the part-side / frame structural links
!> (trunnion cradle yoke, machine column) of a 5-axis machine.
!>
!> Each rigid link is modelled as a set of CAPSULES (round-capped cylinders):
!> a capsule is a line segment [p0,p1] inflated by radius r. Capsule-capsule
!> clearance is exact -- the distance between the two core segments minus the
!> two radii -- via the standard clamped closest-point-between-segments
!> computation (Ericson, Real-Time Collision Detection, sec. 5.1.9).
!>
!> A capsule is a CONSERVATIVE outer bound of the flat-capped cylinder used by
!> the per-tool model (it adds a hemisphere of radius r past each flat end), so
!> the reported clearance is never larger than the true solid-solid clearance --
!> a safe bias for collision avoidance.
!>
!> Frames: the caller supplies BOTH capsule sets already expressed in a common
!> frame, per station, so this module is purely geometric and kinematics-
!> convention agnostic. (bladecam places everything in the PART frame, with the
!> tool from q0/alpha and the structure transformed by the kinematics map.)
module struct_machine_mod
  use vec3_mod
  implicit none
  private
  public :: seg_seg_dist, capsule_clearance, struct_clearance

contains

  !> Closest distance between segment [p1,q1] and segment [p2,q2].
  !> Clamped parametric solution; robust for degenerate (point/parallel) cases.
  pure function seg_seg_dist(p1, q1, p2, q2) result(dist)
    real(dp), intent(in) :: p1(3), q1(3), p2(3), q2(3)
    real(dp) :: dist
    real(dp) :: d1(3), d2(3), r(3), aa, e, f, c, b, denom, s, t
    real(dp) :: c1(3), c2(3)
    real(dp), parameter :: eps = 1.0e-14_dp
    d1 = q1 - p1            ! direction & length of segment 1
    d2 = q2 - p2            ! direction & length of segment 2
    r  = p1 - p2
    aa = dot3(d1, d1)       ! squared length of segment 1, always >= 0
    e  = dot3(d2, d2)       ! squared length of segment 2, always >= 0
    f  = dot3(d2, r)
    if (aa <= eps .and. e <= eps) then
      s = 0.0_dp; t = 0.0_dp                 ! both segments are points
    else if (aa <= eps) then
      s = 0.0_dp                             ! segment 1 is a point
      t = clamp01(f / e)
    else
      c = dot3(d1, r)
      if (e <= eps) then
        t = 0.0_dp                           ! segment 2 is a point
        s = clamp01(-c / aa)
      else
        b = dot3(d1, d2)
        denom = aa*e - b*b                    ! >= 0 (Cauchy-Schwarz)
        if (denom > eps) then
          s = clamp01((b*f - c*e) / denom)
        else
          s = 0.0_dp                         ! parallel: pick an endpoint on seg1
        end if
        t = (b*s + f) / e
        if (t < 0.0_dp) then
          t = 0.0_dp; s = clamp01(-c / aa)
        else if (t > 1.0_dp) then
          t = 1.0_dp; s = clamp01((b - c) / aa)
        end if
      end if
    end if
    c1 = p1 + s*d1
    c2 = p2 + t*d2
    dist = norm3(c1 - c2)
  end function seg_seg_dist

  pure function clamp01(x) result(y)
    real(dp), intent(in) :: x
    real(dp) :: y
    y = min(1.0_dp, max(0.0_dp, x))
  end function clamp01

  !> Signed clearance between two capsules ca, cb (each = [p0(3), p1(3), r]).
  !> Negative = interpenetration.
  pure function capsule_clearance(ca, cb) result(clr)
    real(dp), intent(in) :: ca(7), cb(7)
    real(dp) :: clr
    clr = seg_seg_dist(ca(1:3), ca(4:6), cb(1:3), cb(4:6)) - ca(7) - cb(7)
  end function capsule_clearance

  pure function lerp_cap(c0, c1, t) result(c)
    real(dp), intent(in) :: c0(7), c1(7), t
    real(dp) :: c(7)
    c = (1.0_dp - t)*c0 + t*c1
  end function lerp_cap

  !> Per-station minimum clearance between a tool-side capsule set acaps and a
  !> structure-side capsule set bcaps, swept over each motion segment [i,i+1].
  !> acaps(7,na,nu), bcaps(7,nb,nu): capsule k at station i is acaps(:,k,i).
  !> Both move station-to-station; endpoints/radius are linearly interpolated
  !> over the segment (exact for translation, first-order for rotation -- the
  !> same swept assumption used throughout the collision core). The worst (min)
  !> clearance over the segment is found by a coarse scan plus a golden-section
  !> refine of the worst capsule pair. clr(i) covers segment [i,i+1]; clr(nu) is
  !> the static clearance at the final station. <0 = collision.
  subroutine struct_clearance(acaps, na, bcaps, nb, nu, nscan, clr)
    integer,  intent(in)  :: na, nb, nu, nscan
    real(dp), intent(in)  :: acaps(7,na,nu), bcaps(7,nb,nu)
    real(dp), intent(out) :: clr(nu)
    integer  :: i, ia, ib, sstep, ns, iabest, ibbest
    real(dp) :: t, f, fbest, tbest, tlo, thi, fref

    ns = max(1, nscan)
    do i = 1, nu - 1
      fbest = huge(1.0_dp); tbest = 0.0_dp; iabest = 1; ibbest = 1
      do ia = 1, na
        do ib = 1, nb
          do sstep = 0, ns
            t = real(sstep, dp) / real(ns, dp)
            f = capsule_clearance(lerp_cap(acaps(:,ia,i), acaps(:,ia,i+1), t), &
                                  lerp_cap(bcaps(:,ib,i), bcaps(:,ib,i+1), t))
            if (f < fbest) then
              fbest = f; tbest = t; iabest = ia; ibbest = ib
            end if
          end do
        end do
      end do
      ! golden-section refine of the worst pair around its scan minimum
      tlo = max(0.0_dp, tbest - 1.0_dp/real(ns, dp))
      thi = min(1.0_dp, tbest + 1.0_dp/real(ns, dp))
      call golden_pair(acaps(:,iabest,i), acaps(:,iabest,i+1), &
                       bcaps(:,ibbest,i), bcaps(:,ibbest,i+1), tlo, thi, fref)
      clr(i) = min(fbest, fref)
    end do
    ! final station: static
    clr(nu) = huge(1.0_dp)
    do ia = 1, na
      do ib = 1, nb
        f = capsule_clearance(acaps(:,ia,nu), bcaps(:,ib,nu))
        if (f < clr(nu)) clr(nu) = f
      end do
    end do
  end subroutine struct_clearance

  pure function pair_clr_t(a0, a1, b0, b1, t) result(f)
    real(dp), intent(in) :: a0(7), a1(7), b0(7), b1(7), t
    real(dp) :: f
    f = capsule_clearance(lerp_cap(a0, a1, t), lerp_cap(b0, b1, t))
  end function pair_clr_t

  subroutine golden_pair(a0, a1, b0, b1, tlo, thi, fmin)
    real(dp), intent(in)  :: a0(7), a1(7), b0(7), b1(7), tlo, thi
    real(dp), intent(out) :: fmin
    real(dp), parameter :: gr = 0.6180339887498949_dp
    real(dp) :: a, b, c, d, fc, fd
    integer  :: it
    a = tlo; b = thi
    c = b - gr*(b - a); d = a + gr*(b - a)
    fc = pair_clr_t(a0, a1, b0, b1, c)
    fd = pair_clr_t(a0, a1, b0, b1, d)
    do it = 1, 30
      if (fc < fd) then
        b = d; d = c; fd = fc; c = b - gr*(b - a)
        fc = pair_clr_t(a0, a1, b0, b1, c)
      else
        a = c; c = d; fc = fd; d = a + gr*(b - a)
        fd = pair_clr_t(a0, a1, b0, b1, d)
      end if
    end do
    fmin = min(fc, fd)
  end subroutine golden_pair

end module struct_machine_mod
