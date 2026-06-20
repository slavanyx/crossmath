!> Collision of the tool assembly against an arbitrary TRIANGLE MESH (imported
!> machine body, fixture, clamps or stock) -- the mesh-accurate check that
!> complements the capsule-link structural model.
!>
!> The tool assembly is a set of capsules (round-capped cylinders); each capsule
!> vs a triangle reduces to the exact segment-triangle distance minus the capsule
!> radius. Segment-triangle distance = 0 if the segment pierces the triangle,
!> else the minimum of the two endpoints' point-triangle distances and the three
!> edge-edge (segment vs triangle-edge) distances (Ericson, Real-Time Collision
!> Detection). A capsule conservatively bounds the flat-capped tool cylinder, so
!> the reported clearance is never optimistic.
module mesh_collide_mod
  use vec3_mod
  use struct_machine_mod, only: seg_seg_dist
  implicit none
  private
  public :: pt_tri_dist, seg_tri_dist, mesh_clearance

contains

  !> Closest point on triangle (a,b,c) to point p (Ericson ClosestPtPointTriangle).
  pure function closest_pt_tri(p, a, b, c) result(q)
    real(dp), intent(in) :: p(3), a(3), b(3), c(3)
    real(dp) :: q(3), ab(3), ac(3), ap(3), bp(3), cp(3)
    real(dp) :: d1, d2, d3, d4, d5, d6, va, vb, vc, denom, v, w
    ab = b - a; ac = c - a; ap = p - a
    d1 = dot3(ab, ap); d2 = dot3(ac, ap)
    if (d1 <= 0.0_dp .and. d2 <= 0.0_dp) then; q = a; return; end if
    bp = p - b; d3 = dot3(ab, bp); d4 = dot3(ac, bp)
    if (d3 >= 0.0_dp .and. d4 <= d3) then; q = b; return; end if
    vc = d1*d4 - d3*d2
    if (vc <= 0.0_dp .and. d1 >= 0.0_dp .and. d3 <= 0.0_dp) then
      v = d1 / (d1 - d3); q = a + v*ab; return
    end if
    cp = p - c; d5 = dot3(ab, cp); d6 = dot3(ac, cp)
    if (d6 >= 0.0_dp .and. d5 <= d6) then; q = c; return; end if
    vb = d5*d2 - d1*d6
    if (vb <= 0.0_dp .and. d2 >= 0.0_dp .and. d6 <= 0.0_dp) then
      w = d2 / (d2 - d6); q = a + w*ac; return
    end if
    va = d3*d6 - d5*d4
    if (va <= 0.0_dp .and. (d4 - d3) >= 0.0_dp .and. (d5 - d6) >= 0.0_dp) then
      w = (d4 - d3) / ((d4 - d3) + (d5 - d6)); q = b + w*(c - b); return
    end if
    denom = 1.0_dp / (va + vb + vc)
    v = vb*denom; w = vc*denom
    q = a + ab*v + ac*w
  end function closest_pt_tri

  pure function pt_tri_dist(p, a, b, c) result(d)
    real(dp), intent(in) :: p(3), a(3), b(3), c(3)
    real(dp) :: d
    d = norm3(p - closest_pt_tri(p, a, b, c))
  end function pt_tri_dist

  !> Does segment [p0,p1] pierce triangle (a,b,c)?
  pure function seg_tri_pierce(p0, p1, a, b, c) result(hit)
    real(dp), intent(in) :: p0(3), p1(3), a(3), b(3), c(3)
    logical :: hit
    real(dp) :: n(3), s0, s1, t, P(3)
    real(dp), parameter :: eps = 1.0e-12_dp
    hit = .false.
    n = cross(b - a, c - a)
    s0 = dot3(n, p0 - a); s1 = dot3(n, p1 - a)
    if (s0*s1 > 0.0_dp) return            ! both endpoints on the same side
    if (abs(s1 - s0) < eps) return        ! parallel/coplanar: distance handles it
    t = s0 / (s0 - s1)
    if (t < 0.0_dp .or. t > 1.0_dp) return
    P = p0 + t*(p1 - p0)                   ! plane-crossing point
    ! inside the triangle iff its own closest point on the triangle is itself
    if (norm3(P - closest_pt_tri(P, a, b, c)) < 1.0e-7_dp*(1.0_dp + norm3(P))) &
      hit = .true.
  end function seg_tri_pierce

  !> Is point p inside the closed triangle mesh? Parity of a ray cast in a
  !> slightly off-axis direction (avoids degenerate edge/vertex hits): an odd
  !> number of forward crossings means inside. tri(9,ntri) = (a,b,c) per triangle.
  pure function point_in_mesh(p, tri, ntri) result(inside)
    integer,  intent(in) :: ntri
    real(dp), intent(in) :: p(3), tri(9,ntri)
    logical :: inside
    real(dp) :: dir(3), p1(3)
    integer  :: it, cnt
    dir = [1.0_dp, 0.0001_dp, 0.00007_dp]
    p1 = p + 1.0e7_dp*dir
    cnt = 0
    do it = 1, ntri
      if (seg_tri_pierce(p, p1, tri(1:3,it), tri(4:6,it), tri(7:9,it))) &
        cnt = cnt + 1
    end do
    inside = mod(cnt, 2) == 1
  end function point_in_mesh

  !> Signed clearance of a capsule ([p0,p1], radius r) to the mesh: the unsigned
  !> segment-to-surface distance minus r, made NEGATIVE when the capsule lies
  !> inside the solid (so a tool buried in a fixture body is a collision even if
  !> it is not touching a face).
  pure function cap_mesh_clr(p0, p1, r, tri, ntri) result(clr)
    integer,  intent(in) :: ntri
    real(dp), intent(in) :: p0(3), p1(3), r, tri(9,ntri)
    real(dp) :: clr, d
    integer  :: it
    d = huge(1.0_dp)
    do it = 1, ntri
      d = min(d, seg_tri_dist(p0, p1, tri(1:3,it), tri(4:6,it), tri(7:9,it)))
    end do
    if (point_in_mesh(p0, tri, ntri) .or. point_in_mesh(p1, tri, ntri)) d = -d
    clr = d - r
  end function cap_mesh_clr

  !> Exact distance between segment [p0,p1] and triangle (a,b,c).
  pure function seg_tri_dist(p0, p1, a, b, c) result(d)
    real(dp), intent(in) :: p0(3), p1(3), a(3), b(3), c(3)
    real(dp) :: d
    if (seg_tri_pierce(p0, p1, a, b, c)) then
      d = 0.0_dp; return
    end if
    d = min(pt_tri_dist(p0, a, b, c), pt_tri_dist(p1, a, b, c))
    d = min(d, seg_seg_dist(p0, p1, a, b))
    d = min(d, seg_seg_dist(p0, p1, b, c))
    d = min(d, seg_seg_dist(p0, p1, c, a))
  end function seg_tri_dist

  !> Per-station minimum clearance between a tool-side capsule set acaps and a
  !> static triangle mesh, swept over each motion segment [i,i+1]. acaps(7,na,nu)
  !> capsules ([p0(3),p1(3),r]); tri(9,ntri) triangles (a(3),b(3),c(3)). Capsule
  !> endpoints are linearly interpolated over the segment (nscan samples). clr(i)
  !> covers segment [i,i+1]; clr(nu) is the static clearance. <0 = collision.
  subroutine mesh_clearance(acaps, na, tri, ntri, nu, nscan, clr)
    integer,  intent(in)  :: na, ntri, nu, nscan
    real(dp), intent(in)  :: acaps(7,na,nu), tri(9,ntri)
    real(dp), intent(out) :: clr(nu)
    integer  :: i, ia, sstep, ns
    real(dp) :: t, c0(7), f

    ns = max(1, nscan)
    do i = 1, nu - 1
      clr(i) = huge(1.0_dp)
      do ia = 1, na
        do sstep = 0, ns
          t = real(sstep, dp) / real(ns, dp)
          c0 = (1.0_dp - t)*acaps(:,ia,i) + t*acaps(:,ia,i+1)
          f = cap_mesh_clr(c0(1:3), c0(4:6), c0(7), tri, ntri)
          if (f < clr(i)) clr(i) = f
        end do
      end do
    end do
    ! final station: static
    clr(nu) = huge(1.0_dp)
    do ia = 1, na
      f = cap_mesh_clr(acaps(1:3,ia,nu), acaps(4:6,ia,nu), acaps(7,ia,nu), tri, ntri)
      if (f < clr(nu)) clr(nu) = f
    end do
  end subroutine mesh_clearance

end module mesh_collide_mod
