!> Time-optimal path parameterization (TOPP) for a fixed joint-space path.
!>
!> Given q(s) sampled at n uniform stations on s in [0,1] for ndof axes, with
!> per-axis velocity (vmax) and acceleration (amax) limits, find the squared
!> path speed a(s)=sdot^2 that minimises traversal time T = int ds/sqrt(a).
!>
!> Method: forward/backward integration of the acceleration-limited dynamics
!> under the velocity-limit curve (a standard, robust TOPP scheme).
!>   q'  = q_s,   q'' = q_ss   (finite differences in s)
!>   velocity: a <= min_i (vmax_i/|q'_i|)^2
!>   accel:    |q''_i a + q'_i sdd| <= amax_i,   da/ds = 2 sdd
module topp_mod
  use vec3_mod, only: dp
  implicit none
  private
  public :: topp_ra

contains

  subroutine topp_ra(q, ndof, n, vmax, amax, a0, aN, aprof, ttotal)
    integer,  intent(in)  :: ndof, n
    real(dp), intent(in)  :: q(ndof, n), vmax(ndof), amax(ndof), a0, aN
    real(dp), intent(out) :: aprof(n), ttotal
    real(dp) :: qp(ndof, n), qpp(ndof, n), abar(n)
    real(dp) :: qpm(ndof), qppm(ndof)
    real(dp) :: ds, denom, anew, amaxchg, cap
    integer  :: k, i, it

    ! degenerate paths: nothing to parameterise (n=1) or a single segment with no
    ! interior curvature (n=2). Guard before the 3-point stencils, which would
    ! otherwise read q(:,3)/q(:,n-2) out of bounds for n<3.
    if (n < 2) then
      aprof = 0.0_dp; ttotal = 0.0_dp
      return
    end if

    ds = 1.0_dp / real(n - 1, dp)

    ! finite-difference path derivatives in s (second-order curvature needs n>=3;
    ! for n=2 the segment is straight so qpp=0)
    do k = 1, n
      if (k == 1) then
        qp(:, k)  = (q(:, 2) - q(:, 1)) / ds
        if (n >= 3) then
          qpp(:, k) = (q(:, 3) - 2.0_dp*q(:, 2) + q(:, 1)) / ds**2
        else
          qpp(:, k) = 0.0_dp
        end if
      else if (k == n) then
        qp(:, k)  = (q(:, n) - q(:, n-1)) / ds
        if (n >= 3) then
          qpp(:, k) = (q(:, n) - 2.0_dp*q(:, n-1) + q(:, n-2)) / ds**2
        else
          qpp(:, k) = 0.0_dp
        end if
      else
        qp(:, k)  = (q(:, k+1) - q(:, k-1)) / (2.0_dp*ds)
        qpp(:, k) = (q(:, k+1) - 2.0_dp*q(:, k) + q(:, k-1)) / ds**2
      end if
    end do

    ! velocity-limit curve (kinematic part of the MVC): a = sdot^2 capped per axis
    ! so the REALISED axis speed stays within vmax. The motion the machine runs
    ! between two stations is a straight joint-space segment, so the speed it
    ! actually realises on [k,k+1] is |q_{k+1}-q_k|/ds * sdot_avg -- the SEGMENT
    ! FORWARD-difference slope, not the central-difference slope at a station
    ! (which corresponds to no realised motion). Bounding the central slope is a
    ! discretisation shortcut that under-constrains a curvature kink, letting the
    ! posted rotary speed exceed vmax between stations. Cap each segment by its
    ! own forward slope: cseg = min_i (vmax_i/|q_seg,i|)^2, and apply it to BOTH
    ! endpoint stations. Then sdot_k, sdot_{k+1} <= sqrt(cseg), so
    !     realised speed = |q_seg|*sdot_avg <= |q_seg|*sqrt(cseg) <= vmax,
    ! i.e. the bound holds on the exact quantity the post-processor verifies.
    abar = huge(1.0_dp)
    do k = 1, n - 1
      cap = huge(1.0_dp)
      do i = 1, ndof
        denom = abs(q(i, k+1) - q(i, k)) / ds
        if (denom > 1.0e-12_dp) cap = min(cap, (vmax(i) / denom)**2)
      end do
      abar(k)   = min(abar(k),   cap)
      abar(k+1) = min(abar(k+1), cap)
    end do

    ! Feasible profile by ITERATED forward/backward clamping. The acceleration
    ! constraint is enforced on the SEGMENT MIDPOINT -- the quantity that is
    ! actually realised between two stations -- not at the stations. The realised
    ! joint acceleration on segment [k,k+1] is
    !     acc_i = q''_mid_i * (a_k+a_{k+1})/2  +  q'_mid_i * (a_{k+1}-a_k)/(2 ds)
    !           = P_i a_{k+1} + Q_i a_k,
    ! with P_i = q''_mid_i/2 + q'_mid_i/(2 ds),  Q_i = q''_mid_i/2 - q'_mid_i/(2 ds)
    ! and q'_mid, q''_mid the segment-midpoint derivatives. |acc_i| <= amax_i is a
    ! linear band that the forward pass solves for a_{k+1} (the rising/accel face)
    ! and the backward pass for a_k (the falling/decel face). Because the solver
    ! now bounds the SAME midpoint acceleration the trajectory realises, the
    ! discretisation overshoot at velocity cusps (where q'' peaks between stations)
    ! is removed. Each clamp only LOWERS aprof, so the iteration is monotone,
    ! bounded below by 0, hence convergent to a feasible profile.
    aprof = abar
    aprof(1) = min(aprof(1), a0)
    aprof(n) = min(aprof(n), aN)
    do it = 1, 200
      amaxchg = 0.0_dp
      ! forward: cap a_{k+1} so the realised acceleration on [k,k+1] is feasible
      do k = 1, n - 1
        qpm  = 0.5_dp*(qp(:, k)  + qp(:, k+1))
        qppm = 0.5_dp*(qpp(:, k) + qpp(:, k+1))
        cap  = seg_cap(qpm, qppm, amax, ndof, ds, aprof(k), .true.)
        anew = max(0.0_dp, min(aprof(k+1), cap))
        amaxchg = max(amaxchg, aprof(k+1) - anew)
        aprof(k+1) = anew
      end do
      ! backward: cap a_k so the realised (deceleration) on [k,k+1] is feasible
      do k = n - 1, 1, -1
        qpm  = 0.5_dp*(qp(:, k)  + qp(:, k+1))
        qppm = 0.5_dp*(qpp(:, k) + qpp(:, k+1))
        cap  = seg_cap(qpm, qppm, amax, ndof, ds, aprof(k+1), .false.)
        anew = max(0.0_dp, min(aprof(k), cap))
        amaxchg = max(amaxchg, aprof(k) - anew)
        aprof(k) = anew
      end do
      if (amaxchg <= 1.0e-15_dp) exit
    end do

    ! integrate time: dt = ds / sdot, trapezoid on 1/sqrt(a)
    ttotal = 0.0_dp
    do k = 1, n - 1
      denom = sqrt(aprof(k)) + sqrt(aprof(k+1))
      if (denom > 1.0e-12_dp) ttotal = ttotal + 2.0_dp*ds / denom
    end do
  end subroutine topp_ra

  !> Largest squared path speed at one end of a segment for which the realised
  !> midpoint acceleration stays within amax, given the speed^2 `aother` at the
  !> other end. forward=.true. solves for a_{k+1} given a_k=aother (rising face);
  !> forward=.false. solves for a_k given a_{k+1}=aother (falling face). Returns a
  !> large value if no axis constrains this end; the caller clamps to >=0 and to
  !> the velocity-curve cap.
  pure function seg_cap(qpm, qppm, amax, ndof, ds, aother, forward) result(cap)
    integer,  intent(in) :: ndof
    real(dp), intent(in) :: qpm(ndof), qppm(ndof), amax(ndof), ds, aother
    logical,  intent(in) :: forward
    real(dp) :: cap, P, Q, coef, other, ub
    integer  :: i
    cap = huge(1.0_dp)
    do i = 1, ndof
      P = 0.5_dp*qppm(i) + qpm(i)/(2.0_dp*ds)        ! coeff of a_{k+1}
      Q = 0.5_dp*qppm(i) - qpm(i)/(2.0_dp*ds)        ! coeff of a_k
      if (forward) then
        coef = P;  other = Q*aother                  ! solve P*x in [-amax-other, amax-other]
      else
        coef = Q;  other = P*aother
      end if
      if (abs(coef) > 1.0e-14_dp) then
        if (coef > 0.0_dp) then
          ub = ( amax(i) - other) / coef
        else
          ub = (-amax(i) - other) / coef
        end if
        cap = min(cap, ub)
      end if
    end do
  end function seg_cap

end module topp_mod
