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
    real(dp) :: ds, sdd_lo, sdd_hi, denom, anew, amaxchg
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

    ! velocity-limit curve (a.k.a. maximum-velocity curve / MVC).
    ! Two coupled limits bound the squared path speed a = sdot^2:
    !   (1) kinematic: a <= (vmax_i/|q'_i|)^2 per axis.
    !   (2) dynamic feasibility: there must exist a path acceleration sdd with
    !       |q''_i a + q'_i sdd| <= amax_i for EVERY axis at once. Near a cusp an
    !       axis reverses (q'_i -> 0) so its q'_i*sdd term vanishes and feasibility
    !       collapses to |q''_i| a <= amax_i -- a hard cap on a that no sdd can
    !       rescue. The previous code applied only (1) and let sdd_bounds ignore
    !       near-zero-q' axes, so the curvature acceleration at (near-)cusps went
    !       unbounded and the posted feed exceeded the machine accel limit.
    ! We enforce (2) directly: cap a at each station to the largest value for
    ! which the sdd interval is non-empty (threshold-free; handles exact and
    ! near cusps uniformly).
    do k = 1, n
      abar(k) = huge(1.0_dp)
      do i = 1, ndof
        if (abs(qp(i, k)) > 1.0e-12_dp) then
          abar(k) = min(abar(k), (vmax(i) / abs(qp(i, k)))**2)
        end if
      end do
      abar(k) = accel_capped_a(qp(:, k), qpp(:, k), amax, ndof, abar(k))
    end do

    ! Feasible profile by ITERATED forward/backward clamping. A single forward
    ! and single backward pass (the classic scheme) leaves the profile slope
    ! infeasible near velocity cusps, because min(forward,backward,MVC) can
    ! create a segment steeper than the per-station acceleration interval allows.
    ! Each clamp only LOWERS aprof, so iterating to a fixed point is monotone,
    ! bounded below by 0, hence convergent -- and the limit satisfies both the
    ! acceleration (rising) and deceleration (falling) slope bounds everywhere.
    aprof = abar
    aprof(1) = min(aprof(1), a0)
    aprof(n) = min(aprof(n), aN)
    do it = 1, 200
      amaxchg = 0.0_dp
      ! forward: cap each rise to the max feasible acceleration at station k
      do k = 1, n - 1
        call sdd_bounds(qp(:, k), qpp(:, k), amax, ndof, aprof(k), sdd_lo, sdd_hi)
        anew = min(aprof(k+1), aprof(k) + 2.0_dp*ds*sdd_hi)
        if (anew < 0.0_dp) anew = 0.0_dp
        amaxchg = max(amaxchg, aprof(k+1) - anew)
        aprof(k+1) = anew
      end do
      ! backward: cap each (reverse) rise to the max feasible deceleration
      do k = n - 1, 1, -1
        call sdd_bounds(qp(:, k+1), qpp(:, k+1), amax, ndof, aprof(k+1), sdd_lo, sdd_hi)
        anew = min(aprof(k), aprof(k+1) - 2.0_dp*ds*sdd_lo)
        if (anew < 0.0_dp) anew = 0.0_dp
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

  !> Is there a path acceleration sdd with |q''_i a + q'_i sdd| <= amax_i for
  !> every axis i, at squared path speed a? (dynamic feasibility of the MVC).
  pure function accel_feasible(qp, qpp, amax, ndof, a) result(ok)
    integer,  intent(in) :: ndof
    real(dp), intent(in) :: qp(ndof), qpp(ndof), amax(ndof), a
    logical :: ok
    integer :: i
    real(dp) :: sdd_lo, sdd_hi, t1, t2
    ok = .true.
    sdd_lo = -huge(1.0_dp); sdd_hi = huge(1.0_dp)
    do i = 1, ndof
      if (abs(qp(i)) > 1.0e-12_dp) then
        t1 = (-amax(i) - qpp(i)*a) / qp(i)
        t2 = ( amax(i) - qpp(i)*a) / qp(i)
        sdd_lo = max(sdd_lo, min(t1, t2))
        sdd_hi = min(sdd_hi, max(t1, t2))
      else if (abs(qpp(i))*a > amax(i)) then
        ok = .false.; return            ! q'~0: |q''| a must not exceed amax
      end if
    end do
    if (sdd_hi < sdd_lo) ok = .false.
  end function accel_feasible

  !> Largest squared path speed <= a_hi that is dynamically feasible. Bisects on
  !> the (monotone) feasibility predicate; if a_hi is unbounded it is first
  !> seeded from the per-axis curvature caps amax_i/|q''_i|.
  pure function accel_capped_a(qp, qpp, amax, ndof, a_hi) result(a)
    integer,  intent(in) :: ndof
    real(dp), intent(in) :: qp(ndof), qpp(ndof), amax(ndof), a_hi
    real(dp) :: a, hi, lo, mid, seed
    integer  :: i, it
    hi = a_hi
    if (hi >= huge(1.0_dp)) then         ! no velocity cap (e.g. exact cusp)
      seed = 0.0_dp
      do i = 1, ndof
        if (abs(qpp(i)) > 1.0e-12_dp) seed = max(seed, amax(i)/abs(qpp(i)))
      end do
      if (seed <= 0.0_dp) then; a = a_hi; return; end if   ! straight, no curvature
      hi = seed
    end if
    if (accel_feasible(qp, qpp, amax, ndof, hi)) then; a = hi; return; end if
    lo = 0.0_dp
    do it = 1, 40
      mid = 0.5_dp*(lo + hi)
      if (accel_feasible(qp, qpp, amax, ndof, mid)) then; lo = mid; else; hi = mid; end if
    end do
    a = lo
  end function accel_capped_a

  !> Feasible second-derivative (sdd) interval at a station given speed^2 = a.
  subroutine sdd_bounds(qp, qpp, amax, ndof, a, sdd_lo, sdd_hi)
    integer,  intent(in)  :: ndof
    real(dp), intent(in)  :: qp(ndof), qpp(ndof), amax(ndof), a
    real(dp), intent(out) :: sdd_lo, sdd_hi
    integer :: i
    real(dp) :: lo_i, hi_i, t1, t2
    sdd_lo = -huge(1.0_dp)
    sdd_hi =  huge(1.0_dp)
    do i = 1, ndof
      if (abs(qp(i)) > 1.0e-12_dp) then
        t1 = (-amax(i) - qpp(i)*a) / qp(i)
        t2 = ( amax(i) - qpp(i)*a) / qp(i)
        lo_i = min(t1, t2)
        hi_i = max(t1, t2)
        sdd_lo = max(sdd_lo, lo_i)
        sdd_hi = min(sdd_hi, hi_i)
      end if
    end do
    if (sdd_hi < sdd_lo) then   ! degenerate: clamp
      sdd_hi = 0.0_dp; sdd_lo = 0.0_dp
    end if
  end subroutine sdd_bounds

end module topp_mod
