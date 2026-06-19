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
    real(dp) :: qp(ndof, n), qpp(ndof, n), abar(n), af(n), ab(n)
    real(dp) :: ds, lo, hi, sdd_lo, sdd_hi, denom
    integer  :: k, i

    ds = 1.0_dp / real(n - 1, dp)

    ! finite-difference path derivatives in s
    do k = 1, n
      if (k == 1) then
        qp(:, k)  = (q(:, 2) - q(:, 1)) / ds
        qpp(:, k) = (q(:, 3) - 2.0_dp*q(:, 2) + q(:, 1)) / ds**2
      else if (k == n) then
        qp(:, k)  = (q(:, n) - q(:, n-1)) / ds
        qpp(:, k) = (q(:, n) - 2.0_dp*q(:, n-1) + q(:, n-2)) / ds**2
      else
        qp(:, k)  = (q(:, k+1) - q(:, k-1)) / (2.0_dp*ds)
        qpp(:, k) = (q(:, k+1) - 2.0_dp*q(:, k) + q(:, k-1)) / ds**2
      end if
    end do

    ! velocity-limit curve
    do k = 1, n
      abar(k) = huge(1.0_dp)
      do i = 1, ndof
        if (abs(qp(i, k)) > 1.0e-12_dp) then
          abar(k) = min(abar(k), (vmax(i) / abs(qp(i, k)))**2)
        end if
      end do
    end do

    ! forward pass: maximum acceleration
    af(1) = min(a0, abar(1))
    do k = 1, n - 1
      call sdd_bounds(qp(:, k), qpp(:, k), amax, ndof, af(k), sdd_lo, sdd_hi)
      af(k+1) = min(abar(k+1), af(k) + 2.0_dp*ds*sdd_hi)
      if (af(k+1) < 0.0_dp) af(k+1) = 0.0_dp
    end do

    ! backward pass: maximum deceleration (evaluate bounds at k+1, semi-implicit)
    ab(n) = min(aN, abar(n))
    do k = n - 1, 1, -1
      call sdd_bounds(qp(:, k+1), qpp(:, k+1), amax, ndof, ab(k+1), sdd_lo, sdd_hi)
      ab(k) = min(abar(k), ab(k+1) - 2.0_dp*ds*sdd_lo)
    end do

    ! feasible profile = min of the two passes and the velocity cap
    do k = 1, n
      aprof(k) = max(0.0_dp, min(af(k), ab(k), abar(k)))
    end do

    ! integrate time: dt = ds / sdot, trapezoid on 1/sqrt(a)
    ttotal = 0.0_dp
    do k = 1, n - 1
      denom = sqrt(aprof(k)) + sqrt(aprof(k+1))
      if (denom > 1.0e-12_dp) ttotal = ttotal + 2.0_dp*ds / denom
    end do
  end subroutine topp_ra

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
