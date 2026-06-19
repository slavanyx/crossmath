!> Regenerative chatter stability lobes (single-DOF analytical model).
!>
!> For a tool-tip mode (natural freq wn, damping zeta, modal stiffness k) the
!> receptance is  G(w) = (1/k) / (1 - r^2 + i*2*zeta*r),  r = w/wn.
!> The limiting axial depth of cut and spindle speeds (Altintas) are
!>   a_lim = -1 / (2*Kt*N*Re[G])          (valid where Re[G] < 0, i.e. r > 1)
!>   psi   = atan2(Im[G], Re[G])
!>   eps   = pi - 2*psi
!>   n     = 60*wc / (N*(eps + 2*pi*lobe))      [rpm]
!> Sweeping the chatter frequency wc over r in (1, rmax] and lobe = 0..nlobes-1
!> traces the classic stability-lobe diagram.
module chatter_mod
  use vec3_mod, only: dp
  implicit none
  private
  public :: stability_lobes

contains

  subroutine stability_lobes(wn_hz, zeta, k_stiff, Kt, n_teeth, &
                             nlobes, nptsper, rpm, alim)
    real(dp), intent(in)  :: wn_hz, zeta, k_stiff, Kt
    integer,  intent(in)  :: n_teeth, nlobes, nptsper
    real(dp), intent(out) :: rpm(nlobes*nptsper), alim(nlobes*nptsper)
    real(dp), parameter :: pi = 3.14159265358979323846_dp
    real(dp) :: wn, r, reg, img, denom, psi, eps, wc, a
    integer  :: lobe, j, idx

    wn = 2.0_dp * pi * wn_hz       ! rad/s
    idx = 0
    do lobe = 0, nlobes - 1
      do j = 1, nptsper
        ! r ranges over (1, 2]; only r>1 gives Re[G]<0 (stable lobe region)
        r = 1.0_dp + 1.0_dp * real(j, dp) / real(nptsper, dp)
        denom = (1.0_dp - r*r)**2 + (2.0_dp*zeta*r)**2
        reg = (1.0_dp / k_stiff) * (1.0_dp - r*r) / denom
        img = (1.0_dp / k_stiff) * (-2.0_dp*zeta*r) / denom
        wc = r * wn
        psi = atan2(img, reg)
        eps = pi - 2.0_dp*psi
        a = -1.0_dp / (2.0_dp * Kt * real(n_teeth, dp) * reg)
        idx = idx + 1
        alim(idx) = a
        rpm(idx) = 60.0_dp * wc / (real(n_teeth, dp) * (eps + 2.0_dp*pi*lobe))
      end do
    end do
  end subroutine stability_lobes

end module chatter_mod
