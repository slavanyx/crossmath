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
  public :: stability_lobes, stability_lobes_frf

contains

  !> Stability lobes from a MEASURED tool-tip receptance G(f)=reg+i*img (mm/N),
  !> sampled at nf frequencies (Hz). Same regenerative relations as the modal
  !> model but G comes from a tap test instead of a single mode. Output arrays
  !> have length nlobes*nf; entries where Re[G] >= 0 (no chatter) are set NaN.
  subroutine stability_lobes_frf(freq, reg, img, nf, Kt, n_teeth, nlobes, &
                                 rpm, alim)
    use, intrinsic :: ieee_arithmetic, only: ieee_value, ieee_quiet_nan
    real(dp), intent(in)  :: freq(nf), reg(nf), img(nf), Kt
    integer,  intent(in)  :: nf, n_teeth, nlobes
    real(dp), intent(out) :: rpm(nlobes*nf), alim(nlobes*nf)
    real(dp), parameter :: pi = 3.14159265358979323846_dp
    real(dp) :: wc, psi, eps, nan
    integer  :: lobe, j, idx
    nan = ieee_value(1.0_dp, ieee_quiet_nan)
    idx = 0
    do lobe = 0, nlobes - 1
      do j = 1, nf
        idx = idx + 1
        if (reg(j) < 0.0_dp) then
          wc = 2.0_dp * pi * freq(j)
          psi = atan2(img(j), reg(j))
          eps = modulo(pi - 2.0_dp*psi, 2.0_dp*pi)
          alim(idx) = -1.0_dp / (2.0_dp * Kt * real(n_teeth, dp) * reg(j))
          rpm(idx) = 60.0_dp * wc / (real(n_teeth, dp) * (eps + 2.0_dp*pi*lobe))
        else
          alim(idx) = nan
          rpm(idx) = nan
        end if
      end do
    end do
  end subroutine stability_lobes_frf

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
        ! phase shift between successive teeth, reduced to [0, 2*pi) so that
        ! lobe index k=0 is the highest-speed (high-speed-machining) lobe.
        eps = modulo(pi - 2.0_dp*psi, 2.0_dp*pi)
        a = -1.0_dp / (2.0_dp * Kt * real(n_teeth, dp) * reg)
        idx = idx + 1
        alim(idx) = a
        rpm(idx) = 60.0_dp * wc / (real(n_teeth, dp) * (eps + 2.0_dp*pi*lobe))
      end do
    end do
  end subroutine stability_lobes

end module chatter_mod
