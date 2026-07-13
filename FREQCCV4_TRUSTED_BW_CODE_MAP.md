# FreqCCv4 TrustedBw Code Map

- Window construction and independent spectral scores:
  `FreqCCv4Sender::BuildCruiseWindowResult()`.
- Strict minimum aggregation and independent Delivery Rate/SRTT gates:
  the spectral-integrity block in `BuildCruiseWindowResult()`.
- NORMAL/MERGED eligible-window selection:
  `FreqCCv4Sender::RunTrustedBwSelection()`.
- Publication and lifecycle:
  `PublishTrustedBwSelection()`, `ClearTrustedBwApplication()`, and
  `ClearTrustedBw()`.
- Direct REFILL/UP/DOWN pacing baseline:
  `FreqCCv4Sender::PacingRate()`.
- CSV schemas:
  `DqcTrace::OpenFreqCCv4LoadFile()`,
  `OpenFreqCCv4CruiseSummaryFile()`, and
  `OpenFreqCCv4GateFile()`.
- Configuration parsing:
  `FreqCCv4Config`, `ConfigureFreqCCv4()`, and the scenario parsers.

TrustedBw is an auxiliary pacing-only value. Native BBR bandwidth estimation,
MaxBw filtering, BDP, congestion window, and inflight state continue to use the
native model.
